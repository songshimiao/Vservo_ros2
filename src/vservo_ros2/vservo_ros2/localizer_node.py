# 定位节点：图像时刻的 base_T_camera × camera_T_target → base_T_target。
# 再由 Selector 选择主相机，20 Hz 发布状态心跳；WAITING_TF 区分等待变换与真正双路丢失。
import json
import numpy as np
from collections import deque
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from .geometry import pose_matrix, tf_matrix, fill_pose
from .localization import Selector
from .ros_utils import param, spin, stamp_s, fresh


# 连接双相机观测、TF 缓冲和纯算法选择器，输出统一的基座系目标。
class LocalizerNode(Node):
    # 装载策略参数，为每路创建最多 10 帧的待转换队列，订阅位姿及可见性。
    def __init__(self):
        super().__init__('localizer')
        self.selector = Selector(max_age=param(self, 'max_age_s', 0.3),
                                 pair_skew=param(self, 'pair_skew_s', 0.10),
                                 agreement_m=param(self, 'agreement_m', 0.04),
                                 agreement_deg=param(self, 'agreement_deg', 12.0),
                                 enter_m=param(self, 'fine_enter_m', 0.45),
                                 leave_m=param(self, 'fine_leave_m', 0.55),
                                 handoff_timeout=param(self, 'handoff_timeout_s', 1.0),
                                 require_pair_agreement=param(self, 'require_pair_agreement', False),
                                 selection_policy=param(self, 'selection_policy', 'first_seen'),
                                 target_loss_timeout=param(self, 'target_loss_timeout_s', 2.0),
                                 reacquire_delay=param(self, 'reacquire_delay_s', 0.5))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending = {r: deque(maxlen=10) for r in ('eih', 'eth')}
        self.observed_stamp = {}
        self.tf_errors = {}
        self.observation_errors = {}
        self.invalid_stamp = {}
        self.pub = self.create_publisher(PoseStamped, '/localization/target_pose', 5)
        self.mode_targets = {mode: self.create_publisher(PoseStamped, topic, 1) for mode, topic in (
            ('COARSE', '/localization/coarse_target_pose'), ('FINE', '/localization/fine_target_pose'))}
        self.status = self.create_publisher(String, '/localization/state', 5)
        self.diagnostics = self.create_publisher(String, '/localization/diagnostics', 1)
        for role in ('eih', 'eth'):
            self.create_subscription(PoseWithCovarianceStamped, f'/{role}/target_pose',
                                     lambda m, r=role: self.observe(r, m), 5)
            self.create_subscription(String, f'/{role}/target_visibility',
                                     lambda m, r=role: self.on_visibility(r, m), 1)
        self.create_timer(0.05, self.tick)

    # 检查原始观测时效、坐标系、协方差与正深度，通过后排队等待对应图像时刻的 TF。
    def observe(self, role, msg):
        try:
            if not fresh(self, msg.header.stamp, self.selector.max_age) or msg.header.frame_id != f'{role}_optical_frame':
                raise ValueError('Observation stale / wrong frame')
            if stamp_s(msg.header.stamp) <= self.invalid_stamp.get(role, -np.inf):
                return  # Do not revive a frame already rejected by detection.
            cov = np.asarray(msg.pose.covariance).reshape(6, 6)
            if (not np.isfinite(cov).all() or np.any(np.diag(cov) < 0)
                    or max(np.diag(cov)[:3]) > 0.02**2 or max(np.diag(cov)[3:]) > np.deg2rad(10)**2):
                raise ValueError('Observation uncertainty too large')
            camera_T_target = pose_matrix(msg.pose.pose)
            if camera_T_target[2, 3] <= 0:
                raise ValueError('Negative depth')
            self.observed_stamp[role] = max(stamp_s(msg.header.stamp), self.observed_stamp.get(role, -np.inf))
            self.observation_errors[role] = ''
            self.pending[role].append((msg.header, camera_T_target))
        except Exception as exc:
            self.observation_errors[role] = str(exc)
            self.get_logger().debug(str(exc))

    # 只用新鲜的负向可见性包作废旧样本；正向包不建立目标，必须另有有效米制位姿。
    # 按时间戳删除旧样本与待处理帧，迟到的丢失包不能清除较新的目标。
    def on_visibility(self, role, msg):
        try:
            data = json.loads(msg.data)
            stamp_ns, visible = data['stamp_ns'], data['visible']
            if (type(stamp_ns) is not int or type(visible) is not bool
                    or data['frame_id'] != f'{role}_optical_frame'):
                return
            stamp = stamp_ns//1_000_000_000+(stamp_ns % 1_000_000_000)*1e-9
            now = self.get_clock().now().nanoseconds/1e9
            if not 0 <= now-stamp <= self.selector.max_age or visible or stamp <= self.invalid_stamp.get(role, -np.inf):
                return
            # Positive state packets never establish a pose. A delayed negative
            # packet may remove older frames, but cannot erase a newer target.
            self.invalid_stamp[role] = stamp
            self.selector.invalidate(role, stamp)
            self.pending[role] = deque((item for item in self.pending[role] if stamp_s(item[0].stamp) > stamp), maxlen=10)
            if self.observed_stamp.get(role, -np.inf) <= stamp:
                self.observed_stamp.pop(role, None)
                self.observation_errors[role] = data.get('reason', 'Target lost')
        except (ValueError, TypeError, KeyError):
            return

    # 重试待处理帧的历史 TF，更新选择器、每路诊断和状态，再发布有原始时间戳的选中目标。
    def tick(self):
        # Dynamic robot TF may arrive just after an image. Retain the original
        # image time and retry briefly, rather than substituting latest TF.
        for role, queue in self.pending.items():
            waiting = []
            while queue:
                header, camera_T_target = queue.popleft()
                if not fresh(self, header.stamp, self.selector.max_age):
                    continue
                try:
                    base_T_camera = tf_matrix(self.tf.lookup_transform('base', header.frame_id, Time.from_msg(header.stamp)))
                    self.selector.update(role, stamp_s(header.stamp), base_T_camera @ camera_T_target,
                                         np.linalg.norm(camera_T_target[:3, 3]))
                    self.tf_errors[role] = ''
                except Exception as exc:
                    self.tf_errors[role] = str(exc)[:240]
                    waiting.append((header, camera_T_target))
            queue.extend(waiting)
        now = self.get_clock().now().nanoseconds * 1e-9
        state, sample = self.selector.choose(now)
        cameras = {}
        for role in self.pending:
            raw = self.observed_stamp.get(role)
            base = self.selector.samples.get(role)
            visible = raw is not None and 0 <= now-raw <= self.selector.max_age
            localized = base is not None and 0 <= now-base[0] <= self.selector.max_age
            cameras[role] = dict(visible=visible, localized=localized,
                                 observation_age_s=None if raw is None else now-raw,
                                 base_age_s=None if base is None else now-base[0],
                                 tf_ready=bool(self.tf.can_transform('base', f'{role}_optical_frame', Time())),
                                 pending=len(self.pending[role]),
                                 error=self.tf_errors.get(role, '') if visible and not localized
                                 else self.observation_errors.get(role, '') if not visible else '')
        # A visible target awaiting its image-time transform is not a loss of
        # both camera views. Hold motion while retrying TF; never start homing.
        # 相机看见目标但坐标变换尚未就绪时输出 WAITING_TF，机器人桥等待指令且不会启动丢失复位。
        if state == 'HOLD' and any(c['visible'] for c in cameras.values()) and not any(c['localized'] for c in cameras.values()):
            state = 'WAITING_TF'
            self.selector.diagnostics.update(reason='waiting_tf', source=None)
        self.selector.diagnostics['cameras'] = cameras
        self.status.publish(String(data=state))
        if self.diagnostics.get_subscription_count():
            self.diagnostics.publish(String(data=json.dumps(self.selector.diagnostics)))
        if sample is not None:
            msg = PoseStamped()
            msg.header.frame_id = 'base'
            msg.header.stamp = Time(nanoseconds=round(sample[0]*1e9)).to_msg()
            fill_pose(msg.pose, sample[1])
            self.mode_targets[state].publish(msg)
            self.pub.publish(msg)


# 运行定位节点；没有有效观测时仍发布心跳状态，供机器人桥识别在线 HOLD。
def main(args=None):
    spin(LocalizerNode, args)
