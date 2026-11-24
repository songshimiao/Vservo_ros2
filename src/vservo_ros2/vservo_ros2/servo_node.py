# 伺服目标节点：选中的 base_T_target → 期望 base_T_flange。
# 以 50 Hz 发布完整 desired 目标及兼容微步预览；本节点不调用机器人 SDK。
import json
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from .geometry import pose_matrix, tf_matrix, read_json, validate, fill_pose, error
from .control import step, target_flange, in_workspace
from .ros_utils import param, spin, fresh, LATCHED


# 连接定位状态、两种目标流、手眼矩阵、当前法兰 TF 与速度倍率。
class ServoNode(Node):
    # 要求已验证的 EIH 手眼；tracking_frame 决定距离约束作用于相机中心还是法兰中心。
    def __init__(self):
        super().__init__('servo')
        data = read_json(param(self, 'eih_handeye', ''))
        if data.get('mode') != 'eih' or not data.get('validated'):
            raise ValueError('Need validated EIH hand-eye calibration')
        self.X = validate(data['matrix'])
        self.selection_policy = param(self, 'selection_policy', 'first_seen')
        self.tracking_frame = param(self, 'tracking_frame', 'camera')
        if self.tracking_frame not in ('camera', 'flange'):
            raise ValueError('tracking_frame must be camera or flange')
        self.coarse = param(self, 'coarse_distance_m', 0.40)
        self.fine = param(self, 'fine_distance_m', 0.30)
        if not 0 < self.fine <= self.coarse:
            raise ValueError('Require 0 < fine_distance_m <= coarse_distance_m')
        self.minimum = param(self, 'workspace_min', [-0.6, -0.6, 0.1])
        self.maximum = param(self, 'workspace_max', [0.6, 0.6, 0.8])
        self.max_age = param(self, 'max_age_s', 0.3)
        self.targets = {}
        self.speed_scale = 1.0
        self.state_stamp = None
        self.state = 'HOLD'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseStamped, '/servo/next_flange_pose', 1)
        self.desired_pub = self.create_publisher(PoseStamped, '/servo/desired_flange_pose', 1)
        self.status = self.create_publisher(String, '/servo/status', 5)
        for mode, topic in (('COARSE', '/localization/coarse_target_pose'), ('FINE', '/localization/fine_target_pose')):
            self.create_subscription(PoseStamped, topic, lambda m, key=mode: self.targets.update({key: m}), 1)
        self.create_subscription(String, '/localization/state', self.on_state, 1)
        self.create_subscription(String, '/robot/speed_status', self.on_speed, LATCHED)
        self.create_timer(0.02, self.tick)

    # 从桥接确认的速度状态更新预览倍率，忽略格式错误或越界数据。
    def on_speed(self, msg):
        try:
            data = json.loads(msg.data)
            scale = data['percent']/100
            if np.isfinite(scale) and 0.1 <= scale <= 1.0:
                self.speed_scale = scale
        except (ValueError, KeyError, TypeError):
            pass

    # 切换状态时清除旧模式目标，避免相机交接后继续使用上一模式缓存。
    def on_state(self, msg):
        if msg.data != self.state:
            self.targets.pop(self.state, None)
        self.state, self.state_stamp = msg.data, self.get_clock().now()

    # 核对状态心跳、视觉时间戳和机器人 TF，计算完整法兰目标并检查工作空间。
    # 目标消息保持视觉观测时间；反复 tick 不能把陈旧视觉刷新成有效目标。
    def tick(self):
        try:
            now = self.get_clock().now()
            if self.state not in ('COARSE', 'FINE') or self.state_stamp is None or (now-self.state_stamp).nanoseconds > 200_000_000:
                if self.state == 'WAITING_TF':
                    raise ValueError('WAITING_TF: target visible; waiting for image-time camera transform')
                raise ValueError('HOLD: no fresh localization state')
            target = self.targets.get(self.state)
            if target is None or target.header.frame_id != 'base' or not fresh(self, target.header.stamp, self.max_age):
                raise ValueError('HOLD: no fresh base target')
            current_tf = self.tf.lookup_transform('base', 'flange', Time())
            if not fresh(self, current_tf.header.stamp, 0.1):
                raise ValueError('HOLD: stale robot state')
            current = tf_matrix(current_tf)
            # first_seen / eth_primary 两种来源都使用 fine 距离；只有历史 handoff 的 COARSE 使用 coarse 距离。
            distance = self.fine if self.state == 'FINE' or self.selection_policy != 'handoff' else self.coarse
            # 跟踪法兰时用单位矩阵，直接让法兰中心位于目标 -Z 的指定距离。
            extrinsic = self.X if self.tracking_frame == 'camera' else np.eye(4)
            desired = target_flange(pose_matrix(target.pose), extrinsic, distance)
            if not in_workspace(desired, self.minimum, self.maximum):
                raise ValueError('HOLD: desired flange outside workspace')
            t, a = error(current, desired)
            # Preserve the legacy preview contract. The bridge follows the full
            # desired pose below with its own live Cartesian/joint limits.
            next_pose = step(current, desired, max_translation=0.001*self.speed_scale,
                             max_rotation=0.01*self.speed_scale)
            out = PoseStamped()
            out.header.frame_id = 'base'
            # Preserve observation time: repeated ticks cannot refresh old vision.
            out.header.stamp = target.header.stamp
            fill_pose(out.pose, next_pose)
            full_target = PoseStamped()
            full_target.header = out.header
            fill_pose(full_target.pose, desired)
            self.desired_pub.publish(full_target)
            if t < 0.001 and a < np.deg2rad(0.5):
                self.status.publish(String(data=f'TRACKING {self.state}: within tolerance; {self.tracking_frame}={distance*1000:.0f}mm'))
            else:
                self.status.publish(String(data=f'{self.state}: position={t:.5f}m rotation={np.degrees(a):.2f}deg'))
            self.pub.publish(out)
        except Exception as exc:
            self.status.publish(String(data=str(exc)))


# 运行目标计算节点，实体控制的使能与限制仍由 robot_bridge 负责。
def main(args=None):
    spin(ServoNode, args)
