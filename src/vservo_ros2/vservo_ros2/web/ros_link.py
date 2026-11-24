"""ROS subscriptions and Trigger clients. JPEG encoding is outside the executor."""
from collections import deque
import json
import math
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from ..ros_utils import LATCHED, stamp_s
from .workspace import ROLES, service_result
from .target_overlay import TARGET_MAX_AGE, tracking_id42, matched_target_frame, draw_target_axes

FRAME_TIMEOUT = 1.5
WARN_LEVEL = int.from_bytes(Log.WARN, 'big') if isinstance(Log.WARN, bytes) else Log.WARN


def camera_status(frames, dimensions, metadata, now, ros_now):
    last = frames[-1] if frames else None
    age = ros_now - last[1] if last else None
    online = bool(last and now-last[0] <= FRAME_TIMEOUT and 0 <= age <= FRAME_TIMEOUT)
    hz = (len(frames)-1)/(frames[-1][0]-frames[0][0]) if online and len(frames) > 1 and frames[-1][0] > frames[0][0] else 0
    return dict(online=online, size=dimensions, hz=round(hz, 1), age_s=age,
                stamp=last[1] if last else None, metadata=metadata)


def pose_fields(msg):
    p, q = msg.pose.position, msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not all(map(math.isfinite, values)):
        return None
    return dict(frame=msg.header.frame_id, position_m=values[:3], quaternion_xyzw=values[3:],
                stamp=stamp_s(msg.header.stamp))


class RosLink(Node):
    def __init__(self, preview_fps):
        super().__init__('vservo_web_console')
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.bridge = CvBridge()
        self.frames = {r: deque(maxlen=100) for r in ROLES}
        self.sizes = {}
        self.raw = {}
        self.image_history = {r: deque(maxlen=12) for r in ROLES}
        self.target_poses = {}
        self.camera_info = {}
        self.servo_axes = {}
        self.jpegs = {}
        self.metadata = {}
        self.values = {}
        self.trigger_clients = {}
        self.arm_client = self.create_client(SetBool, '/robot_bridge/arm')
        self.speed_client = self.create_client(SetParametersAtomically, '/robot_bridge/set_parameters_atomically')
        self.inflight = {}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        for role in ROLES:
            self.create_subscription(Image, f'/{role}/image_raw',
                                     lambda m, r=role: self.image(r, m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{role}/camera_info',
                                     lambda m, r=role: self.intrinsics(r, m), LATCHED)
            self.create_subscription(PoseWithCovarianceStamped, f'/{role}/target_pose',
                                     lambda m, r=role: self.target_pose(r, m), 1)
            self.create_subscription(String, f'/{role}/metadata',
                                     lambda m, r=role: self.meta(r, m.data), LATCHED)
            self.create_subscription(String, f'/{role}/calibration/preview_status',
                                     lambda m, r=role: self.value(f'{r}_preview', self.decode(m.data)), 1)
            self.create_subscription(CompressedImage, f'/{role}/calibration/preview_image',
                                     lambda m, r=role: self.overlay(r, m), qos_profile_sensor_data)
            for action in ('capture', 'solve'):
                self.trigger_clients[role, action] = self.create_client(Trigger, f'/{role}/calibration/{action}')
        for key, topic in [('localization', '/localization/state'), ('servo', '/servo/status'),
                           ('robot', '/robot/status'), ('eih_detector', '/eih/detector_status'),
                           ('eth_detector', '/eth/detector_status')]:
            self.create_subscription(String, topic, lambda m, k=key: self.value(k, m.data), 1)
        self.create_subscription(String, '/localization/diagnostics',
                                 lambda m: self.value('agreement', self.decode(m.data)), 1)
        self.create_subscription(String, '/robot/speed_status',
                                 lambda m: self.value('speed', self.decode(m.data)), LATCHED)
        self.create_subscription(String, '/robot/servo_state',
                                 lambda m: self.value('follow', self.decode(m.data)), LATCHED)
        for key, topic in [('target', '/localization/target_pose'), ('next_flange', '/servo/next_flange_pose')]:
            self.create_subscription(PoseStamped, topic, lambda m, k=key: self.value(k, pose_fields(m)), 1)
        self.create_subscription(Log, '/rosout', self.camera_log, 20)
        self.encoder = threading.Thread(target=self.encode, args=(1/preview_fps,), daemon=True)
        self.encoder.start()

    def camera_log(self, msg):
        role = next((r for r in ROLES if msg.name == f'{r}.camera'), None)
        if role and msg.level >= WARN_LEVEL:
            with self.lock:
                self.values[f'{role}_camera_log'] = (time.monotonic(), msg.msg)

    @staticmethod
    def decode(raw):
        try:
            data = json.loads(raw, parse_constant=lambda x: None)
            return data if isinstance(data, dict) else None
        except (ValueError, RecursionError):
            return None

    def meta(self, role, raw):
        with self.lock:
            self.metadata[role] = self.decode(raw)

    def value(self, key, value):
        with self.lock:
            self.values[key] = (time.monotonic(), value)

    def image(self, role, msg):
        now = time.monotonic()
        with self.lock:
            frames = self.frames[role]
            frames.append((now, stamp_s(msg.header.stamp)))
            while frames and now-frames[0][0] > 2:
                frames.popleft()
            self.sizes[role] = [msg.width, msg.height]
            self.raw[role] = (now, msg)
            self.image_history[role].append((now, msg))
            while self.image_history[role] and now-self.image_history[role][0][0] > 0.4:
                self.image_history[role].popleft()

    def intrinsics(self, role, msg):
        with self.lock:
            self.camera_info[role] = msg

    def target_pose(self, role, msg):
        if msg.header.frame_id == f'{role}_optical_frame':
            with self.lock:
                self.target_poses[role] = (time.monotonic(), msg)

    def overlay(self, role, msg):
        if msg.format == 'jpeg':
            with self.lock:
                self.jpegs[role, 'overlay'] = (time.monotonic(), stamp_s(msg.header.stamp), bytes(msg.data))

    def encode(self, period):
        previous = {}
        previous_servo = {}
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock:
                latest = self.raw.copy()
                histories = {r: list(frames) for r, frames in self.image_history.items()}
                poses, infos, values = self.target_poses.copy(), self.camera_info.copy(), self.values.copy()
            ros_now = self.get_clock().now().nanoseconds/1e9
            preview_now = time.monotonic()
            for role, (received, msg) in latest.items():
                if started-received > FRAME_TIMEOUT:
                    continue
                try:
                    if previous.get(role) != received:
                        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                        if ok:
                            with self.lock:
                                self.jpegs[role, 'raw'] = (received, stamp_s(msg.header.stamp), jpeg.tobytes())
                        previous[role] = received
                    match = matched_target_frame(histories[role], poses.get(role), infos.get(role),
                                                 values.get(f'{role}_detector'), preview_now, ros_now)
                    source_received, source, pose = match if match else (received, msg, None)
                    key = (source_received, pose is not None)
                    if previous_servo.get(role) == key:
                        continue
                    if pose is None:
                        with self.lock:
                            raw_frame = self.jpegs.get((role, 'raw'))
                            if raw_frame and raw_frame[0] == source_received:
                                self.jpegs[role, 'servo'] = raw_frame
                                self.servo_axes[role] = None
                                previous_servo[role] = key
                                continue
                    bgr = self.bridge.imgmsg_to_cv2(source, 'bgr8')
                    axes_stamp = None
                    if pose is not None:
                        try:
                            bgr = draw_target_axes(bgr, pose, infos[role])
                            axes_stamp = stamp_s(pose.header.stamp)
                        except (ValueError, cv2.error, RuntimeError):
                            pass  # Projection failure must not interrupt live video.
                    ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        with self.lock:
                            self.jpegs[role, 'servo'] = (source_received, stamp_s(source.header.stamp), jpeg.tobytes())
                            self.servo_axes[role] = axes_stamp
                        previous_servo[role] = key
                except (ValueError, cv2.error, RuntimeError) as exc:
                    self.get_logger().warning(f'JPEG {role}: {exc}')
            self.stop.wait(max(0, period-(time.monotonic()-started)))

    def snapshot(self):
        with self.lock:
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
            cameras = {r: camera_status(self.frames[r], self.sizes.get(r), self.metadata.get(r), now, ros_now)
                       for r in ROLES}
            values = {k: v for k, (t, v) in self.values.items() if now-t <= (120 if k.endswith('_camera_log') else FRAME_TIMEOUT)}
        for key in ('target', 'next_flange'):
            pose = values.get(key)
            if pose and not 0 <= ros_now-pose['stamp'] <= 0.3:
                values.pop(key)
        services = {r: {a: self.trigger_clients[r, a].service_is_ready() for a in ('capture', 'solve')} for r in ROLES}
        try:
            stamp = stamp_s(self.tf.lookup_transform('base', 'flange', Time()).header.stamp)
            age = ros_now-stamp
            tf = dict(ok=0 <= age <= 0.3, age_s=age)
        except Exception as exc:
            tf = dict(ok=False, reason=str(exc))
        nodes = self.get_node_names_and_namespaces()
        calibration_nodes = {r: nodes.count(('calibration', f'/{r}')) for r in ROLES}
        return dict(cameras=cameras, values=values, services=services, tf=tf,
                    arm_available=self.arm_client.service_is_ready(),
                    speed_available=self.speed_client.service_is_ready(),
                    calibration_nodes=calibration_nodes, ros_now=ros_now)

    def jpeg(self, role, variant):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
        with self.lock:
            frame = self.jpegs.get((role, variant))
            axes_stamp = self.servo_axes.get(role) if variant == 'servo' else None
            if axes_stamp is not None and (not 0 <= ros_now-axes_stamp <= TARGET_MAX_AGE
                    or not tracking_id42(self.values.get(f'{role}_detector'), now)):
                frame = self.jpegs.get((role, 'raw'))
        if frame and time.monotonic()-frame[0] <= FRAME_TIMEOUT:
            age = ros_now-frame[1]
            if 0 <= age <= FRAME_TIMEOUT:
                return frame[2]
        return None

    def trigger(self, role, action, timeout=40):
        key = role, action
        return RosLink.call(self, role, self.trigger_clients[key], Trigger.Request(), timeout)

    def arm(self, enabled):
        return self.call('robot', self.arm_client, SetBool.Request(data=enabled), 3)

    def set_speed(self, percent, profile=None):
        values = dict(speed_scale=percent/100)
        if profile is not None:
            values.update(linear_speed_m_s=profile['linear_mm_s']/1000,
                          angular_speed_rad_s=math.radians(profile['angular_deg_s']),
                          max_joint_accel_rad_s2=profile['joint_accel_rad_s2'])
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            for name, value in values.items()])
        return self.call('speed', self.speed_client, request, 3, convert=lambda response: dict(
            success=response.result.successful, message=response.result.reason or '运动速度已应用'))

    def call(self, group, client, request, timeout, convert=service_result):
        with self.lock:
            pending = self.inflight.get(group)
            if pending is not None and not pending.done():
                return dict(success=False, message='上一次服务调用仍在等待 ROS 响应')
            if not client.service_is_ready():
                return dict(success=False, message='ROS 服务不可用')
            future = client.call_async(request)
            self.inflight[group] = future
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            return dict(success=False, message='等待 ROS 响应超时。请求可能仍在执行，请查看会话和节点日志。')
        try:
            return convert(future.result())
        except Exception as exc:
            return dict(success=False, message=str(exc))

    def close(self):
        self.stop.set()
        self.encoder.join()
