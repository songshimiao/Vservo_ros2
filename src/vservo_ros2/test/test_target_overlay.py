"""ID42 overlays use validated, timestamp-matched detections; no robot SDK."""
import json
import time
import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from vservo_ros2.geometry import transform, fill_pose
from vservo_ros2.ros_utils import LATCHED
from vservo_ros2.detector_node import DetectorNode
from vservo_ros2.web.ros_link import RosLink
from vservo_ros2.web.target_overlay import draw_target_axes, matched_target_frame


def preview_messages():
    image = np.full((480, 640, 3), 255, np.uint8)
    frame = CvBridge().cv2_to_imgmsg(image, 'bgr8')
    frame.header.frame_id = 'eih_optical_frame'
    frame.header.stamp.sec = 5
    info = CameraInfo(width=640, height=480, k=[500., 0, 320, 0, 500., 240, 0, 0, 1.], d=[0.]*5)
    info.header.frame_id = frame.header.frame_id
    pose = PoseWithCovarianceStamped()
    pose.header = frame.header
    fill_pose(pose.pose.pose, transform(t=[0.08, 0.02, 0.4]))
    return image, frame, info, pose


def test_draw_axes_preserves_raw_frame_and_uses_into_marker_positive_z():
    raw, _, info, pose = preview_messages()
    drawn = draw_target_axes(raw, pose, info)
    assert np.all(raw == 255)
    for channel in range(3):
        other = [c for c in range(3) if c != channel]
        assert np.count_nonzero((drawn[:, :, channel] > 150)
                                & (drawn[:, :, other[0]] < 100)
                                & (drawn[:, :, other[1]] < 100)) > 5
    # +Z extends from z=0.4 to 0.44 m, toward the principal point in this view.
    x, y = round(320+500*0.08/0.44), round(240+500*0.02/0.44)
    patch = drawn[y-2:y+3, x-2:x+3]
    assert np.any((patch[:, :, 0] > 150) & (patch[:, :, 1] < 100) & (patch[:, :, 2] < 100))


@pytest.mark.parametrize('condition', ['valid', 'stale', 'lost', 'other_id', 'wrong_frame', 'other_stamp', 'other_size'])
def test_axes_require_fresh_id42_and_exact_source_frame(condition):
    _, frame, info, pose = preview_messages()
    # Give pose its own header so timestamp/frame mismatch cannot mutate image.
    from copy import deepcopy
    pose.header = deepcopy(frame.header)
    status, ros_now = (10.0, 'TRACKING aruco_42 (0.10 px)'), 5.1
    if condition == 'stale': ros_now = 6.0
    if condition == 'lost': status = (10.0, 'INVALID: Need exactly one ArUco ID 42')
    if condition == 'other_id': status = (10.0, 'TRACKING aruco_41 (0.10 px)')
    if condition == 'wrong_frame': pose.header.frame_id = 'base'
    if condition == 'other_stamp': pose.header.stamp.nanosec = 1
    if condition == 'other_size': info.width = 320
    result = matched_target_frame([(10.0, frame)], (10.0, pose), info, status, 10.1, ros_now)
    assert (result is not None) == (condition == 'valid')


@pytest.mark.parametrize('role', ['eih', 'eth'])
def test_real_detector_to_web_overlay_and_loss_clear_axes(role):
    rclpy.init(args=['--ros-args', '-p', f'role:={role}', '-p', 'target_id:=aruco_42'])
    detector, link, fixture = DetectorNode(), RosLink(5), Node('axes_image_fixture')
    image_pub = fixture.create_publisher(Image, f'/{role}/image_raw', qos_profile_sensor_data)
    info_pub = fixture.create_publisher(CameraInfo, f'/{role}/camera_info', LATCHED)
    meta_pub = fixture.create_publisher(String, f'/{role}/metadata', LATCHED)
    executor = SingleThreadedExecutor()
    nodes = [detector, link, fixture]
    for node in nodes: executor.add_node(node)
    _, _, info, _ = preview_messages()
    info.header.frame_id = f'{role}_optical_frame'
    info_pub.publish(info)
    meta_pub.publish(String(data=json.dumps(dict(validated=True))))
    K, D = np.asarray(info.k).reshape(3, 3), np.zeros(5)
    target = transform(cv2.Rodrigues(np.array([0.25, -0.30, 0.10]))[0], [0.02, -0.01, 0.4])
    raw_rotation = target[:3, :3] @ np.diag([1, -1, -1])
    pixels = cv2.projectPoints(detector.detector.points, cv2.Rodrigues(raw_rotation)[0],
                              target[:3, 3], K, D)[0].reshape(4, 2)
    bitmap = cv2.aruco.generateImageMarker(detector.detector.dictionary, 42, 240)
    corners = np.array([[0, 0], [239, 0], [239, 239], [0, 239]], np.float32)
    image = cv2.warpPerspective(bitmap, cv2.getPerspectiveTransform(corners, pixels),
                                (640, 480), borderValue=255)
    bridge = CvBridge()
    def colored(bgr):
        return np.count_nonzero(np.max(bgr, axis=2).astype(int)-np.min(bgr, axis=2).astype(int) > 60)
    try:
        for visible in (True, False):
            deadline, last_sent, passed = time.monotonic()+6, 0, False
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.003)
                if time.monotonic()-last_sent > 0.07:
                    gray = image if visible else np.full_like(image, 255)
                    frame = bridge.cv2_to_imgmsg(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), 'bgr8')
                    frame.header.frame_id = f'{role}_optical_frame'
                    frame.header.stamp = fixture.get_clock().now().to_msg()
                    image_pub.publish(frame)
                    last_sent = time.monotonic()
                jpeg = link.jpeg(role, 'servo')
                if jpeg is None: continue
                bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if visible and colored(bgr) > 50:
                    raw = cv2.imdecode(np.frombuffer(link.jpeg(role, 'raw'), np.uint8), cv2.IMREAD_COLOR)
                    assert colored(raw) == 0
                    passed = True
                    break
                with link.lock:
                    status = link.values.get(f'{role}_detector', (0, ''))[1]
                if not visible and status.startswith('INVALID:') and colored(bgr) == 0 and bgr.mean() > 240:
                    passed = True
                    break
            assert passed, (role, visible, link.values.get(f'{role}_detector'))
    finally:
        executor.shutdown()
        link.close()
        for node in reversed(nodes): node.destroy_node()
        rclpy.shutdown()
