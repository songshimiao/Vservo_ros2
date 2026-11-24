"""Render -> actual detector node -> timestamp TF -> selector -> PBVS topic.

Synthetic images only; no cameras, robot SDK or movement.
"""
import time
import json
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
from vservo_ros2.board import Board
from vservo_ros2.geometry import write_json, fill_tf
from vservo_ros2.geometry import fill_pose, transform, inverse, pose_matrix
from vservo_ros2.detector_node import DetectorNode
from vservo_ros2.localizer_node import LocalizerNode
from vservo_ros2.servo_node import ServoNode


@pytest.mark.parametrize('overlap', [True, False])
def test_ros_detection_to_servo_with_timestamped_tf_and_loss(tmp_path, overlap):
    calibration = tmp_path / 'handeye.json'
    write_json(calibration, dict(mode='eih', validated=True, matrix=np.eye(4).tolist()))
    rclpy.init(args=['--ros-args', '-p', f'eih_handeye:={calibration}',
                    '-p', 'selection_policy:=handoff',
                    '-p', 'workspace_min:=[-2.0,-2.0,-2.0]', '-p', 'workspace_max:=[2.0,2.0,2.0]'])
    detector, localizer, servo = DetectorNode(), LocalizerNode(), ServoNode()
    fixture = Node('synthetic_fixture')
    executor = SingleThreadedExecutor()
    nodes = [detector, localizer, servo, fixture]
    for node in nodes:
        executor.add_node(node)
    received, states = [], []
    fixture.create_subscription(PoseStamped, '/servo/next_flange_pose', received.append, 5)
    fixture.create_subscription(String, '/localization/state', lambda m: states.append(m.data), 5)
    secondary = fixture.create_publisher(PoseWithCovarianceStamped, '/eth/target_pose', 5)
    def mirror(msg):
        msg.header.frame_id = 'eth_optical_frame'
        secondary.publish(msg)
    if overlap:
        fixture.create_subscription(PoseWithCovarianceStamped, '/eih/target_pose', mirror, 5)
    static = StaticTransformBroadcaster(fixture)
    transforms = []
    for child in ('flange', 'eih_optical_frame', 'eth_optical_frame'):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = 'base', child
        msg.header.stamp = fixture.get_clock().now().to_msg()
        transforms.append(fill_tf(msg, np.eye(4)))
    static.sendTransform(transforms)
    raw = Board().board.generateImage((700, 500))
    canvas = np.full((700, 900), 255, np.uint8)
    canvas[100:600, 100:800] = raw
    K = [1000., 0., 450., 0., 1000., 350., 0., 0., 1.]
    detector.info = CameraInfo(width=900, height=700, k=K, d=[0.]*5)
    detector.meta = {'validated': True}
    bridge = CvBridge()
    # Static flange is valid at all times; production servo requires fresh dynamic
    # robot timestamp, so refresh header of the returned flange transform in test.
    real_lookup = servo.tf.lookup_transform
    def lookup(*args, **kwargs):
        result = real_lookup(*args, **kwargs)
        result.header.stamp = fixture.get_clock().now().to_msg()
        return result
    servo.tf.lookup_transform = lookup
    try:
        if not overlap:
            coarse = PoseWithCovarianceStamped()
            coarse.header.frame_id = 'eth_optical_frame'
            coarse.header.stamp = fixture.get_clock().now().to_msg()
            fill_pose(coarse.pose.pose, transform(t=[0.0, 0.0, 0.4]))
            start = time.monotonic()
            localizer.observe('eth', coarse)
            while time.monotonic()-start < 0.38:
                executor.spin_once(timeout_sec=0.01)
            assert 'COARSE' in states and states[-1] == 'HOLD'
        deadline, last_image = time.monotonic()+5, 0
        while time.monotonic() < deadline and 'FINE' not in states:
            executor.spin_once(timeout_sec=0.01)
            if time.monotonic()-last_image > 0.08:
                image = bridge.cv2_to_imgmsg(np.repeat(canvas[:, :, None], 3, axis=2), 'bgr8')
                image.header.frame_id = 'eih_optical_frame'
                image.header.stamp = fixture.get_clock().now().to_msg()
                detector.detect(image)
                last_image = time.monotonic()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
        assert 'FINE' in states
        assert received and received[-1].header.frame_id == 'base'
        if not overlap:
            assert localizer.selector.diagnostics['eth_age_s'] is None
        # Stop feeding images: neither localizer nor servo can keep old vision alive.
        deadline = time.monotonic()+0.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert states[-1] == 'HOLD'
        count = len(received)
        deadline = time.monotonic()+0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(received) == count
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


def test_localizer_retries_original_image_time_when_tf_arrives_later():
    rclpy.init()
    node = LocalizerNode()
    try:
        observation = PoseWithCovarianceStamped()
        observation.header.frame_id = 'eih_optical_frame'
        observation.header.stamp = node.get_clock().now().to_msg()
        fill_pose(observation.pose.pose, transform(t=[0, 0, 0.3]))
        observation.pose.covariance = np.diag([0.003**2]*3+[0.01**2]*3).reshape(-1).tolist()
        node.observe('eih', observation)
        node.tick()
        assert len(node.pending['eih']) == 1 and 'eih' not in node.selector.samples
        assert node.selector.diagnostics['reason'] == 'waiting_tf'
        assert node.selector.diagnostics['cameras']['eih']['visible']
        assert not node.selector.diagnostics['cameras']['eih']['localized']
        tf = TransformStamped()
        tf.header.frame_id = 'base'
        tf.child_frame_id = 'eih_optical_frame'
        tf.header.stamp = observation.header.stamp
        fill_tf(tf, transform(t=[0.2, 0, 0]))
        node.tf.set_transform(tf, 'synthetic_test')
        node.tick()
        assert not node.pending['eih']
        assert np.allclose(node.selector.samples['eih'][1][:3, 3], [0.2, 0, 0.3])
        assert node.selector.diagnostics['source'] == 'eih'
        assert node.selector.diagnostics['cameras']['eih']['localized']
        time.sleep(0.32)
        node.tick()
        assert node.selector.diagnostics['reason'] == 'both_lost'
        assert not node.selector.diagnostics['cameras']['eih']['visible']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stamped_detection_loss_switches_camera_before_pose_ttl_and_ignores_old_packets():
    rclpy.init()
    node = LocalizerNode()
    try:
        def observation(role):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = f'{role}_optical_frame'
            msg.header.stamp = node.get_clock().now().to_msg()
            fill_pose(msg.pose.pose, transform(t=[0, 0, 0.3]))
            return msg
        for role in ('eth', 'eih'):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base', f'{role}_optical_frame'
            node.tf.set_transform_static(fill_tf(tf, np.eye(4)), 'test')
        eth, eih = observation('eth'), observation('eih')
        node.observe('eth', eth)
        node.tick()
        assert node.selector.diagnostics['source'] == 'eth'
        node.observe('eih', eih)
        node.tick()
        assert node.selector.diagnostics['source'] == 'eth'
        lost_stamp = node.get_clock().now().nanoseconds
        node.on_visibility('eth', String(data=json.dumps(dict(
            frame_id='eth_optical_frame', stamp_ns=lost_stamp, visible=False))))
        node.tick()
        assert node.selector.diagnostics['source'] == 'eih'
        assert not node.selector.diagnostics['cameras']['eth']['visible']
        # A delayed pose for an image already marked lost cannot revive it.
        node.observe('eth', eth)
        node.tick()
        assert node.selector.diagnostics['source'] == 'eih'
        assert 'eth' not in node.selector.samples and not node.pending['eth']
        fresh = observation('eth')
        node.observe('eth', fresh)
        node.tick()
        assert node.selector.diagnostics['cameras']['eth']['visible']
        node.on_visibility('eth', String(data=json.dumps(dict(
            frame_id='eth_optical_frame', stamp_ns=lost_stamp-1, visible=False))))
        node.tick()
        assert node.selector.diagnostics['cameras']['eth']['visible']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rendered_id42_keeps_following_after_arrival_with_eih_fallback(tmp_path):
    path = tmp_path/'handeye.json'
    # A camera mounted beside the flange keeps an oblique view at arrival.
    # A coincident optical/flange frame becomes exactly frontal and the raster
    # corners can legitimately trigger the monocular ambiguity rejection.
    flange_camera = transform(t=[0.08, 0, 0])
    write_json(path, dict(mode='eih', validated=True, matrix=flange_camera.tolist()))
    rclpy.init(args=['--ros-args', '-p', f'eih_handeye:={path}', '-p', 'target_id:=aruco_42',
                    '-p', 'tracking_frame:=flange', '-p', 'fine_distance_m:=0.30',
                    '-p', 'workspace_min:=[-2.0,-2.0,-2.0]', '-p', 'workspace_max:=[2.0,2.0,2.0]'])
    detector, localizer, servo = DetectorNode(), LocalizerNode(), ServoNode()
    fixture = Node('moving_marker_fixture')
    executor = SingleThreadedExecutor()
    nodes = [detector, localizer, servo, fixture]
    for node in nodes:
        executor.add_node(node)
    current = transform(t=[0.0, 0.0, 0.15])
    world_target = transform(cv2.Rodrigues(np.array([0.16, -0.20, 0.0]))[0], [0.0, 0.0, 0.45])
    initial = current.copy()
    poses = []
    detector_states = []
    fixture.create_subscription(String, '/eih/detector_status', lambda m: detector_states.append(m.data), 5)

    def follow(msg):
        nonlocal current
        current = pose_matrix(msg.pose)
        poses.append(current)

    fixture.create_subscription(PoseStamped, '/servo/next_flange_pose', follow, 1)

    def flange_tf(*args):
        msg = TransformStamped()
        msg.header.stamp = fixture.get_clock().now().to_msg()
        return fill_tf(msg, current)

    servo.tf.lookup_transform = flange_tf
    eth_tf = TransformStamped()
    eth_tf.header.frame_id, eth_tf.child_frame_id = 'base', 'eth_optical_frame'
    localizer.tf.set_transform_static(fill_tf(eth_tf, np.eye(4)), 'test')
    K = np.array([[400., 0, 320], [0, 400., 240], [0, 0, 1]])
    detector.info = CameraInfo(width=640, height=480, k=K.ravel().tolist(), d=[0.]*5)
    detector.meta = {'validated': True}
    marker = detector.detector
    bitmap = cv2.aruco.generateImageMarker(marker.dictionary, 42, 240)
    corners = np.array([[0, 0], [239, 0], [239, 239], [0, 239]], np.float32)
    bridge = CvBridge()
    try:
        start = time.monotonic()
        last_image = 0
        while time.monotonic()-start < 3.0:
            executor.spin_once(timeout_sec=0.005)
            elapsed = time.monotonic()-start
            if time.monotonic()-last_image < 0.065:
                continue
            last_image = time.monotonic()
            # First arrive at a stationary target, then move it by 10 mm.
            world_target[0, 3] = initial[0, 3]+min(0.01, max(0, elapsed-1)*0.01)
            camera_target = inverse(current @ flange_camera) @ world_target
            raw_rotation = camera_target[:3, :3] @ np.diag([1, -1, -1])
            pixels = cv2.projectPoints(marker.points, cv2.Rodrigues(raw_rotation)[0], camera_target[:3, 3], K, np.zeros(5))[0]
            warp = cv2.getPerspectiveTransform(corners, pixels.reshape(4, 2))
            image = cv2.warpPerspective(bitmap, warp, (640, 480), borderValue=255)
            msg = bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), 'bgr8')
            msg.header.frame_id = 'eih_optical_frame'
            msg.header.stamp = fixture.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base', 'eih_optical_frame'
            tf.header.stamp = msg.header.stamp
            localizer.tf.set_transform(fill_tf(tf, current @ flange_camera), 'test')
            detector.detect(msg)
            if elapsed < 0.2:
                coarse = PoseWithCovarianceStamped()
                coarse.header.frame_id, coarse.header.stamp = 'eth_optical_frame', msg.header.stamp
                other = world_target.copy()
                fill_pose(coarse.pose.pose, other)
                localizer.observe('eth', coarse)
        if not localizer.selector.fine:
            pytest.fail(f'{localizer.selector.diagnostics}\n{detector_states[-8:]}\nposes={len(poses)}\n{current}')
        assert localizer.selector.diagnostics['pair_status'] == 'missing'
        assert len(poses) > 20
        assert current[0, 3]-initial[0, 3] > 0.005
        assert np.linalg.norm(current[:3, 3]-world_target[:3, 3]) == pytest.approx(0.30, abs=0.006)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
