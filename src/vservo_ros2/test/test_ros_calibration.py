import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from vservo_ros2.board import Board
from vservo_ros2.calibration_node import CalibrationNode
from vservo_ros2.geometry import fill_tf


def test_capture_preserves_raw_data_rejects_duplicates_and_small_solve(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'session_dir:={tmp_path}/session', '-p', f'output:={tmp_path}/result.json'])
    node = CalibrationNode()
    try:
        raw = Board().board.generateImage((700, 500))
        canvas = np.full((700, 900, 3), 255, np.uint8)
        canvas[100:600, 100:800] = np.repeat(raw[:, :, None], 3, axis=2)
        node.image = CvBridge().cv2_to_imgmsg(canvas, 'bgr8')
        node.image.header.frame_id = 'eih_optical_frame'
        node.image.header.stamp = node.get_clock().now().to_msg()
        node.info = CameraInfo(width=900, height=700, k=[1000.,0.,450.,0.,1000.,350.,0.,0.,1.], d=[0.]*5)
        node.meta = {'camera': {'serial': 'TEST', 'role': 'eye_in_hand'}, 'source': 'factory_sdk'}
        out = node.capture(Trigger.Request(), Trigger.Response())
        assert out.success, out.message
        assert (tmp_path/'session/0000.png').exists()
        assert (tmp_path/'session/samples.json').exists()
        duplicate = node.capture(Trigger.Request(), Trigger.Response())
        assert not duplicate.success and len(node.samples) == 1
        solve = node.solve(Trigger.Request(), Trigger.Response())
        assert not solve.success and not (tmp_path/'result.json').exists()
        node.image.header.stamp = node.get_clock().now().to_msg()
        node.meta['camera']['serial'] = 'OTHER'
        mismatch = node.capture(Trigger.Request(), Trigger.Response())
        assert not mismatch.success and 'identity' in mismatch.message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_handeye_capture_uses_latest_frame_with_bracketing_tf(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'mode:=handeye', '-p', f'session_dir:={tmp_path}'])
    node = CalibrationNode()
    try:
        canvas = np.full((700, 900, 3), 255, np.uint8)
        canvas[100:600, 100:800] = Board().board.generateImage((700, 500))[:, :, None]
        node.info = CameraInfo(width=900, height=700, k=[1000., 0., 450., 0., 1000., 350., 0., 0., 1.], d=[0.]*5)
        node.meta = dict(camera=dict(serial='TEST', role='eye_in_hand'), source='charuco_calibrated', validated=True)
        now = node.get_clock().now()
        for ms in (600, 30):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base', 'flange'
            tf.header.stamp = (now-Duration(nanoseconds=ms*1_000_000)).to_msg()
            node.tf.set_transform(fill_tf(tf, np.eye(4)), 'test')
        for ms in (70, 5):
            image = CvBridge().cv2_to_imgmsg(canvas, 'bgr8')
            image.header.frame_id = 'eih_optical_frame'
            image.header.stamp = (now-Duration(nanoseconds=ms*1_000_000)).to_msg()
            node.on_image(image)
        response = node.capture(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert node.last_stamp == now.nanoseconds - 70_000_000
    finally:
        node.destroy_node()
        rclpy.shutdown()
