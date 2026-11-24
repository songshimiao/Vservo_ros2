import json
import threading
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from .vendor.camera_stream import CameraSettings, create_camera
from .geometry import read_json
from .calibration import validate_intrinsic
from .ros_utils import param, spin, LATCHED


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        role = param(self, 'role', 'eih')
        if role not in ('eih', 'eth'):
            raise ValueError('role must be eih / eth')
        serial = param(self, 'serial', 'CV27864000EE' if role == 'eih' else 'CP08863000Z7')
        width, height, fps = [param(self, k, v) for k, v in [('width', 640), ('height', 480), ('fps', 30)]]
        self.frame = f'{role}_optical_frame'
        path = param(self, 'intrinsics', '')
        self.stream = create_camera(CameraSettings('orbbec', width, height, fps, serial,
                                                   'gemini305' if role == 'eih' else 'gemini335'))
        try:
            data = read_json(path) if path else self.stream.get_factory_intrinsics()
            K, D = validate_intrinsic(data, role, self.stream.metadata['serial'], (width, height))
            self.info = CameraInfo(width=width, height=height, distortion_model='rational_polynomial' if D.size == 8 else 'plumb_bob')
            self.info.k = K.reshape(-1).tolist()
            self.info.d = D.reshape(-1).tolist()
            self.info.r = np.eye(3).reshape(-1).tolist()
            self.info.p = np.column_stack((K, np.zeros(3))).reshape(-1).tolist()
            self.info_pub = self.create_publisher(CameraInfo, 'camera_info', LATCHED)
            self.pub = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
            self.meta_pub = self.create_publisher(String, 'metadata', LATCHED)
            data['camera'] = self.stream.metadata
            self.meta_pub.publish(String(data=json.dumps(data)))
            self.info.header.frame_id = self.frame
            self.info_pub.publish(self.info)
            self.bridge = CvBridge()
            self.stop_event = threading.Event()
            self.worker = threading.Thread(target=self.capture, daemon=True)
            self.worker.start()
            self.get_logger().info(f'{role} camera ready: {serial}; source={data["source"]}')
        except Exception:
            self.stream.stop()
            raise

    def capture(self):
        while not self.stop_event.is_set():
            try:
                image = self.stream.get_frame()
                if self.stop_event.is_set() or not self.context.ok():
                    break
                if image is None:
                    continue
                # Host receive time, not a hardware exposure timestamp. See docs.
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame
                self.pub.publish(msg)
            except Exception as exc:
                if self.context.ok():
                    self.get_logger().error(f'Camera acquisition stopped: {exc}')
                self.stop_event.set()

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=3)
        self.stream.stop()
        return super().destroy_node()


def main(args=None):
    spin(CameraNode, args)
