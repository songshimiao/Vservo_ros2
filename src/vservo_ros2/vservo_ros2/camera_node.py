# 相机节点：按 EIH/ETH 角色绑定型号和序列号，发布 BGR 图像、CameraInfo 和元数据。
# 采集在独立线程中运行；工厂内参可预览，但检测定位还要求通过 ChArUco 验证的内参。
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


# 一个节点管理一台相机；由 launch 放入 /eih 或 /eth 命名空间。
class CameraNode(Node):
    # 连接相机并核对内参与设备身份/分辨率，成功后发布锁存信息并启动采集线程。
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

    # 循环读帧并发布 Image；使用主机接收时间，不代表硬件曝光同步时间。
    # 读帧或发布发生异常时停止本采集线程并记录错误，不用旧图像冒充新帧。
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

    # 先通知采集退出并等待，再关闭相机，最后销毁 ROS 资源。
    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=3)
        self.stream.stop()
        return super().destroy_node()


# 通过公共 spin 包装器运行节点，保证退出时调用上述清理。
def main(args=None):
    spin(CameraNode, args)
