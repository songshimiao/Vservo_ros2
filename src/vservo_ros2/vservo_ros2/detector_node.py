# 检测节点：image_raw + CameraInfo → 本相机坐标系的 target_pose。
# 每帧另发带原始时间戳的可见性消息，供定位器快速撤销丢失目标。
import importlib
import json
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from .board import ArucoMarker
from .geometry import fill_pose, validate
from .ros_utils import param, spin, LATCHED, board_from_params, fresh


# 统一接入 ChArUco、单 ArUco 或外部米制 6D 插件；OpenCV 限制为单线程减少争用。
class DetectorNode(Node):
    # 选择检测实现并订阅图像、内参、元数据；target_id 是两路必须一致的物理目标标识。
    def __init__(self):
        super().__init__('detector')
        cv2.setNumThreads(1)
        self.role = param(self, 'role', 'eih')
        self.target_id = param(self, 'target_id', 'charuco_board')
        plugin = param(self, 'plugin', '')
        if plugin:
            module, cls = plugin.split(':')
            self.detector = getattr(importlib.import_module(module), cls)()
        elif self.target_id == 'charuco_board':
            self.detector = board_from_params(self)
        elif self.target_id.startswith('aruco_'):
            self.detector = ArucoMarker(int(self.target_id.removeprefix('aruco_')),
                                        param(self, 'aruco_marker_m', 0.080))
        else:
            raise ValueError(f'Unknown built-in target: {self.target_id}')
        self.info = self.meta = None
        self.bridge = CvBridge()
        self.pub = self.create_publisher(PoseWithCovarianceStamped, f'/{self.role}/target_pose', 5)
        self.status = self.create_publisher(String, f'/{self.role}/detector_status', 5)
        self.visibility = self.create_publisher(String, f'/{self.role}/target_visibility', 1)
        self.create_subscription(CameraInfo, f'/{self.role}/camera_info', lambda m: setattr(self, 'info', m), LATCHED)
        self.create_subscription(String, f'/{self.role}/metadata', lambda m: setattr(self, 'meta', json.loads(m.data)), LATCHED)
        self.create_subscription(Image, f'/{self.role}/image_raw', self.detect, qos_profile_sensor_data)

    # 检查图像时效、内参验证状态和尺寸，再检查位姿、协方差、ID 与 2 px 重投影门槛。
    # 有效位姿保留图像 header；失败只发布 INVALID/visible=False，不重发上一次位姿。
    def detect(self, msg):
        visible, reason = False, ''
        try:
            if self.info is None or self.meta is None or not self.meta.get('validated'):
                raise ValueError('Validated ChArUco intrinsics required for localization')
            if not fresh(self, msg.header.stamp, 0.3) or msg.header.frame_id != f'{self.role}_optical_frame':
                raise ValueError('Image stale or frame mismatch')
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if (image.shape[1], image.shape[0]) != (self.info.width, self.info.height):
                raise ValueError('Image dimensions differ from calibrated CameraInfo')
            obs = self.detector.detect(image,
                                       np.array(self.info.k).reshape(3, 3), np.array(self.info.d))
            validate(obs.camera_T_target)
            cov = np.asarray(obs.covariance, float)
            if (obs.target_id != self.target_id or cov.shape != (6, 6) or not np.isfinite(cov).all()
                    or not np.allclose(cov, cov.T) or np.min(np.linalg.eigvalsh(cov)) < 0
                    or not np.isfinite(obs.reprojection_px) or obs.reprojection_px > 2.0):
                raise ValueError('Target identity / metric pose / uncertainty / reprojection gate failed')
            out = PoseWithCovarianceStamped()
            out.header = msg.header
            fill_pose(out.pose.pose, obs.camera_T_target)
            out.pose.covariance = cov.reshape(-1).tolist()
            self.pub.publish(out)
            visible = True
            self.status.publish(String(data=f'TRACKING {obs.target_id} ({obs.reprojection_px:.2f} px)'))
        except Exception as exc:
            # No stale pose republished on detection loss.
            self.status.publish(String(data=f'INVALID: {exc}'))
            reason = str(exc)
        finally:
            self.visibility.publish(String(data=json.dumps(dict(
                frame_id=msg.header.frame_id,
                stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                visible=visible, target_id=self.target_id, reason=reason))))


# 启动单路检测节点；两路节点由 launch 分别创建。
def main(args=None):
    spin(DetectorNode, args)
