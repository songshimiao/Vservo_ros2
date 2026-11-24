# 精度采集节点：从指定基座系目标话题手动保存样本，可选状态过滤和独立参考。
from pathlib import Path
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .geometry import pose_matrix, read_json, write_json
from .ros_utils import param, spin, fresh


# 只记录定位观测，不发送运动；重复性与绝对误差的计算在 accuracy.report。
class AccuracyRecorder(Node):
    # 读取输出/参考配置，缓存最新位姿及定位状态，开放 ~/capture 服务。
    def __init__(self):
        super().__init__('accuracy_recorder')
        self.output = Path(param(self, 'output', '/tmp/vservo_accuracy.json')).expanduser()
        reference = param(self, 'reference', '')
        self.reference = read_json(reference)['base_T_target'] if reference else None
        self.samples, self.stamps, self.sources = [], [], []
        self.frame = param(self, 'frame', 'base')
        self.topic = param(self, 'topic', '/localization/target_pose')
        self.state_filter = param(self, 'state_filter', '')
        self.state = 'UNKNOWN'
        self.states = []
        self.msg = None
        self.create_subscription(PoseStamped, self.topic, lambda m: setattr(self, 'msg', m), 5)
        self.create_subscription(String, '/localization/state', lambda m: setattr(self, 'state', m.data), 5)
        self.create_service(Trigger, '~/capture', self.capture)

    # 只接收指定坐标系的新鲜且未采过时间戳的位姿，满足状态过滤后原子落盘。
    def capture(self, request, response):
        try:
            m = self.msg
            if m is None or m.header.frame_id != self.frame or not fresh(self, m.header.stamp, 0.3):
                raise ValueError('Need fresh pose in expected frame')
            if self.state_filter and self.state != self.state_filter:
                raise ValueError(f'Expected state {self.state_filter}, got {self.state}')
            stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
            if stamp in self.stamps:
                raise ValueError('Duplicate observation')
            self.samples.append(pose_matrix(m.pose).tolist())
            self.stamps.append(stamp)
            self.states.append(self.state)
            data = dict(poses=self.samples, stamps=self.stamps, states=self.states, frame=self.frame, topic=self.topic)
            if self.reference is not None:
                data['references'] = [self.reference]*len(self.samples)
            write_json(self.output, data)
            response.success, response.message = True, f'Saved {len(self.samples)} samples: {self.output}'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response


# 运行手动精度记录节点。
def main(args=None):
    spin(AccuracyRecorder, args)
