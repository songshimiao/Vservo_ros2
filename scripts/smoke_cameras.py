#!/usr/bin/env python3
# 现场双相机冒烟测试：会打开两台实体相机，验证帧数、序列号和尺寸；不连接机械臂。
"""Open both cameras through ROS launch, decode frames, then stop OUR launch.

Run after sourcing install/setup.bash. Does not connect to or move the robot.
No other process may own either camera while this check runs.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from vservo_ros2.ros_utils import LATCHED


# 启动自有 ROS 子进程并订阅反馈，达到条件后检查结果；finally 只停止本脚本创建的进程组。
def main():
    rclpy.init()
    node = Node('dual_camera_smoke')
    counts, metadata, infos, shapes = {}, {}, {}, {}
    bridge = CvBridge()
    # 将 ROS 图像解码为 BGR，按角色统计收到的有效帧并检查 optical_frame。
    def on_image(role, msg):
        image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        assert msg.header.frame_id == f'{role}_optical_frame'
        shapes[role] = list(image.shape)
        counts[role] = counts.get(role, 0)+1
    for role in ('eih', 'eth'):
        node.create_subscription(Image, f'/{role}/image_raw', lambda m, r=role: on_image(r, m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{role}/camera_info', lambda m, r=role: infos.update({r: m}), LATCHED)
        node.create_subscription(String, f'/{role}/metadata', lambda m, r=role: metadata.update({r: json.loads(m.data)}), LATCHED)
    log = Path('/tmp/vservo_camera_smoke.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'vservo_ros2', 'dual_camera.launch.py'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(counts.get(r, 0) >= 3 and r in infos and r in metadata for r in ('eih', 'eth')):
                break
        assert all(counts.get(r, 0) >= 3 for r in ('eih', 'eth')), f'Frames missing: {counts}; see {log.name}'
        for role, serial in [('eih', 'CV27864000EE'), ('eth', 'CP08863000Z7')]:
            assert metadata[role]['camera']['serial'] == serial
            assert shapes[role] == [480, 640, 3]
            assert (infos[role].width, infos[role].height) == (640, 480)
        print(json.dumps(dict(result='PASS', frames=counts, image_shapes=shapes,
                              serials={r: m['camera']['serial'] for r, m in metadata.items()}), indent=2))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
