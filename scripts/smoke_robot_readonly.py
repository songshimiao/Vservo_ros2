#!/usr/bin/env python3
# 现场只读机械臂测试：启动 execute=false 的桥接并读取反馈，不调用 arm 或 ServoJ。
"""Live FR5 state smoke check. execute=false; no arm/ServoJ calls."""
import os
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# 启动自有 ROS 子进程并订阅反馈，达到条件后检查结果；finally 只停止本脚本创建的进程组。
def main():
    rclpy.init()
    node = Node('robot_readonly_smoke')
    samples, statuses = [], []
    node.create_subscription(JointState, '/robot/joint_states', samples.append, 5)
    node.create_subscription(String, '/robot/status', lambda m: statuses.append(m.data), 5)
    log = open('/tmp/vservo_robot_smoke.log', 'w')
    process = subprocess.Popen(['ros2', 'run', 'vservo_ros2', 'robot_bridge', '--ros-args', '-p', 'execute:=false'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(samples) >= 5 and 'READ_ONLY' in statuses:
                break
        assert len(samples) >= 5 and 'READ_ONLY' in statuses, 'No valid robot state; see /tmp/vservo_robot_smoke.log'
        assert all(np.isfinite(m.position).all() and len(m.position) == 6 for m in samples)
        print(f'PASS: {len(samples)} finite joint states, status READ_ONLY; zero motion requested')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
