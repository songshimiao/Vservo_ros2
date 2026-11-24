#!/usr/bin/env bash
# 离线回归入口：只给本次 pytest 设置源码 PYTHONPATH；测试内部使用合成图像/假 SDK。
# 需要可用的 ROS 及算法依赖，不能把此脚本与打开实体设备的 smoke 脚本混淆。
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH="$PWD/src/vservo_ros2${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q src/vservo_ros2/test
