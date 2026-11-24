#!/usr/bin/env bash
# 独立内参验收：参数依次为新会话 samples.json、已求解内参、输出报告；返回 CLI 的真实退出码。
set -euo pipefail
if [ "$#" -ne 3 ]; then
  echo "Usage: $0 independent_samples.json intrinsics.json report.json" >&2
  exit 1
fi
ros2 run vservo_ros2 tools check-intrinsics --session "$1" --calibration "$2" --out "$3"
