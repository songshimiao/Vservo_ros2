#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 3 ]; then
  echo "Usage: $0 independent_samples.json handeye.json report.json" >&2
  exit 1
fi
ros2 run vservo_ros2 tools check-handeye --session "$1" --calibration "$2" --out "$3"
