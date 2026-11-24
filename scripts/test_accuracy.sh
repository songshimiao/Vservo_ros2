#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 observations.json report.json [max_mm=5] [max_deg=2]" >&2
  exit 1
fi
ros2 run vservo_ros2 tools accuracy --input "$1" --out "$2" --max-mm "${3:-5}" --max-deg "${4:-2}"
