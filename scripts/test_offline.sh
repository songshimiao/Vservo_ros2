#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH="$PWD/src/vservo_ros2${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q src/vservo_ros2/test
