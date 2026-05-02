#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOM="${1:-living_room}"

source_ros_env() {
  set +u
  source "$1"
  set -u
}

source_ros_env /opt/ros/noetic/setup.bash
if [ -f "${SCRIPT_DIR}/devel/setup.bash" ]; then
  source_ros_env "${SCRIPT_DIR}/devel/setup.bash"
else
  echo "Missing ${SCRIPT_DIR}/devel/setup.bash. Build the workspace first with catkin_make."
  exit 1
fi

if ! rostopic list >/tmp/cleanbot_rostopic_check.log 2>&1; then
  echo "ROS master is not running."
  echo "Start the full simulator first:"
  echo "  bash ./run_cleanbot_full_demo.sh"
  exit 1
fi

if ! rostopic list | grep -q '^/move_base/goal$'; then
  echo "/move_base is not running, so the robot cannot move to cleaning goals yet."
  echo "Start the full SLAM/navigation stack first:"
  echo "  bash ./run_cleanbot_full_demo.sh"
  exit 1
fi

if ! rostopic list | grep -q '^/room_coverage_cleaner/status$'; then
  echo "room_coverage_cleaner is not running, so /cleanbot/clean_room has no active cleaner to handle it."
  echo "Start the full demo first:"
  echo "  bash ./run_cleanbot_full_demo.sh"
  exit 1
fi

echo "Requesting room cleaning: ${ROOM}"
rostopic pub /cleanbot/clean_room std_msgs/String "data: ${ROOM}" -1
