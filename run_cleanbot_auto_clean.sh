#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOM="${1:-living_room}"
if [ "$#" -gt 0 ]; then
  shift
fi

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

roslaunch cleanbot_course_project static_room_cleaning_demo.launch \
  gui:=true \
  autostart_room:="${ROOM}" \
  autostart_delay:=15.0 \
  "$@"
