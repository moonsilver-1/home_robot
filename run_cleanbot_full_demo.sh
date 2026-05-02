#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

if [ "$#" -eq 0 ]; then
  set -- gui:=true
fi

roslaunch cleanbot_course_project static_room_cleaning_demo.launch "$@"
