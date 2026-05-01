#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
if [ -f "${SCRIPT_DIR}/devel/setup.bash" ]; then
  source "${SCRIPT_DIR}/devel/setup.bash"
else
  echo "Missing ${SCRIPT_DIR}/devel/setup.bash. Build the workspace first with catkin_make."
  exit 1
fi

roslaunch cleanbot_course_project robot_vacuum_house_demo.launch "$@"
