#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Find catkin workspace
if [ -f "${SCRIPT_DIR}/devel/setup.bash" ]; then
  CATKIN_DIR="${SCRIPT_DIR}"
elif [ -f "${SCRIPT_DIR}/../devel/setup.bash" ]; then
  CATKIN_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
else
  CATKIN_DIR="$(find /mnt/d -name "devel/setup.bash" -path "*/catkin_ws/devel/setup.bash" 2>/dev/null | head -1 | xargs dirname 2>/dev/null | xargs dirname 2>/dev/null)"
  if [ -z "${CATKIN_DIR}" ]; then
    echo "ERROR: cannot find catkin workspace"
    exit 1
  fi
fi

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
source "${CATKIN_DIR}/devel/setup.bash"

ROOM="${1:-living_room}"
AUTOSTART_DELAY="${2:-9999}"

echo "==> Starting robot vacuum house demo..."
echo "==> Auto-starting room: ${ROOM} after ${AUTOSTART_DELAY}s"

roslaunch cleanbot_course_project robot_vacuum_house_demo.launch \
  autostart_room:="${ROOM}" \
  autostart_delay:="${AUTOSTART_DELAY}"
