#!/usr/bin/env bash
set -euo pipefail

ROOM="${1:-living_room}"
RUN_SECONDS="${2:-45}"
GUI="${3:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="/tmp/cleanbot_room_cleaning_${ROOM}.log"
PUB_LOG="/tmp/cleanbot_room_cleaning_${ROOM}_pub.log"
ODOM_START="/tmp/cleanbot_room_cleaning_${ROOM}_odom_start.txt"
ODOM_END="/tmp/cleanbot_room_cleaning_${ROOM}_odom_end.txt"

source_ros_env() {
  set +u
  source "$1"
  set -u
}

cleanup() {
  if [ -n "${LAUNCH_PID:-}" ]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
  fi
  pkill -x roslaunch 2>/dev/null || true
  pkill -x gzserver 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -x rosmaster 2>/dev/null || true
  pkill -x rostopic 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 2

source_ros_env /opt/ros/noetic/setup.bash
source_ros_env "${SCRIPT_DIR}/devel/setup.bash"

rm -f "${LOG_FILE}" "${PUB_LOG}" "${ODOM_START}" "${ODOM_END}"

roslaunch cleanbot_course_project static_room_cleaning_demo.launch "gui:=${GUI}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

sleep 18

timeout 5 rostopic echo -n 1 /odom/pose/pose/position >"${ODOM_START}" 2>&1 || true
rostopic pub /cleanbot/clean_room std_msgs/String "data: ${ROOM}" -1 >"${PUB_LOG}" 2>&1 || true

sleep "${RUN_SECONDS}"

timeout 5 rostopic echo -n 1 /odom/pose/pose/position >"${ODOM_END}" 2>&1 || true
cleanup
sleep 2

echo "__ROOM__ ${ROOM}"
echo "__ODOM_START__"
cat "${ODOM_START}" || true
echo "__ODOM_END__"
cat "${ODOM_END}" || true
echo "__PUBLISH__"
cat "${PUB_LOG}" || true
echo "__CLEANER_EVENTS__"
grep -a -E "planned|fallback_cmd_vel|goal_sent|goal_reached|goal_failed|finished|move_base_unavailable|invalid_room" "${LOG_FILE}" | tail -n 80 || true
echo "__MOVE_BASE_WARNINGS__"
grep -a -E "Aborting|Failed|Rotate recovery|Clearing both costmaps|Got new plan" "${LOG_FILE}" | tail -n 40 || true
