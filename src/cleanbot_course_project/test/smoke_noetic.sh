#!/usr/bin/env bash

set -u

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FAILURES=0

pass() {
  echo "PASS: $1"
}

fail() {
  echo "FAIL: $1"
  FAILURES=$((FAILURES + 1))
}

check_cmd() {
  if "$@"; then
    pass "$*"
  else
    fail "$*"
  fi
}

echo "== Cleanbot Noetic Smoke Test =="

if command -v python3 >/dev/null 2>&1; then
  pass "python3 is available"
else
  fail "python3 is missing"
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  pass "sourced /opt/ros/noetic/setup.bash"
else
  fail "/opt/ros/noetic/setup.bash not found"
fi

if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "$WORKSPACE_DIR/devel/setup.bash"
  set -u
  pass "sourced workspace devel/setup.bash"
else
  echo "INFO: workspace devel/setup.bash not found yet; rospack checks may fail until catkin_make runs."
fi

if [ "${ROS_DISTRO:-}" != "noetic" ]; then
  fail "ROS_DISTRO must be noetic (current: ${ROS_DISTRO:-unset})"
else
  pass "ROS_DISTRO is noetic"
fi

if compgen -G "$PACKAGE_DIR/scripts/*.py" >/dev/null 2>&1; then
  if python3 -m py_compile "$PACKAGE_DIR"/scripts/*.py; then
    pass "python3 -m py_compile scripts/*.py"
  else
    fail "python syntax check failed"
  fi
else
  fail "No Python scripts found under scripts/"
fi

if PACKAGE_DIR="$PACKAGE_DIR" python3 - <<'PY'
import pathlib
import os
import sys
import yaml

root = pathlib.Path(os.environ["PACKAGE_DIR"])
files = [
    root / "config" / "scan_goals.yaml",
    root / "config" / "rooms.yaml",
    root / "config" / "gmapping.yaml",
    root / "config" / "move_base.yaml",
    root / "config" / "costmap_common.yaml",
    root / "config" / "global_costmap.yaml",
    root / "config" / "local_costmap.yaml",
    root / "config" / "follow_params.yaml",
    root / "config" / "arm_presets.yaml",
    root / "config" / "room_markers.yaml",
]

for path in files:
    with path.open("r", encoding="utf-8") as handle:
        yaml.safe_load(handle)
print("yaml ok")
PY
then
  pass "YAML configuration files parse"
else
  fail "YAML configuration check failed"
fi

if rospack profile >/dev/null 2>&1; then
  pass "rospack profile"
else
  fail "rospack profile"
fi

if rospack find cleanbot_course_project >/dev/null 2>&1; then
  pass "rospack find cleanbot_course_project"
else
  fail "rospack find cleanbot_course_project"
fi

if rostopic list >/dev/null 2>&1; then
  echo "INFO: ROS master detected, running 3-second startup probe."
  if timeout 3s python3 "$PACKAGE_DIR/scripts/image_preprocess_node.py" >/tmp/cleanbot_image_preprocess.log 2>&1; then
    pass "image_preprocess_node.py started and exited cleanly"
  else
    code=$?
    if [ "$code" -eq 124 ]; then
      pass "image_preprocess_node.py started successfully (timed out after 3s as expected)"
    else
      fail "image_preprocess_node.py startup probe failed"
      echo "----- probe log -----"
      cat /tmp/cleanbot_image_preprocess.log
    fi
  fi
else
  echo "INFO: ROS master not detected; skipping startup probe."
fi

if [ "$FAILURES" -eq 0 ]; then
  echo "== PASS =="
  exit 0
fi

echo "== FAIL: $FAILURES issue(s) found =="
exit 1
