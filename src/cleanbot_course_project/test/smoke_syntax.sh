#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON_BIN=${PYTHON_BIN:-/usr/bin/python3}
$PYTHON_BIN -m py_compile scripts/*.py
echo "[PASS] Python syntax check passed."
$PYTHON_BIN - <<'PY' || true
# 可选检查：如果本机 python3 可用，验证配置文件存在。
from pathlib import Path
required = [
    'config/scan_goals.yaml',
    'config/rooms.yaml',
    'config/gmapping.yaml',
    'config/move_base.yaml',
    'config/costmap_common.yaml',
    'config/global_costmap.yaml',
    'config/local_costmap.yaml',
    'config/follow_params.yaml',
    'launch/coverage_demo.launch',
    'launch/room_cleaning_demo.launch',
    'launch/slam_room_cleaning_demo.launch',
]
for f in required:
    assert Path(f).exists(), f
print('[PASS] Required config/launch files exist.')
PY
