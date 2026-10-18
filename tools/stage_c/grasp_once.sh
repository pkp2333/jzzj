#!/usr/bin/env bash
# Explicit, operator-controlled field test. The heartbeat is a manual handoff assertion.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
session="${1:-stage-c-$(date +%s)-$$}"
color="${2:-yellow}"
/usr/bin/python3 "$JZZJ_ROOT/tools/stage_c/check_control.py" --ready
echo "单次现场测试 session=$session color=$color；调试心跳代表操作者已确认停车交权。"
/usr/bin/python3 "$JZZJ_ROOT/tools/stage_c/heartbeat.py" "$session" &
heartbeat_pid=$!
trap 'kill -TERM "$heartbeat_pid" 2>/dev/null || true; wait "$heartbeat_pid" 2>/dev/null || true' EXIT
/usr/bin/python3 "$JZZJ_ROOT/tools/stage_c/check_control.py" --ready --session "$session"
rosrun jzzj_grasp stage_c_client.py _session_id:="$session" _target_color:="$color"
