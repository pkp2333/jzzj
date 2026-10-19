#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec python3 "$JZZJ_ROOT/mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_grasp.py" "$@"
