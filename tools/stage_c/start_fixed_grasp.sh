#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$JZZJ_ROOT"
exec python3 "$JZZJ_ROOT/tools/stage_c/start_fixed_grasp.py" "$@"
