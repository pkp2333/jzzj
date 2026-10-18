#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec rosrun jzzj_grasp stage_c_taught.py "$@"
