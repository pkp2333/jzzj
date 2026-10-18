#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec roslaunch jzzj_grasp stage_c_observe_full.launch   config:="$JZZJ_ROOT/deploy/stage_c.yaml"   detector_config:="$JZZJ_ROOT/deploy/detector.yaml" target_color:="${1:-yellow}"
