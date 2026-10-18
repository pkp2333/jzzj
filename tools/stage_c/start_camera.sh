#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec roslaunch ascamera hp60c.launch flip_vertical:=false
