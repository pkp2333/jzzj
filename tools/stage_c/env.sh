#!/usr/bin/env bash
# Source this file before manual ROS commands for the canonical C-stage project.
JZZJ_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
export JZZJ_ROOT
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
source /opt/ros/noetic/setup.bash
for workspace in catkin_ws mission_ws; do
  if [[ ! -f "$JZZJ_ROOT/$workspace/devel/setup.bash" ]]; then
    echo "先运行 $JZZJ_ROOT/tools/stage_c/build.sh" >&2
    return 1 2>/dev/null || exit 1
  fi
  source "$JZZJ_ROOT/$workspace/devel/setup.bash"
done
for package in arm chassis user_msgs user_params jzzj_grasp jzzj_grasp_interfaces jzzj_visual_servo jzzj_misson; do
  package_path="$(rospack find "$package")" || return 1
  case "$package_path" in "$JZZJ_ROOT"/*) ;; *) echo "包路径冲突: $package -> $package_path" >&2; return 1;; esac
done
