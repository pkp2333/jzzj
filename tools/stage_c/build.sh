#!/usr/bin/env bash
set -eo pipefail
JZZJ_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
stage_c_base_packages='user_msgs;user_params;chassis;arm;ascamera;usb_cam;yolov5_rknn'
stage_c_mission_packages='jzzj_grasp_interfaces;jzzj_visual_servo;jzzj_grasp;jzzj_misson'
if [[ "${1:-}" == '--with-base-navigation' ]]; then
  echo 'Building stage-C fruit-relative base adapter; no Tag localization dependency'
elif [[ $# -gt 0 ]]; then
  echo 'Usage: build.sh [--with-base-navigation]' >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
cd "$JZZJ_ROOT/catkin_ws"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="$stage_c_base_packages"
source devel/setup.bash
cd "$JZZJ_ROOT/mission_ws"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSTAGE_C_BUILD_KINEMATICS=ON -DCATKIN_WHITELIST_PACKAGES="$stage_c_mission_packages"
echo "STAGE_C_BUILD_COMPLETE"
