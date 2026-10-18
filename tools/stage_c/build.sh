#!/usr/bin/env bash
set -eo pipefail
JZZJ_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
source /opt/ros/noetic/setup.bash
cd "$JZZJ_ROOT/catkin_ws"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3   -DCATKIN_WHITELIST_PACKAGES='user_msgs;user_params;chassis;arm;ascamera;usb_cam;yolov5_rknn'
source devel/setup.bash
cd "$JZZJ_ROOT/mission_ws"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSTAGE_C_BUILD_KINEMATICS=ON   -DCATKIN_WHITELIST_PACKAGES='jzzj_grasp_interfaces;jzzj_visual_servo;jzzj_grasp;jzzj_misson'
echo "STAGE_C_BUILD_COMPLETE"
