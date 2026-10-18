#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
case "${1:-sim}" in
  sim) exec roslaunch jzzj_grasp stage_c_sim.launch ;;
  real)
    nodes="$(timeout 5 rosnode list 2>/dev/null)" || { echo '先在单独终端运行 roscore'; exit 1; }
    if echo "$nodes" | grep -Eq '/arm_left/(arm_control_node|arm_control|arm_trajectory|arm_trajectory_node|arm_test_node|arm_test)$|/stage_c_(server|observer|kinematics)$|/detections_merger$|(^|/)yolov5'; then
      echo '先退出已有左臂测试/抓取/观察入口，避免重复控制；本脚本不会强杀其他任务。' >&2
      exit 1
    fi
    camera=true
    if echo "$nodes" | grep -qx '/ascamera_hp60c'; then camera=false; fi
    /usr/bin/python3 "$JZZJ_ROOT/tools/stage_c/check_control.py"
    exec roslaunch jzzj_grasp stage_c.launch backend:=real       config:="$JZZJ_ROOT/deploy/stage_c.yaml" start_camera:="$camera"       start_detector:=true detector_config:="$JZZJ_ROOT/deploy/detector.yaml"       start_arm:=true arm_config:="$JZZJ_ROOT/deploy/arm.yaml"       start_kinematics:=true kinematics_config:="$JZZJ_ROOT/deploy/kinematics.yaml"
    ;;
  *) echo '用法: start_control.sh sim|real' >&2; exit 2 ;;
esac
