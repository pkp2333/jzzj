# jzzj_misson

This package is the task-level framework for the current JJZJ mission. The spelling follows the requested package name: jzzj_misson.

The design follows the existing mission_manage idea: one task state machine coordinates independent hardware and perception nodes. The package does not implement a second arm, obstacle avoidance, SLAM, or a full navigation stack.

## Task flow

WAIT_TAG -> LOCALIZE -> MOVE_TO_PICK -> PICK_ALIGN -> SEARCH_OBJECT -> VISUAL_APPROACH -> GRASP -> VERIFY_GRASP -> MOVE_TO_DROP -> DROP_ALIGN -> RELEASE -> FINISH

AprilTag is used for vehicle pose and point-to-point correction. Since the field is unobstructed, MOVE_TO_PICK and MOVE_TO_DROP should be implemented by the existing chassis driver plus a Tag-error controller, rather than a global path planner.

## Framework topics

- jzzj_misson/start: std_msgs/Empty
- jzzj_misson/stop: std_msgs/Empty
- jzzj_misson/advance: std_msgs/Empty
- jzzj_misson/tag_ready: std_msgs/Bool
- jzzj_misson/grasp_confirmed: std_msgs/Bool
- jzzj_misson/drop_confirmed: std_msgs/Bool
- jzzj_misson/state: std_msgs/String
- jzzj_misson/motion_command: std_msgs/String
- jzzj_misson/left_arm_command: std_msgs/String

The task node now publishes `/cmd_vel` and `/car_cmd` for point-to-point motion, and sends calibrated left-arm `JointState` poses for prepare, close, lift, and release. The external confirmation topics remain available for reliable grasp and drop verification.

The visual-servo package is a separate package named jzzj_visual_servo. The task package can launch it with enable_visual_servo:=true. It receives the visual package grasp_ready result through a launch remap and sends it an enable signal during PICK_ALIGN, SEARCH_OBJECT, and VISUAL_APPROACH.

## Start

    cd mission_ws
    catkin_make
    source devel/setup.bash
    roslaunch jzzj_misson jzzj_misson_test.launch

The normal launch does not start automatically:

    roslaunch jzzj_misson jzzj_misson.launch target_color:=red enable_visual_servo:=true
    rostopic pub -1 /jzzj_misson/start std_msgs/Empty '{}'

Use advance only while testing a stage manually. Real integration should let the AprilTag, chassis, vision and left-arm nodes report stage completion.

## Integration order

1. Connect front and rear AprilTag detection and publish tag_ready after pose validation.
2. Connect the chassis point-to-point controller to motion_command.
3. Connect the front camera detector and color selector to SEARCH_OBJECT and VISUAL_APPROACH.
4. Connect only the left-arm controller to left_arm_command.
5. Publish grasp_confirmed from the gripper or a verified vision result.
6. Connect the drop-box alignment and release confirmation.

Set `pick_pose` and `drop_pose` in `params/mission.yaml` before running. The task node stops and enters `FAULT` if a target pose or localization is unavailable. Test on the robot with an emergency stop available.
