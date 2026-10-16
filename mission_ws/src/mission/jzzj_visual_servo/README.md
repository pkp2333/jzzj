# jzzj_visual_servo

This package isolates the front-camera visual-servoing interface for the JZZJ mission. It is independent from the task state machine and from the left-arm implementation.

## Input contract

The package uses standard ROS messages so it can be connected before the final detector message is known.

- jzzj_visual_servo/target_visible: std_msgs/Bool
- jzzj_visual_servo/target_error: geometry_msgs/Vector3
- target_error.x: normalized horizontal image error; right is positive
- target_error.y: normalized vertical image error; down is positive
- target_error.z: distance error; positive means too far from the target
- jzzj_visual_servo/enable: std_msgs/Bool
- jzzj_visual_servo/stop: std_msgs/Empty

## Output contract

- jzzj_visual_servo/cmd_vel: geometry_msgs/Twist
- jzzj_visual_servo/grasp_ready: std_msgs/Bool
- jzzj_visual_servo/state: std_msgs/String

The cmd_vel output is a placeholder interface for the chassis or arm alignment controller. It is zero when disabled, when the target is lost, or after stop. The package does not publish left-arm joint commands.

## Start

    cd mission_ws
    catkin_make
    source devel/setup.bash
    roslaunch jzzj_visual_servo jzzj_visual_servo.launch target_color:=red

Enable the controller:

    rostopic pub -1 /jzzj_visual_servo/enable std_msgs/Bool '{data: true}'

For a simple interface test:

    rostopic pub -1 /jzzj_visual_servo/target_visible std_msgs/Bool '{data: true}'
    rostopic pub -1 /jzzj_visual_servo/target_error geometry_msgs/Vector3 '{x: 0.2, y: 0.0, z: 0.3}'

The gains and limits are placeholders and must be calibrated on the robot. Keep the output disconnected from the real chassis during the first message-level test.
