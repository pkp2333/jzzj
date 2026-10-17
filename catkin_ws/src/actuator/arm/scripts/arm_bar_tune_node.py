#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from arm_motions import radians_list

import rospy
from sensor_msgs.msg import JointState


def nothing(x):
    pass


def main():
    rospy.init_node("arm_bar_tune")

    arm_ns = rospy.get_namespace()
    arm_dir = arm_ns[5:-1]

    delta_servo = []
    for i in range(1, 7):
        delta_servo.append(rospy.get_param(f'delta_servo_{i}', 0))


    win_name = f"{arm_dir} Arm Tuner"
    cv2.namedWindow(win_name)

    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    arm_servo_degree_pub = rospy.Publisher(f"arm_joint_states", JointState, queue_size=5)

    cv2.createTrackbar("servo_1", win_name, 135, 270, nothing)
    cv2.createTrackbar("servo_2", win_name, 90, 180, nothing)
    cv2.createTrackbar("servo_3", win_name, 90, 180, nothing)
    cv2.createTrackbar("servo_4", win_name, 90, 180, nothing)
    cv2.createTrackbar("servo_5", win_name, 135, 270, nothing)
    cv2.createTrackbar("servo_6", win_name, 50, 100, nothing)
    cv2.createTrackbar("center", win_name, 0, 1, nothing)
    
    last = None

    while not rospy.is_shutdown():
        canvas = np.zeros((200, 250, 3))
        
        servo_1 = cv2.getTrackbarPos("servo_1", win_name)-135
        servo_2 = cv2.getTrackbarPos("servo_2", win_name)-90
        servo_3 = cv2.getTrackbarPos("servo_3", win_name)-90
        servo_4 = cv2.getTrackbarPos("servo_4", win_name)-90
        servo_5 = cv2.getTrackbarPos("servo_5", win_name)-135
        servo_6 = cv2.getTrackbarPos("servo_6", win_name)
        center = cv2.getTrackbarPos("center", win_name)
        
        texts = [
            f"servo_1:{servo_1}",
            f"servo_2:{servo_2}",
            f"servo_3:{servo_3}",
            f"servo_4:{servo_4}",
            f"servo_5:{servo_5}",
            f"servo_6:{servo_6}",
        ]
        
        for i, text in enumerate(texts):
            y = 30 + i*30
            cv2.putText(canvas, text, (10, y), 3, 1, (0, 0, 255), 2)

        cv2.imshow(win_name, canvas)

        servo_data = [servo_1, servo_2, servo_3, servo_4, servo_5, servo_6]
        
        if center:
            cv2.setTrackbarPos("servo_1", win_name, 135)
            cv2.setTrackbarPos("servo_2", win_name, 90)
            cv2.setTrackbarPos("servo_3", win_name, 90)
            cv2.setTrackbarPos("servo_4", win_name, 90)
            cv2.setTrackbarPos("servo_5", win_name, 135)
            cv2.setTrackbarPos("servo_6", win_name, 50)
            arm_servo_degree.position = radians_list([0, 0, 0, 0, 0, 50])
            arm_servo_degree_pub.publish(arm_servo_degree)
            cv2.setTrackbarPos("center", win_name, 0)
            last = [0, 0, 0, 0, 0, 50]
            center = 0 
        
        if last is None:
            last = servo_data
            arm_servo_degree.position = radians_list(servo_data)
            arm_servo_degree_pub.publish(arm_servo_degree)
        elif servo_data == last:
            pass
        else:
            arm_servo_degree.position = radians_list(servo_data)
            arm_servo_degree_pub.publish(arm_servo_degree)

        key = cv2.waitKey(100) & 0xff
        if key in [27, ord('q'), ord('Q')]:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()