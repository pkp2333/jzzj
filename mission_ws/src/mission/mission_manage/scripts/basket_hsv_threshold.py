#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import cv2
import threading
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


img_raw = None
img_lock = threading.Lock()


def nothing(x):
    pass


def image_raw_cb(msg: Image):
    global img_raw
    bridge = CvBridge()
    with img_lock:
        img_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')


def main():

    rospy.init_node('basket_seq_test_node', anonymous=True)
    sort_basket_rois = rospy.get_param('/mission_magage/sort_basket_rois', [[[400, 130], [480, 170]], 
                                                                            [[120, 140], [160, 200]], 
                                                                            [[0, 200], [50, 320]], 
                                                                            [[500, 260], [640, 380]]])
    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)
    
    img = None
    r = rospy.Rate(25)

    cv2.namedWindow("Result")
    cv2.createTrackbar("HL", "Result", 0, 255, nothing)
    cv2.createTrackbar("SL", "Result", 0, 255, nothing)
    cv2.createTrackbar("VL", "Result", 0, 255, nothing)
    cv2.createTrackbar("HU", "Result", 255, 255, nothing)
    cv2.createTrackbar("SU", "Result", 255, 255, nothing)
    cv2.createTrackbar("VU", "Result", 255, 255, nothing)

    while not rospy.is_shutdown():
        with img_lock:
            if img_raw is None:
                continue
            img = img_raw.copy()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        l_h = cv2.getTrackbarPos("HL", "Result")
        l_s = cv2.getTrackbarPos("SL", "Result")
        l_v = cv2.getTrackbarPos("VL", "Result")
        u_h = cv2.getTrackbarPos("HU", "Result")
        u_s = cv2.getTrackbarPos("SU", "Result")
        u_v = cv2.getTrackbarPos("VU", "Result")

        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])
        
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        result = cv2.bitwise_and(img, img, mask=mask)

        cv2.imshow("HSV", hsv)
        cv2.imshow("Result", result)
        key = cv2.waitKey(1) & 0xff
        if key == 27:
            print(f"\n\n\n\n\n[{lower[0]}, {lower[1]}, {lower[2]}], [{upper[0]}, {upper[1]}, {upper[2]}]\n\n\n\n\n")
            break

        r.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

    
