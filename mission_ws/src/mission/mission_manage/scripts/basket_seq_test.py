#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import threading
from utils import sort_basket_color_seq

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


img_raw = None
img_lock = threading.Lock()


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

    while not rospy.is_shutdown():
        with img_lock:
            if img_raw is None:
                continue
            img = img_raw.copy()

        basket_seq = sort_basket_color_seq(img, sort_basket_rois)
        # print(basket_seq)
        r.sleep()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

    
