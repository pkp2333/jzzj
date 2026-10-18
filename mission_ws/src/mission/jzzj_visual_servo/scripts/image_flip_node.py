#!/usr/bin/env python3
"""把180度倒装的相机画面转正后转发，仅用于人工预览，不改变任何下游算法的输入。"""
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageFlipNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/ascamera_hp60c/rgb0/image_upright', Image, queue_size=1)
        rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.cb, queue_size=1)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        flipped = cv2.flip(img, -1)  # 水平+垂直 = 180度转正
        out = self.bridge.cv2_to_imgmsg(flipped, 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('image_flip_node')
    ImageFlipNode()
    rospy.spin()
