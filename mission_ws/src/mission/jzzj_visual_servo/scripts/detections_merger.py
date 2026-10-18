#!/usr/bin/env python3
"""合并YOLO左右分区检测结果为单一话题，供stage C视觉闭环observer订阅。

部署的YOLO节点只发布 yolov5_detections_left/_right 两个分区话题，
stage_c observer 订阅的是合并话题 yolov5_detections。本节点把两侧
检测结果按时间戳合并转发，不做任何过滤。
"""
import rospy
from user_msgs.msg import ObjDets


class DetectionsMerger:
    def __init__(self):
        self.left = None
        self.right = None
        self.stamp = 0.0
        self.pub = rospy.Publisher('/ascamera_hp60c/rgb0/yolov5_detections', ObjDets, queue_size=1)
        rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections_left', ObjDets, self.cb, callback_args='left', queue_size=1)
        rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections_right', ObjDets, self.cb, callback_args='right', queue_size=1)

    def cb(self, msg, side):
        # 以最新一侧的时间戳为基准，合并两侧在20ms内的检测
        t = msg.header.stamp.to_sec()
        other = self.right if side == 'left' else self.left
        if other is not None and abs(other.header.stamp.to_sec() - t) < 0.04:
            merged = ObjDets()
            merged.header = msg.header
            merged.fps = msg.fps
            merged.dets = list(msg.dets) + list(other.dets)
            merged.num = len(merged.dets)
            self.pub.publish(merged)
        setattr(self, side, msg)


if __name__ == '__main__':
    rospy.init_node('detections_merger')
    DetectionsMerger()
    rospy.spin()
