#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import os
import cv2
import threading

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


frame_raw = None
frame_lock = threading.Lock()


def image_raw_cb(msg:Image):
    global frame_raw
    bridge = CvBridge()
    with frame_lock:
        frame_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')


def main():

    rospy.init_node('prepare_dataset_node', anonymous=True)

    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)

    dataset_path = "/home/orangepi/dataset/original"

    if not os.path.exists(dataset_path+"/images"):
        os.makedirs(dataset_path+"/images")
        print(f"创建文件夹 {dataset_path}/images")
    if not os.path.exists(dataset_path+"/labels"):
        os.makedirs(dataset_path+"/labels")
        print(f"创建文件夹 {dataset_path}/labels")

    src_image_names = os.listdir(dataset_path+"/images")
    src_image_names.sort(key=lambda x: int(x[:-4]))
    if src_image_names:
        cnt = int(os.path.splitext(src_image_names[-1])[0])
    else:
        cnt = 0
    print(f"已有 {cnt} 张照片")

    print("\n按空格拍摄，按Esc 或 q 退出")

    img = None
    r = rospy.Rate(25)

    while not rospy.is_shutdown():
        with frame_lock:
            if frame_raw is None:
                continue
            img = frame_raw.copy()

        cv2.imshow("camera", img)
        
        key = cv2.waitKey(1) & 0xff

        if key == 27:
            break
        elif key == 32:
            cnt += 1
            img_src = img.copy()
            cv2.namedWindow("Pic")
            cv2.moveWindow("Pic", 380, 20)
            cv2.putText(img, f"No. {cnt} picture", (7, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (125, 0, 255), 2)
            cv2.imshow("Pic", img)
            cv2.waitKey(600)
            cv2.imwrite(f"{dataset_path}/images/{cnt}.jpg", img)
            print(f"\n已保存 第{cnt}张 的照片")
            cv2.destroyWindow("Pic")
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
