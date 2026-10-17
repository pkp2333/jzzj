#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
from user_msgs.msg import PID


def nothing(x):
    pass


def main():
    rospy.init_node("pid_tuner")

    nh = rospy.get_namespace()
    
    pid_target = nh[1:-1]
    last_dash_pos = pid_target.rfind('_') + 1
    axel = pid_target[last_dash_pos:]

    range_default = [2.0, 0.5, 2.0, 0.5, 0.5]
    param_base = f"/wp_nav/{pid_target}/"
    kp = rospy.get_param(param_base + "kp", 0.0)
    ki = rospy.get_param(param_base + "ki", 0.0)
    kd = rospy.get_param(param_base + "kd", 0.0)
    i_a = rospy.get_param(param_base + "i_a", 0.0)
    o_t = rospy.get_param(param_base + "o_t", 0.0)
    range_params = rospy.get_param(param_base + "tune_range", range_default)

    if len(range_params) != 5:
        rospy.logerr("检查yaml文件中的 'tune_range' 参数数量")
        return

    win_name = f"PID-{axel} Tuner"
    cv2.namedWindow(win_name)

    pid_pub = rospy.Publisher(f"/wp_nav_pid/pid_{axel}", PID, queue_size=5)
    pid_msg = PID(p=kp,
                  i=ki,
                  d=kd,
                  i_activate=i_a,
                  out_thresh=o_t)

    def calc_pos(val, idx):
        return int(val * 10000 / range_params[idx])
    
    trackbars = [
        ("p", calc_pos(kp, 0)),
        ("i", calc_pos(ki, 1)),
        ("d", calc_pos(kd, 2)),
        ("i-a", calc_pos(i_a, 3)),
        ("o-t", calc_pos(o_t, 4))
    ]
    
    for name, pos in trackbars:
        cv2.createTrackbar(name, win_name, 0, 10000, nothing)
        cv2.setTrackbarPos(name, win_name, pos)

    # 主循环
    while not rospy.is_shutdown():
        # 创建显示画布
        canvas = cv2.cvtColor(np.zeros((200, 280), dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        
        # 从滑动条获取参数值
        params = [(cv2.getTrackbarPos(name, win_name) * range_params[i] / 10000) for i, (name, _) in enumerate(trackbars)
        ]
        kp, ki, kd, i_a, o_t = params
        
        # 显示参数值
        texts = [
            f"PID-{axel}",
            f"{axel} P   : {kp:.4f}",
            f"{axel} I    : {ki:.4f}",
            f"{axel} D   : {kd:.4f}",
            f"{axel} I-A : {i_a:.4f}",
            f"{axel} O-T : {o_t:.4f}"
        ]
        
        for i, text in enumerate(texts):
            y = 30 + i*30
            cv2.putText(canvas, text, (20, y), 3, 0.8, (0, 0, 255), 2)

        cv2.imshow(win_name, canvas)

        # 参数更新检测
        if any([
            pid_msg.p != kp,
            pid_msg.i != ki,
            pid_msg.d != kd,
            pid_msg.i_activate != i_a,
            pid_msg.out_thresh != o_t
        ]):
            pid_msg.p = kp
            pid_msg.i = ki
            pid_msg.d = kd
            pid_msg.i_activate = i_a
            pid_msg.out_thresh = o_t
            pid_pub.publish(pid_msg)

        # ESC退出处理
        key = cv2.waitKey(100)
        if key == 27:  # ESC键
            print(f"\n\n\033[1m\33[32m{'#'*60}\33[0m")
            print(f"\n  {pid_target}:"
                  f"\n    kp: {kp}"
                  f"\n    ki: {ki}"
                  f"\n    kd: {kd}"
                  f"\n    i_a: {i_a}  # °"
                  f"\n    o_t: {o_t}  # °"
                  f"\n    tune_range: {range_params}  # pid调节器范围\n\n")
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass