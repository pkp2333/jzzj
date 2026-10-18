#!/usr/bin/env python3
"""标定点捕获: 用法 python3 capture_point.py X Y
等待一个带颜色掩膜的检出帧, 记录掩膜质心像素与臂系坐标对应关系到 calib_points.json"""
import rospy, numpy as np, json, sys, time, os
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
from collections import deque
sys.path.insert(0,str(PROJECT/'mission_ws/src/mission/jzzj_visual_servo/src'))
from sensor_msgs.msg import Image, CameraInfo
from user_msgs.msg import ObjDets
from jzzj_grasp_interfaces.msg import TargetSelection
from cv_bridge import CvBridge
from jzzj_visual_servo.perception import colour_mask

X,Y=float(sys.argv[1]),float(sys.argv[2])
Z=-0.05
ARM=(X,Y,Z)
rospy.init_node('calib_capture', anonymous=True, disable_signals=True)
b=CvBridge()
p=json.load(open(str(PROJECT/'deploy/stage_c.yaml')))['perception']
rgb_q=deque(maxlen=200);det_q=deque(maxlen=40)
rospy.Subscriber(p['rgb_topic'],Image,lambda m:rgb_q.append(m),queue_size=5)
rospy.Subscriber(p['detections_topic'],ObjDets,lambda m:det_q.append(m),queue_size=5)
sel=rospy.wait_for_message('/stage_c/selection',TargetSelection,timeout=4)
color=sel.target_color
t0=time.time();cap=None
while time.time()-t0<15 and cap is None:
    rospy.sleep(0.05)
    for d in list(det_q):
        if d.num==0:continue
        rgb=next((m for m in reversed(rgb_q) if abs((m.header.stamp-d.header.stamp).to_sec())<1e-6),None)
        if rgb is None:continue
        det_q.remove(d)
        x=next((x for x in d.dets if x.label in p['allowed_labels']),None)
        if x is None:continue
        bgr=b.imgmsg_to_cv2(rgb,'bgr8')
        x1,y1=max(0,int(x.bbox.x1)),max(0,int(x.bbox.y1));x2,y2=min(bgr.shape[1],int(x.bbox.x2)),min(bgr.shape[0],int(x.bbox.y2))
        m=colour_mask(bgr[y1:y2,x1:x2],p['colors'][color])
        c=int(m.sum())
        if c<p['min_color_pixels']:continue
        vv,uu=np.nonzero(m)
        cap=(float((uu+x1).mean()),float((vv+y1).mean()),c,(x1,y1,x2-x1,y2-y1),bgr)
        break
if cap is None:
    print('FAIL: 15秒内未捕获到带颜色掩膜的检出');sys.exit(1)
u,v,c,box,bgr=cap
print('捕获: 检测框%s 掩膜px=%d 质心像素=(%.1f, %.1f)'%(box,c,u,v))
f=str(PROJECT/'deploy/calib_points.json')
pts=json.load(open(f)) if os.path.exists(f) else []
pts=[pt for pt in pts if not (abs(pt['x']-X)<1e-6 and abs(pt['y']-Y)<1e-6)]
pts.append({'u':u,'v':v,'x':X,'y':Y,'z':Z})
json.dump(pts,open(f,'w'),indent=1)
print('已记录 (%.3f, %.3f) 共%d点'%(X,Y,len(pts)))
