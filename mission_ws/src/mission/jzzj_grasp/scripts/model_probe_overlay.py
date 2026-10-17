#!/usr/bin/env python3
"""RGB + ObjDets overlay only. No depth, target selection, action, base or arm commands."""
from collections import deque
import copy
import math
import threading
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from user_msgs.msg import ObjDets
from jzzj_grasp.fruit6 import FrameGate


class Overlay:
    def __init__(self):
        self.lock=threading.RLock();self.rgb=deque(maxlen=90);self.pending=deque(maxlen=20)
        self.bridge=CvBridge();self.gate=FrameGate(rospy.get_param('~max_age',.5))
        self.last_frame=None;self.last_received=None;self.last_output=None;self.started=__import__('time').monotonic()
        self.labels=rospy.get_param('~species_labels',['cherry','lemon'])
        self.image_pub=rospy.Publisher('/stage_c/model_probe/image',Image,queue_size=1)
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
        self.diag_types=(DiagnosticArray,DiagnosticStatus,KeyValue)
        self.status_pub=rospy.Publisher('/stage_c/model_probe/status',DiagnosticArray,queue_size=1,latch=True)
        self.status(1,'waiting for RGB and detections')
        rospy.Subscriber(rospy.get_param('~rgb_topic','/ascamera_hp60c/rgb0/image'),Image,self.on_rgb,queue_size=5)
        rospy.Subscriber(rospy.get_param('~detections_topic','/ascamera_hp60c/rgb0/yolov5_detections'),ObjDets,self.on_detection,queue_size=5)
        rospy.Timer(rospy.Duration(.03),self.tick)

    def status(self,level,message):
        Array,Status,Value=self.diag_types
        array=Array();array.header.stamp=rospy.Time.now()
        s=Status();s.name='stage_c/fruit6_model_probe';s.level=level;s.message=message
        s.values=[Value(key='semantic_mapping',value=','.join(self.labels)),
                  Value(key='motion_enabled',value='false')]
        array.status=[s];self.status_pub.publish(array)

    def on_rgb(self,msg):
        with self.lock:self.rgb.append(msg)

    def on_detection(self,msg):
        with self.lock:self.pending.append(msg)

    def tick(self,_):
        now=rospy.Time.now().to_sec()
        import time
        with self.lock:
            self.pending=deque((d for d in self.pending if d.header.stamp.to_sec()>0 and -.05<=now-d.header.stamp.to_sec()<=self.gate.max_age),maxlen=20)
            matched=[]
            for d in self.pending:
                for image in reversed(self.rgb):
                    if image.header.stamp==d.header.stamp and image.header.frame_id==d.header.frame_id:
                        if d.header.stamp.to_sec()>self.gate.last:matched.append((d,image))
                        break
            if not matched:
                if self.last_received is None and time.monotonic()-self.started>self.gate.max_age:
                    self.status(1,'waiting for a fresh matched RGB/detection frame')
                if self.last_received is not None and (time.monotonic()-self.last_received>self.gate.max_age or now-self.gate.last>self.gate.max_age):
                    self.status(1,'stale or repeated source frame: overlay invalid')
                    # Clear old boxes with an explicitly stamped invalid view, not another successful detection.
                    if self.last_frame is not None:
                        blank=self.last_frame.copy();cv2.putText(blank,'STALE - NOT A VALID DETECTION',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
                        m=self.bridge.cv2_to_imgmsg(blank,'bgr8');m.header=copy.deepcopy(self.last_output)
                        self.image_pub.publish(m)
                    self.last_received=None
                return
            det,image=max(matched,key=lambda pair:pair[0].header.stamp.to_sec())
            if not self.gate.accept(det.header.stamp.to_sec(),now):return
            self.last_received=time.monotonic()
        try:
            frame=self.bridge.imgmsg_to_cv2(image,'bgr8').copy();self.last_frame=frame.copy();self.last_output=copy.deepcopy(image.header)
            if det.num!=len(det.dets):raise ValueError('num/array count mismatch')
            h,w=frame.shape[:2]
            for item in det.dets:
                if not 0<=item.class_id<len(self.labels) or item.label!=self.labels[item.class_id]:raise ValueError('class/label mismatch')
                values=(item.bbox.x1,item.bbox.y1,item.bbox.x2,item.bbox.y2,item.score)
                if not all(math.isfinite(v) for v in values) or not 0<=item.score<=1:raise ValueError('nonfinite detection')
                x1,y1,x2,y2=values[:4]
                if x2<=x1 or y2<=y1:raise ValueError('inverted box')
                x1,y1=max(0,min(w-1,int(x1))),max(0,min(h-1,int(y1)))
                x2,y2=max(0,min(w-1,int(x2))),max(0,min(h-1,int(y2)))
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(frame,'%s %.3f #%d'%(item.label,item.score,item.index),(x1,max(15,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
            cv2.putText(frame,'fruit species: %d detections'%len(det.dets),(8,h-10),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,180,0),1)
            result=self.bridge.cv2_to_imgmsg(frame,'bgr8');result.header=image.header;self.image_pub.publish(result)
            self.status(0,'fresh source frame; '+str(len(det.dets))+' detections')
        except Exception as exc:
            if self.last_frame is not None and self.last_output is not None:
                invalid=self.last_frame.copy();cv2.putText(invalid,'INVALID DETECTION - SEE STATUS',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
                msg=self.bridge.cv2_to_imgmsg(invalid,'bgr8');msg.header=copy.deepcopy(self.last_output);self.image_pub.publish(msg)
            self.status(2,str(exc));rospy.logwarn_throttle(2.,'model probe overlay rejected: %s',exc)

if __name__=='__main__':
    rospy.init_node('fruit6_overlay');Overlay();rospy.spin()
