#!/usr/bin/env python3
"""Only YOLO boxes may create observations. Sensor callbacks never move hardware."""
from collections import deque
import threading
import copy
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from user_msgs.msg import ObjDets
from jzzj_grasp_interfaces.msg import TargetSelection, TargetObservation, TargetObservations
from jzzj_visual_servo.perception import Candidate, DetectorAdapter, project_depth_to_rgb
from jzzj_visual_servo.coordinate_info import coordinate_info

class Observer:
    def __init__(self):
        self.cfg=rospy.get_param('~perception')
        self.adapter=DetectorAdapter(self.cfg)
        self.mode=self.cfg.get('localization_mode','depth')
        if self.mode not in ('depth','plane'):raise ValueError('invalid localization_mode')
        self.tune=rospy.get_param('~tune_color',False)
        self.log_coordinates=rospy.get_param('~log_arm_coordinates',True)
        self.grasp_profile_name=rospy.get_param('~grasp_profile','tabletop')
        self.grasp_profile=rospy.get_param('~execution/profiles',{}).get(self.grasp_profile_name,{})
        if self.mode=='plane':
            for key in ('vertical_axis','table_plane_offset_m'):
                if key not in self.grasp_profile:raise ValueError('grasp profile missing %s for plane mode'%key)
        self.bridge=CvBridge();self.lock=threading.RLock()
        self.rgb=deque(maxlen=120);self.depth=deque(maxlen=120);self.pending=deque(maxlen=15)
        self.rgb_info=self.depth_info=None;self.selection=None;self.last=0.
        self.pub=rospy.Publisher('/stage_c/observations',TargetObservations,queue_size=2)
        self.single=rospy.Publisher('/stage_c/candidate_observation',TargetObservation,queue_size=2)
        self.debug=rospy.Publisher('/stage_c/debug_image',Image,queue_size=1)
        rospy.Subscriber('/stage_c/selection',TargetSelection,self.select,queue_size=1)
        rospy.Subscriber(self.cfg['rgb_topic'],Image,lambda m:self.cache(self.rgb,m),queue_size=5)
        rospy.Subscriber(self.cfg['rgb_info_topic'],CameraInfo,self.info,callback_args='rgb',queue_size=1)
        if self.mode=='depth':
            rospy.Subscriber(self.cfg['depth_topic'],Image,lambda m:self.cache(self.depth,m),queue_size=5)
            rospy.Subscriber(self.cfg['depth_info_topic'],CameraInfo,self.info,callback_args='depth',queue_size=1)
        rospy.Subscriber(self.cfg['detections_topic'],ObjDets,self.detect,queue_size=5)
        rospy.Timer(rospy.Duration(.02),self.tick)

    def cache(self,q,m):
        with self.lock:q.append(m)

    def info(self,m,kind):
        with self.lock:
            if kind=='rgb':self.rgb_info=m
            else:self.depth_info=m

    def select(self,m):
        with self.lock:
            self.selection=m;self.last=0.;self.pending.clear()

    def detect(self,m):
        with self.lock:self.pending.append(m)

    def tick(self,_):
        with self.lock:
            s=self.selection
            if not s or not s.enabled:return
            now=rospy.Time.now().to_sec()
            self.pending=deque((d for d in self.pending if now-d.header.stamp.to_sec()<=self.cfg['max_age']),maxlen=15)
            available=[]
            for d in self.pending:
                stamp=d.header.stamp.to_sec()
                if stamp<=self.last or stamp<=0 or now-stamp<-.05:continue
                rgb=next((m for m in reversed(self.rgb) if abs((m.header.stamp-d.header.stamp).to_sec())<1e-6),None)
                if self.mode=='plane':
                    # No depth stream to pair; the calibrated table plane provides range.
                    if rgb:available.append((d,rgb,None))
                    continue
                dep=min(self.depth,key=lambda m:abs((m.header.stamp-d.header.stamp).to_sec()),default=None)
                if rgb and dep and abs((dep.header.stamp-d.header.stamp).to_sec())<=self.cfg['sync_tolerance']:
                    available.append((d,rgb,dep))
            if not available:return
            d,rgb,dep=available[-1];self.last=d.header.stamp.to_sec()
            ri,di=self.rgb_info,self.depth_info
        output=TargetObservations();output.header=copy.deepcopy(d.header);output.header.frame_id=self.cfg['arm_frame'];output.session_id=s.session_id
        try:
            if not self.cfg['calibration_confirmed']:raise ValueError('camera calibration not confirmed')
            if ri is None or (self.mode=='depth' and di is None):raise ValueError('camera_info missing')
            if (ri.width,ri.height)!=(rgb.width,rgb.height) or (self.mode=='depth' and (di.width,di.height)!=(dep.width,dep.height)):
                raise ValueError('camera_info dimensions mismatch')
            if rgb.header.frame_id!=self.cfg['rgb_frame'] or d.header.frame_id!=rgb.header.frame_id:
                raise ValueError('RGB/detection frame mismatch')
            if ri.header.frame_id!=rgb.header.frame_id or (self.mode=='depth' and di.header.frame_id!=dep.header.frame_id):
                raise ValueError('camera_info frame mismatch')
            if self.mode=='depth' and dep.header.frame_id!=self.cfg['depth_frame']:raise ValueError('unexpected depth frame')
            if any(abs(v)>1e-9 for v in ri.D) or (self.mode=='depth' and any(abs(v)>1e-9 for v in di.D)):
                raise ValueError('use rectified image/camera_info streams (nonzero distortion rejected)')
            bgr=self.bridge.imgmsg_to_cv2(rgb,'bgr8')
            if self.mode=='plane':
                meters=None
            else:
                depth=self.bridge.imgmsg_to_cv2(dep,'passthrough')
                scales=self.cfg['depth_scales']
                if dep.encoding not in scales:raise ValueError('unsupported/unconfigured depth encoding')
                scale=float(scales[dep.encoding])
                if not np.isfinite(scale) or scale<=0:raise ValueError('invalid configured depth scale')
                if self.cfg['depth_registered_to_rgb']:
                    if depth.shape!=bgr.shape[:2]:raise ValueError('registered depth size mismatch')
                    if not np.allclose(di.K,ri.K,rtol=1e-5,atol=1e-5):raise ValueError('registered depth intrinsics must match RGB')
                    meters=depth.astype(float)*scale
                else:
                    if not self.cfg['registration_calibrated']:raise ValueError('depth registration extrinsics not confirmed')
                    meters=project_depth_to_rgb(depth,di.K,ri.K,self.cfg['depth_to_rgb'],bgr.shape[:2],scale)
            if self.tune:
                import cv2
                window='Stage C HSV (YOLO ROIs only)'
                if not getattr(self,'tune_ready',False):
                    cv2.namedWindow(window)
                    initial=self.cfg['colors'][s.target_color][0]
                    for name,value,limit in zip(('H low','S low','V low','H high','S high','V high'),initial[0]+initial[1],(180,255,255,180,255,255)):
                        cv2.createTrackbar(name,window,int(value),limit,lambda _:None)
                    self.tune_ready=True
                values=[cv2.getTrackbarPos(n,window) for n in ('H low','S low','V low','H high','S high','V high')]
                self.cfg['colors'][s.target_color]=[[values[:3],values[3:]]]
                cv2.imshow(window,bgr);cv2.waitKey(1)
                rospy.loginfo_throttle(3.,'HSV tuning %s: %s',s.target_color,self.cfg['colors'][s.target_color])
            candidates=[Candidate(x.index,x.class_id,x.label,x.score,(x.bbox.x1,x.bbox.y1,x.bbox.x2,x.bbox.y2)) for x in d.dets]
            if self.mode=='plane':
                plane=(tuple(float(v) for v in self.grasp_profile['vertical_axis']),
                       float(self.grasp_profile['table_plane_offset_m']))
                results=self.adapter.extract(bgr,None,ri.K,candidates,s.target_color,plane=plane)
            else:
                results=self.adapter.extract(bgr,meters,ri.K,candidates,s.target_color)
            for c,p,q,box in results:
                o=TargetObservation();o.header=output.header;o.session_id=s.session_id;o.track_id=c.track_id
                o.class_id=c.class_id;o.label=c.label;o.color=s.target_color;o.position=Point(*p);o.quality=q;o.valid=True
                o.roi.x_offset,o.roi.y_offset,o.roi.width,o.roi.height=box
                output.targets.append(o)
            output.diagnostic='ok'
            if self.debug.get_num_connections():
                import cv2
                image=bgr.copy()
                for o in output.targets:
                    x,y,w,h=o.roi.x_offset,o.roi.y_offset,o.roi.width,o.roi.height
                    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(image,'%s %s %.3fm'%(o.label,o.color,o.position.z),(x,y+15),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
                msg=self.bridge.cv2_to_imgmsg(image,'bgr8');msg.header=rgb.header;self.debug.publish(msg)
        except Exception as exc:
            output.diagnostic=str(exc);rospy.logwarn_throttle(2.,'stage C observation rejected: %s',exc)
        with self.lock:
            if self.selection is not s:return
        self.pub.publish(output)
        for o in output.targets:
            self.single.publish(o)
            if self.log_coordinates and o.valid:
                rospy.loginfo(coordinate_info(
                    o.header.frame_id, o.header.stamp.to_sec(), o.track_id,
                    o.label, o.color, (o.position.x, o.position.y, o.position.z),
                    self.grasp_profile_name, self.grasp_profile))

if __name__=='__main__':
    rospy.init_node('stage_c_observer');Observer();rospy.spin()
