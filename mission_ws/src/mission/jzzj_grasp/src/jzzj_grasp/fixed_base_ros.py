"""Fixed-grasp scheduler: only car_target + car_cmd; no velocity publisher."""
import math
import time
from .core import Failure
from .fixed_base import correction,validate_base,map_pose,angular
from .taught import Snapshot


class NavigationAlignment:
    def __init__(self,io,cfg):
        from geometry_msgs.msg import PoseStamped
        from user_msgs.msg import CarCmd
        self.io,self.r,self.cfg=io,io.rospy,validate_base(cfg)
        self.pose_type,self.cmd=PoseStamped,CarCmd
        self.pose=None;self.arrived=False;self.held=False;self.moving=False;self.anchor=None
        self.trace=[];self.track=None;self.seq=int(time.time());self.aligning=False
        self.goal=self.r.Publisher('/car_target',PoseStamped,queue_size=1)
        self.mode=self.r.Publisher('/car_cmd',CarCmd,queue_size=1)
        self.subs=[self.r.Subscriber(cfg['pose_topic'],PoseStamped,self.position,queue_size=1),
                   self.r.Subscriber('/car_cmd',CarCmd,self.status,queue_size=10)]

    def position(self,msg):
        try:
            pose,stamp=map_pose(msg,self.r.Time.now().to_sec())
            if self.pose is None or stamp>self.pose[1]:self.pose=(pose,stamp,time.monotonic())
        except Exception:self.pose=None

    def current(self):
        data=self.pose
        if data is None or time.monotonic()-data[2]>.3 or not -.05<=self.r.Time.now().to_sec()-data[1]<=.3:
            raise Failure('SENSOR_INVALID','fresh map localization absent')
        return data[0],data[1]

    def status(self,msg):
        if getattr(msg,'_connection_header',{}).get('callerid')=='/stage_c_fixed_navigation' and msg.cmd==self.cmd.ARRIVE:
            self.arrived=True

    def graph(self,pubs,subs):
        own=self.r.get_name();nav='/stage_c_fixed_navigation'
        if pubs.get('/cmd_vel')!=[nav] or subs.get('/cmd_vel')!=['/chassis']:
            raise Failure('NOT_READY','exclusive bounded navigator/chassis required')
        if pubs.get(self.cfg['pose_topic'])!=['/car_localization_node']:
            raise Failure('NOT_READY','one canonical map localization publisher required')
        if set(pubs.get('/car_cmd',[]))!={own,nav} or pubs.get('/car_target')!=[own]:
            raise Failure('NOT_READY','another navigation command publisher exists')
        if set(subs.get('/car_cmd',[]))!={own,nav,'/chassis'} or subs.get('/car_target')!=[nav]:
            raise Failure('NOT_READY','unexpected navigation subscribers')
        if pubs.get('/stage_c/base/status') or pubs.get('/stage_c/base/command'):
            raise Failure('NOT_READY','legacy micro-base controller must be stopped')
        if self.r.get_param('/stage_c_fixed_navigation/bounded_micro_navigation',False) is not True:
            raise Failure('NOT_READY','bounded navigator not ready')

    def ready(self):
        if self.r.get_param('/chassis/fixed_navigation_watchdog',0)!=.3:
            raise Failure('NOT_READY','chassis command watchdog not enabled')
        for key,value in (('max_vel_linear',.02),('activate_zone_xy',0.),('activate_zone_z',0.)):
            if self.r.get_param('/chassis/'+key,None)!=value:
                raise Failure('NOT_READY','bounded chassis profile mismatch: '+key)
        self.stop();self.trace.append(dict(step='BASE_INITIAL_STOP_CONFIRMED'))

    def guard(self):
        pose,_=self.current()
        if self.held and self.anchor is not None:
            if math.dist(pose[:2],self.anchor[:2])>.003 or abs(angular(pose[2],self.anchor[2]))>math.radians(.5):
                self.mode.publish(self.cmd(cmd=self.cmd.HOLD))
                raise Failure('STOP_FAILED','base moved while arm was operating')

    def stop(self):
        self.mode.publish(self.cmd(cmd=self.cmd.HOLD));self.moving=False;self.held=False
        end=time.monotonic()+3.;samples=[];last=0.
        while time.monotonic()<end and not self.r.is_shutdown():
            self.mode.publish(self.cmd(cmd=self.cmd.HOLD))
            pose,stamp=self.current()
            if stamp>last:
                last=stamp
                if any(math.dist(pose[:2],p[:2])>.002 or abs(angular(pose[2],p[2]))>math.radians(.5) for _,p in samples):
                    samples=[]
                samples.append((stamp,pose))
                if len(samples)>=4 and stamp-samples[0][0]>=1.:
                    self.anchor=pose;self.held=True;return
            time.sleep(.05)
        raise Failure('STOP_FAILED','HOLD sent but fresh localization did not confirm one second stationary')

    def observe(self,deadline,anchor=None):
        cfg=self.io.cfg;identity=cfg['identity']
        gate=Snapshot(self.io.session,cfg['arm_frame'],(identity['class_id'],identity['label'],identity['color']),
                      self.r.Time.now().to_sec(),anchor=anchor)
        end=min(deadline,time.monotonic()+6.)
        while time.monotonic()<end:
            self.io.guard()
            with self.io.lock:frames=list(self.io.frames);latest=self.io.last_received
            if latest is None or time.monotonic()-latest[0]>.5:raise Failure('SENSOR_INVALID','fruit observations lost')
            for frame in frames:
                if frame[0]<=gate.last:continue
                for o in frame[3]:
                    if o.valid and (o.class_id,o.label,o.color)==(identity['class_id'],identity['label'],identity['color']):
                        if self.track is None:self.track=o.track_id
                        elif o.track_id!=self.track:raise Failure('NO_TARGET','fruit track changed during alignment')
                point=gate.add(*frame,self.r.Time.now().to_sec())
                if point is not None:return point
            time.sleep(.02)
        raise Failure('NO_TARGET','no stable single-fruit observation for base alignment')

    def navigate(self,target):
        self.held=False;self.moving=True;self.arrived=False;self.seq+=1
        msg=self.pose_type();msg.header.frame_id='map';msg.header.stamp=self.r.Time.now();msg.header.seq=self.seq
        msg.pose.position.x,msg.pose.position.y=target[:2]
        msg.pose.orientation.z=math.sin(target[2]/2);msg.pose.orientation.w=math.cos(target[2]/2)
        self.mode.publish(self.cmd(cmd=self.cmd.NAVIGATION));time.sleep(.05);self.goal.publish(msg)
        end=time.monotonic()+11.
        while time.monotonic()<end:
            self.io.guard()
            if max(abs(a-b) for a,b in zip(self.io.arm.measured(),self.arm_anchor))>.035:
                raise Failure('MOTION_FAILED','arm changed posture during base adjustment')
            self.mode.publish(self.cmd(cmd=self.cmd.NAVIGATION))
            pose,_=self.current()
            if (self.arrived and self.r.get_param('/stage_c_fixed_navigation/completed_seq',-1)==self.seq and
                    math.dist(pose[:2],target[:2])<=.002 and abs(angular(pose[2],target[2]))<math.radians(.5)):
                self.stop();return
            time.sleep(.05)
        raise Failure('TIMEOUT','micro map goal did not arrive with matching fresh feedback')

    def align(self):
        timeout=self.cfg['timeout_seconds']
        deadline=time.monotonic()+timeout if timeout is not None else math.inf
        total=0.;measured=0.;attempt=0;self.aligning=True
        self.arm_anchor=tuple(self.io.arm.measured());previous=None
        try:
            self.stop()
            while True:
                # Unlimited travel does not require unbounded in-memory telemetry.
                if len(self.trace)>1000:del self.trace[:len(self.trace)-1000]
                self.io.guard()
                if time.monotonic()>deadline:raise Failure('TIMEOUT','base alignment deadline')
                point=self.observe(deadline,previous);pose,_=self.current()
                update=correction(self.cfg,point,self.io.cfg['reference_position_m'],pose)
                self.trace.append(dict(step='BASE_REMEASURE',position_m=list(point),map_pose=list(pose)))
                if update is None:
                    self.stop()
                    point=self.observe(deadline,point)
                    if correction(self.cfg,point,self.io.cfg['reference_position_m'],self.current()[0]) is not None:
                        raise Failure('NEEDS_REPOSITION','fruit moved after final base stop')
                    return point
                target,body=update;distance=math.hypot(*body)
                if ((self.cfg['max_steps'] is not None and attempt>=self.cfg['max_steps']) or
                        (self.cfg['total_m'] is not None and total+distance>self.cfg['total_m']+1e-9)):
                    raise Failure('NEEDS_REPOSITION','base step/travel budget exhausted')
                self.trace.append(dict(step='BASE_MAP_GOAL',body_delta_m=list(body),map_target=list(target)))
                self.navigate(target);total+=distance;attempt+=1
                travelled=math.dist(pose[:2],self.current()[0][:2]);measured+=travelled
                if travelled>.055 or (self.cfg['total_m'] is not None and measured>self.cfg['total_m']):
                    raise Failure('NEEDS_REPOSITION','measured base travel budget exceeded')
                # Predicted association centre allows a legitimate <=50mm apparent fruit shift.
                import numpy as np
                predicted=np.asarray(point,dtype=float);predicted[:2]+=np.asarray(self.cfg['base_matrix'])@np.asarray(body)
                previous=tuple(predicted)
        except Exception:
            self.mode.publish(self.cmd(cmd=self.cmd.HOLD));raise
        finally:
            self.aligning=False;self.io.deadline=time.monotonic()+180.
