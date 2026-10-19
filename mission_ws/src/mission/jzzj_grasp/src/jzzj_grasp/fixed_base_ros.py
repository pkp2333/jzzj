"""Fruit-driven scheduler: relative car_target/car_cmd, explicit ownership, no map/Tag."""
import math
import time
import numpy as np
from .core import Failure
from .fixed_base import correction,validate_base
from .taught import Snapshot


class NavigationAlignment:
    def __init__(self,io,cfg):
        from geometry_msgs.msg import PoseStamped
        from user_msgs.msg import CarCmd,StageCBaseStatus
        from jzzj_grasp_interfaces.msg import VisualBaseFeedback
        from jzzj_grasp_interfaces.srv import StageCControl,StageCControlRequest
        self.io,self.r,self.cfg=io,io.rospy,validate_base(cfg,io.cfg)
        self.pose_type,self.cmd,self.request_type=PoseStamped,CarCmd,StageCControlRequest
        self.feedback=None;self.drive=None;self.arrived=False;self.held=False;self.anchor=None
        self.trace=[];self.seq=int(time.time());self.aligning=False;self.phase='VISUAL';self.acquired=False
        self.stop_evidence='none';self.pregrasp_visual_stop=False
        self.goal=self.r.Publisher('/car_target',PoseStamped,queue_size=1)
        self.mode=self.r.Publisher('/car_cmd',CarCmd,queue_size=1)
        self.control=self.r.ServiceProxy('/stage_c/base/control',StageCControl)
        self.subs=[self.r.Subscriber(cfg['feedback_topic'],VisualBaseFeedback,self.position,queue_size=1),
                   self.r.Subscriber('/stage_c/base/drive_status',StageCBaseStatus,self.drive_status,queue_size=1),
                   self.r.Subscriber('/car_cmd',CarCmd,self.status,queue_size=10)]

    def set_state(self,state,reason=''):
        callback=getattr(self.io,'stage_state',None)
        if callback is not None:callback(state,reason)

    def position(self,msg):
        if msg.session_id!=self.io.session:return
        if not msg.valid or msg.header.frame_id!='stage_c_visual_local':
            self.feedback=None;return
        values=(msg.x,msg.y,*msg.fruit_position_m)
        if not all(math.isfinite(v) for v in values):self.feedback=None;return
        stamp=msg.header.stamp.to_sec()
        if self.feedback is None or stamp>self.feedback[1]:
            self.feedback=((msg.x,msg.y),stamp,time.monotonic(),tuple(msg.fruit_position_m),msg.track_id)

    def drive_status(self,msg):
        if getattr(msg,'_connection_header',{}).get('callerid')=='/chassis':self.drive=(msg,time.monotonic())

    def drive_stopped(self,after=0.):
        value=self.drive
        return (value is not None and time.monotonic()-value[1]<=.3 and value[0].serial_stop_written and
                value[0].header.stamp.to_sec()>=after and
                -.05<=self.r.Time.now().to_sec()-value[0].header.stamp.to_sec()<=.3)

    def current(self):
        data=self.feedback
        if data is None or time.monotonic()-data[2]>.5 or not -.05<=self.r.Time.now().to_sec()-data[1]<=.5:
            raise Failure('SENSOR_INVALID','fresh stationary-fruit relative feedback absent')
        return data[0],data[1]

    def status(self,msg):
        if getattr(msg,'_connection_header',{}).get('callerid')=='/stage_c_fixed_navigation' and msg.cmd==self.cmd.ARRIVE:
            self.arrived=True

    def graph(self,pubs,subs):
        own=self.r.get_name();nav='/stage_c_fixed_navigation'
        if pubs.get('/cmd_vel')!=[nav] or subs.get('/cmd_vel')!=['/chassis']:
            raise Failure('NOT_READY','exclusive stage-C adapter/chassis required')
        if pubs.get(self.cfg['feedback_topic'])!=[nav] or pubs.get('/stage_c/base/drive_status')!=['/chassis']:
            raise Failure('NOT_READY','visual feedback and actual driver zero-write evidence required')
        if set(pubs.get('/car_cmd',[]))!={own,nav} or pubs.get('/car_target')!=[own]:
            raise Failure('NOT_READY','another navigation publisher exists')
        if set(subs.get('/car_cmd',[]))!={own,nav,'/chassis'} or subs.get('/car_target')!=[nav]:
            raise Failure('NOT_READY','old Tag/navigation subscribers have not released command interfaces')
        if pubs.get('/stage_c/base/status') or pubs.get('/stage_c/base/command'):
            raise Failure('NOT_READY','old micro-base controller conflicts')

    def base_request(self,operation,confirmed=True,empty=False):
        req=self.request_type();req.operation=operation;req.session_id=self.io.session
        req.request_id=time.time_ns();req.issued_at=self.r.Time.now();req.expires_at=req.issued_at+self.r.Duration(3.)
        req.confirmed_stopped=confirmed;req.confirmed_empty_and_clear=empty
        response=self.control(req)
        if not response.accepted:raise Failure('NOT_READY','base ownership rejected: '+response.diagnostic)

    def ready(self):
        self.profile(True)
        if self.r.get_param('/chassis/fixed_navigation_watchdog',0)!=.3:raise Failure('NOT_READY','driver watchdog absent')
        if self.r.get_param('/chassis/fixed_calib_interface',False) is not True:
            raise Failure('NOT_READY','d-forward/cyy-right chassis adapter absent; update driver before motion')
        for key,value in (('max_vel_linear',.02),('activate_zone_xy',0.),('activate_zone_z',0.)):
            if self.r.get_param('/chassis/'+key,None)!=value:raise Failure('NOT_READY','bounded chassis profile mismatch: '+key)
        self.r.wait_for_service('/stage_c/base/control',timeout=3.)
        self.base_request(4);self.acquired=True;self.io.select(True)
        end=time.monotonic()+6.;reason='visual feedback not yet available'
        while time.monotonic()<end:
            try:self.io.check_graph(force=True);self.current();break
            except Exception as exc:reason=str(exc);time.sleep(.05)
        else:raise Failure('NOT_READY',reason)
        self.stop();self.set_state('OBSERVE','base acquired; fresh fruit-relative stop confirmed')

    def guard(self):
        if self.phase=='ARM':
            if not self.drive_stopped():raise Failure('STOP_FAILED','driver zero-write evidence lost during arm action')
            return
        position,_=self.current()
        if self.held and self.anchor is not None and math.dist(position,self.anchor)>.003:
            self.mode.publish(self.cmd(cmd=self.cmd.HOLD));raise Failure('STOP_FAILED','relative fruit moved after base stop')

    def stop(self,visual=None):
        if visual is None:visual=self.phase!='ARM'
        self.mode.publish(self.cmd(cmd=self.cmd.HOLD));self.held=False
        start=self.r.Time.now().to_sec();end=time.monotonic()+3.;samples=[];last=0.
        while time.monotonic()<end and not self.r.is_shutdown():
            self.mode.publish(self.cmd(cmd=self.cmd.HOLD))
            if visual:
                position,stamp=self.current()
                if stamp>last and stamp>=start:
                    last=stamp
                    if any(math.dist(position,p)>.002 for _,p in samples):samples=[]
                    samples.append((stamp,position))
                measured=len(samples)>=4 and samples[-1][0]-samples[0][0]>=1.
            else:measured=time.monotonic()>end-2.8
            if measured and self.drive_stopped(start):
                self.held=True
                if visual:
                    self.anchor=samples[-1][1];self.stop_evidence='stationary_fruit_relative_1s_and_serial_zero'
                else:self.stop_evidence='serial_zero_only_after_grasp; external stop confirmation required for handoff'
                return
            time.sleep(.05)
        raise Failure('STOP_FAILED','fresh stop evidence missing: '+('fruit stability and ' if visual else '')+'serial zero write')

    def prepare_grasp(self):
        self.set_state('STOP_CONFIRM','final fruit-relative stop before grasp');self.stop(visual=True)
        self.pregrasp_visual_stop=True;self.phase='ARM'
        self.set_state('ARM_GRASP','base held; fruit ceases to be an independent base reference during grasp')

    def observe(self,deadline,anchor=None):
        cfg=self.io.cfg;identity=cfg['identity']
        gate=Snapshot(self.io.session,cfg['arm_frame'],(identity['class_id'],identity['label'],identity['color']),
                      self.r.Time.now().to_sec(),anchor=anchor)
        end=min(deadline,time.monotonic()+6.)
        while time.monotonic()<end:
            self.io.guard()
            with self.io.lock:frames=list(self.io.frames);latest=self.io.last_received
            if latest is None or time.monotonic()-latest[0]>.5:raise Failure('SENSOR_INVALID','fruit stream lost')
            for frame in frames:
                point=gate.add(*frame,self.r.Time.now().to_sec())
                if point is not None:return point
            time.sleep(.02)
        raise Failure('NO_TARGET','no stable single fruit for one second')

    def navigate(self,delta):
        self.held=False;self.arrived=False;self.seq+=1;self.set_state('BASE_ADJUST','bounded relative-body step')
        origin,_=self.current();target=tuple(a+b for a,b in zip(origin,delta))
        msg=self.pose_type();msg.header.frame_id='stage_c_body';msg.header.stamp=self.r.Time.now();msg.header.seq=self.seq
        msg.pose.position.x,msg.pose.position.y=delta;msg.pose.orientation.w=1.
        self.mode.publish(self.cmd(cmd=self.cmd.NAVIGATION));time.sleep(.05);self.goal.publish(msg)
        end=time.monotonic()+11.
        while time.monotonic()<end:
            self.io.guard()
            if max(abs(a-b) for a,b in zip(self.io.arm.measured(),self.arm_anchor))>.035:
                raise Failure('MOTION_FAILED','arm changed during fruit-relative motion')
            self.mode.publish(self.cmd(cmd=self.cmd.NAVIGATION));position,_=self.current()
            if math.dist(position,origin)>.055:raise Failure('NEEDS_REPOSITION','measured visual step exceeds 55mm')
            if self.arrived and self.r.get_param('/stage_c_fixed_navigation/completed_seq',-1)==self.seq and math.dist(position,target)<=.002:
                self.set_state('STOP_CONFIRM','relative step arrived');self.stop();return
            time.sleep(.05)
        raise Failure('TIMEOUT','relative goal did not obtain fresh matching arrival evidence')

    def align(self):
        timeout=self.cfg['timeout_seconds'];deadline=time.monotonic()+timeout if timeout is not None else math.inf
        total=0.;measured=0.;attempt=0;self.aligning=True;self.arm_anchor=tuple(self.io.arm.measured());previous=None
        try:
            self.set_state('STOP_CONFIRM','rear ready; stop before observing');self.stop()
            while True:
                if len(self.trace)>1000:del self.trace[:-1000]
                self.io.guard();self.set_state('OBSERVE','measure stationary fruit')
                if time.monotonic()>deadline:raise Failure('TIMEOUT','optional alignment deadline')
                point=self.observe(deadline,previous);position,_=self.current()
                delta=correction(self.cfg,point,self.io.cfg['reference_position_m'])
                self.trace.append(dict(step='BASE_REMEASURE',position_m=list(point),relative_xy=list(position)))
                if delta is None:
                    self.set_state('STOP_CONFIRM','within 20mm; confirm stop');self.stop()
                    point=self.observe(deadline,point)
                    if correction(self.cfg,point,self.io.cfg['reference_position_m']) is not None:
                        raise Failure('NEEDS_REPOSITION','fruit changed after final stop')
                    return point
                distance=math.hypot(*delta)
                if ((self.cfg['max_steps'] is not None and attempt>=self.cfg['max_steps']) or
                        (self.cfg['total_m'] is not None and total+distance>self.cfg['total_m']+1e-9)):
                    raise Failure('NEEDS_REPOSITION','optional step/travel budget exhausted')
                self.trace.append(dict(step='BASE_RELATIVE_GOAL',body_delta_m=list(delta)))
                self.navigate(delta);total+=distance;attempt+=1;measured+=math.dist(position,self.current()[0])
                if self.cfg['total_m'] is not None and measured>self.cfg['total_m']:
                    raise Failure('NEEDS_REPOSITION','optional measured total exceeded')
                predicted=np.asarray(point,dtype=float);predicted[:2]+=np.asarray(self.cfg['base_matrix'])@delta
                previous=tuple(predicted)
        except Exception:self.mode.publish(self.cmd(cmd=self.cmd.HOLD));raise
        finally:self.aligning=False;self.io.deadline=time.monotonic()+180.

    def release(self,empty=False):
        # Only the FSM's explicit external stop acknowledgement can authorize this call.
        self.stop(visual=False);self.base_request(5,empty=empty);self.profile(False);self.acquired=False
        self.goal.unregister();self.mode.unregister()

    def profile(self,enabled):
        from std_srvs.srv import SetBool
        self.r.wait_for_service('/chassis/stage_c_profile',timeout=3.)
        response=self.r.ServiceProxy('/chassis/stage_c_profile',SetBool)(enabled)
        if not response.success:raise Failure('NOT_READY','driver profile switch rejected: '+response.message)
