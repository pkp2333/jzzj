"""ROS transports for taught execution. Only forward_only kinematics is allowed."""
import copy
import hashlib
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from .core import Failure, Observation
from .backends import RosArmBackend
from .planning import PathPlanner
from .taught import Snapshot, digest


class TaughtArm(RosArmBackend):
    def solve(self,point,orientation,guard,forward=False,seed=None,endpoint=False):
        if not forward or endpoint:raise Failure('NOT_READY','inverse kinematics forbidden in taught mode')
        return super().solve(None,None,guard,forward=True,seed=seed)

    def ready(self,guard):
        guard()
        for k in ('hardware_enabled','calibration_confirmed','workspace_clear_confirmed'):
            if self.rospy.get_param('/arm_left/arm_trajectory/'+k,False) is not True:
                raise Failure('NOT_READY','arm driver gate not confirmed: '+k)
        if (self.rospy.get_param('/arm_left/arm_trajectory/servo_position_min',96)!=96 or
                self.rospy.get_param('/arm_left/arm_trajectory/servo_position_max',4000)!=4000):
            raise Failure('NOT_READY','taught mode requires raw servo command range 96..4000')
        for k in ('real_motion_enabled','robot_calibration_confirmed','workspace_clear_confirmed','model_validated'):
            if self.cfg.get(k) is not True:raise Failure('NOT_READY','execution gate: '+k)
        if not self.client.wait_for_server(self.rospy.Duration(2.)):raise Failure('NOT_READY','trajectory server absent')
        self.rospy.wait_for_service('/stage_c/compute_arm_motion',timeout=2.)
        self.fresh_joints()


class BaseClient:
    def __init__(self,rospy,session,cancel):
        from user_msgs.msg import StageCBaseCommand,StageCBaseStatus
        self.rospy,self.session,self.cancel=rospy,session,cancel;self.message_type=StageCBaseCommand
        self.lock=threading.RLock();self.status=None;self.received=0.;self.timer=None
        self.operation=0;self.velocity=(0.,0.);self.operator_confirmed=False
        self.counter=time.time_ns();self.requested=0;self.owned=False
        self.pub=rospy.Publisher('/stage_c/base/command',StageCBaseCommand,queue_size=1)
        self.sub=rospy.Subscriber('/stage_c/base/status',StageCBaseStatus,self.callback,queue_size=1)

    def callback(self,msg):
        with self.lock:self.status=msg;self.received=time.monotonic()

    def publish(self,_=None):
        with self.lock:
            m=self.message_type();m.header.stamp=self.rospy.Time.now();m.session_id=self.session
            self.counter+=1;m.command_id=self.counter;m.operation=self.operation
            m.vx,m.vy=self.velocity;m.operator_confirmed=self.operator_confirmed;self.pub.publish(m)

    def switch(self,operation,velocity=(0.,0.),confirmed=False):
        with self.lock:
            self.operation,self.velocity,self.operator_confirmed=operation,velocity,confirmed
            self.requested=self.counter+1;self.publish()

    def fresh_status(self):
        with self.lock:m=self.status;age=time.monotonic()-self.received
        if m is None or age>.3 or not -.05<=(self.rospy.Time.now()-m.header.stamp).to_sec()<=.3:
            raise Failure('NOT_READY','base status stale or absent')
        return m

    def wait(self,phases,stop=False,released=False):
        end=time.monotonic()+2.
        while time.monotonic()<end:
            if self.cancel.is_set() and not stop:raise Failure('CANCELED','operator stop')
            try:
                m=self.fresh_status()
                if m.phase=='FAULT' and m.session_id==self.session and 'FAULT' not in phases and self.operation!=4:
                    raise Failure('MOTION_FAILED','base fault: '+m.diagnostic)
                owner='' if released else self.session
                if m.session_id==owner and m.command_id>=self.requested and m.phase in phases and (not stop or m.serial_stop_written):return
            except Failure as exc:
                if exc.code=='MOTION_FAILED':raise
            time.sleep(.02)
        raise Failure('STOP_FAILED' if stop else 'NOT_READY','base acknowledgement deadline')

    def acquire(self):
        end=time.monotonic()+2
        while self.pub.get_num_connections()==0 and time.monotonic()<end:time.sleep(.02)
        if self.pub.get_num_connections()<1:raise Failure('NOT_READY','micro base driver absent')
        self.switch(0);self.timer=self.rospy.Timer(self.rospy.Duration(.05),self.publish)
        self.wait(('ALIGN',),stop=True);self.owned=True

    def hold(self,locked=False):
        self.switch(3 if locked else 2)
        self.wait(('LOCKED',) if locked else ('ALIGN','LOCKED'),stop=True)
        return self.rospy.Time.now().to_sec()

    def stop(self):
        try:
            self.switch(3);self.wait(('LOCKED','FAULT'),stop=True);return True
        except Exception:return False

    def guard(self):
        if self.owned:
            m=self.fresh_status()
            if m.session_id!=self.session or m.phase not in ('ALIGN','LOCKED'):
                raise Failure('MOTION_FAILED','base ownership/lock lost: '+m.diagnostic)

    def release(self):
        self.switch(4,confirmed=True);self.wait(('IDLE',),released=True)
        self.owned=False

    def close(self):
        if self.timer:self.timer.shutdown();self.timer=None
        # No implicit release. Lost heartbeat latches HOLD until operator recovery.


class RosAdapter:
    def __init__(self,rospy,cfg,template,session,cancel):
        from jzzj_grasp_interfaces.msg import TargetObservations,TargetSelection
        from std_msgs.msg import Bool
        self.rospy,self.cfg,self.template,self.session,self.cancel=rospy,cfg,template,session,cancel
        cfg['active_profile']='tabletop';self.arm=TaughtArm(cfg);self.base=BaseClient(rospy,session,cancel)
        self.frames=deque(maxlen=150);self.frame_lock=threading.RLock();self.last_frame=None
        self.after=0.;self.arm_started=False;self.deadline=time.monotonic()+360.
        self.capture_identity=None;self.selection_type=TargetSelection
        self.pub=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
        self.sub=rospy.Subscriber('/stage_c/observations',TargetObservations,self.observation,queue_size=10)
        self.stop_sub=rospy.Subscriber('/stage_c/taught/stop',Bool,lambda m:cancel.set() if m.data else None,queue_size=1)
        from sensor_msgs.msg import JointState
        self.arm.joint_cb(rospy.wait_for_message('/arm_left/joint_states',JointState,timeout=5.))

    def observation(self,msg):
        targets=[Observation(o.header.stamp.to_sec(),o.session_id,o.header.frame_id,
                 (o.position.x,o.position.y,o.position.z),o.color,o.label,o.class_id,o.track_id,
                 o.quality,o.valid) for o in msg.targets]
        frame=(msg.header.stamp.to_sec(),msg.session_id,msg.diagnostic,targets)
        with self.frame_lock:self.frames.append(frame);self.last_frame=(time.monotonic(),frame)

    def select(self,enabled=True):
        m=self.selection_type(session_id=self.session,target_color=self.template.get('identity',{}).get('color','yellow'),enabled=enabled)
        self.pub.publish(m)

    def bindings(self):
        from sensor_msgs.msg import CameraInfo
        p=self.rospy.get_param('/stage_c_observer/perception')
        info=self.rospy.wait_for_message(p['rgb_info_topic'],CameraInfo,timeout=5.)
        observer_profile=self.rospy.get_param('/stage_c_observer/execution/profiles/tabletop')
        kin={k:self.rospy.get_param('/stage_c_kinematics/'+k) for k in ('tcp_offset_m','urdf')}
        kin['urdf_sha256']=hashlib.sha256(Path(kin.pop('urdf')).read_bytes()).hexdigest()
        arm={k:self.rospy.get_param('/arm_left/arm_trajectory/'+k) for k in
             ('servo_signs','servo_offsets_deg','servo_position_min','servo_position_max')}
        return dict(camera=digest(dict(perception=p,K=list(info.K),D=list(info.D),size=[info.width,info.height],frame=info.header.frame_id)),
                    table=digest({k:observer_profile[k] for k in ('vertical_axis','table_plane_offset_m')}),
                    tcp_model=digest(kin),servo=digest(arm))

    def guard(self):
        if self.cancel.is_set() or self.rospy.is_shutdown():raise Failure('CANCELED','operator/shutdown stop')
        if time.monotonic()>self.deadline:raise Failure('TIMEOUT','taught execution deadline')
        self.arm.fresh_joints();self.base.guard()
        if self.base.owned and not self.arm_started:self.check_start(self.template['start_rad'])

    def ready(self):
        import rosgraph
        if self.rospy.get_param('/stage_c/taught/recovery_required',False):
            raise Failure('NOT_READY','previous taught session requires explicit operator release')
        if self.template['arm_frame']!=self.cfg['arm_frame'] or self.template['table_normal']!=self.cfg['profiles']['tabletop']['vertical_axis']:
            raise Failure('NOT_READY','template arm frame/table normal differs from execution calibration')
        for k in ('stop_failed_latched','holding_latched'):
            if self.rospy.get_param('/stage_c_server/'+k,False):raise Failure('NOT_READY','C server '+k)
        pubs,_,_=rosgraph.Master(self.rospy.get_name()).getSystemState();pubs=dict(pubs)
        if len(pubs.get('/stage_c/base/status',[]))!=1:
            raise Failure('NOT_READY','missing/duplicate micro base status publisher')
        if pubs.get('/arm_left/joint_states')!=['/arm_left/arm_trajectory']:
            raise Failure('NOT_READY','missing/duplicate canonical arm driver')
        if pubs.get('/cmd_vel') or pubs.get('/car_cmd'):
            raise Failure('NOT_READY','stop other navigation/teleop publishers before taught test')
        self.arm.ready(self.guard)

    def check_start(self,expected):
        if max(abs(a-b) for a,b in zip(self.arm.measured(),expected))>.035:
            raise Failure('NOT_READY','arm differs from taught starting pose')

    def wait_arm_still(self):
        end=time.monotonic()+8.;previous=None;last=0;stable=0
        while time.monotonic()<end:
            self.guard();m=self.arm.fresh_joints();stamp=m.header.stamp.to_sec();q=self.arm.measured()
            if stamp>last:
                stable=stable+1 if previous and max(abs(a-b) for a,b in zip(q,previous))<=.01 else 0
                if stable>=3:return
                previous=q;last=stamp
            time.sleep(.02)
        raise Failure('MOTION_FAILED','arm did not settle across four fresh scans')

    def preflight(self,steps):
        names=self.cfg['joint_names'];template=copy.deepcopy(self.arm.fresh_joints())
        def forward(q):
            from .planning import Pose
            seed=copy.deepcopy(template);seed.name=names;seed.position=q
            p=self.arm.solve(None,None,self.guard,forward=True,seed=seed).tcp_pose.pose
            return Pose((p.position.x,p.position.y,p.position.z),(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
        lower=self.rospy.get_param('/arm_left/arm_trajectory/joint_lower_rad')
        upper=self.rospy.get_param('/arm_left/arm_trajectory/joint_upper_rad')
        planner=PathPlanner(self.cfg,lower,upper,forward,None,self.guard)
        self.fixed_planner=planner
        current=self.arm.measured()
        for step in steps:
            q=tuple(step['positions_rad']);planner.valid_joints(q)
            if step['phase'] in ('open','close'):
                planner.valid_pose(forward(q),planner.floor)
            else:planner.edge(current,q)
            current=q

    def acquire_base(self):
        try:self.base.acquire()
        finally:
            try:
                if self.base.fresh_status().session_id==self.session:
                    self.rospy.set_param('/stage_c/taught/recovery_required',True)
            except Failure:pass
        self.after=self.rospy.Time.now().to_sec()
    def hold_base(self,locked=False):self.after=self.base.hold(locked)
    def lock_base(self):self.hold_base(True)
    def stop_base(self):return self.base.stop()

    def observe(self,anchor,deadline):
        identity=self.template.get('identity',{})
        key=tuple(identity[k] for k in ('class_id','label','color')) if 'class_id' in identity else None
        snapshot=None;end=min(deadline,time.monotonic()+5.)
        while time.monotonic()<end:
            self.guard()
            with self.frame_lock:frames=list(self.frames)
            for frame in frames:
                if snapshot is None:
                    matching=[o for o in frame[3] if o.valid and o.color==identity.get('color','yellow')]
                    if frame[1]!=self.session or frame[0]<=self.after:continue
                    if key is None:
                        if len(matching)>1:raise Failure('NO_TARGET','ambiguous target during teaching')
                        if not matching:continue
                        o=matching[0];key=(o.class_id,o.label,o.color);self.capture_identity=dict(class_id=o.class_id,label=o.label,color=o.color)
                    snapshot=Snapshot(self.session,self.cfg['arm_frame'],key,self.after,anchor)
                point=snapshot.add(*frame,self.rospy.Time.now().to_sec())
                if point is not None:
                    self.last_snapshot_stamps=[s for s,_ in snapshot.samples]
                    return point
            time.sleep(.02)
        raise Failure('NO_TARGET','no fresh stable target after base stop')

    def pulse(self,axis,speed,seconds):
        vector=[0.,0.];vector[axis]=speed;self.base.switch(1,tuple(vector));end=time.monotonic()+seconds
        try:
            while time.monotonic()<end:
                self.guard()
                with self.frame_lock:latest=self.last_frame
                if latest is None or time.monotonic()-latest[0]>.5:raise Failure('SENSOR_INVALID','vision lost during base pulse')
                frame=latest[1]
                age=self.rospy.Time.now().to_sec()-frame[0]
                if not -.05<=age<=.5 or frame[1]!=self.session or frame[2]!='ok' or len([o for o in frame[3] if o.valid and o.stamp==frame[0] and
                        o.frame==self.cfg['arm_frame'] and o.session==self.session and .25<=o.quality<=1 and
                        (o.class_id,o.label,o.color)==tuple(self.template['identity'][k] for k in ('class_id','label','color'))])!=1:
                    raise Failure('NO_TARGET','target lost/ambiguous during base pulse')
                time.sleep(.01)
        finally:self.hold_base()

    def execute(self,step):
        self.arm_started=True;self.base.guard()
        if self.base.fresh_status().phase!='LOCKED':raise Failure('NOT_READY','base not locked')
        actual=self.arm.measured();q=tuple(step['positions_rad'])
        if step['phase'] in ('open','close'):
            q=actual[:5]+(q[5],);self.fixed_planner.valid_joints(q)
            self.fixed_planner.valid_pose(self.fixed_planner.forward(q),self.fixed_planner.floor)
        else:
            q=q[:5]+(actual[5],);self.fixed_planner.edge(actual,q)
        self.arm.trajectory(q,self.guard,step['seconds'])
        if max(abs(a-b) for a,b in zip(self.arm.measured(),q))>.035:
            raise Failure('MOTION_FAILED','measured taught point not reached')
        return dict(measured_rad=list(self.arm.measured()),at=self.rospy.Time.now().to_sec())

    def hold_arm(self):
        try:return self.arm.hold(self.cfg['stop_timeout'])
        except Exception:return False

    def close(self):
        self.select(False);self.base.close()
