"""ROS adapter for the fixed sequence; one trajectory writer, no terminal input."""
import math
import threading
import time
from collections import deque
from .backends import RosArmBackend
from .core import Failure,Observation,finite_vector
from .fixed_grasp import ReferenceGate,SIGNS,OFFSETS_DEG,calibration_digest


class FixedArm(RosArmBackend):
    def solve(self,*a,**kw):raise Failure('NOT_READY','IK is forbidden in fixed grasp')
    def planner(self,*a,**kw):raise Failure('NOT_READY','Cartesian/IK planning is forbidden in fixed grasp')


class FixedRos:
    def __init__(self,rospy,cfg,session,cancel):
        from jzzj_grasp_interfaces.msg import TargetObservations,TargetSelection
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        self.rospy,self.cfg,self.session,self.cancel=rospy,cfg,session,cancel
        self.backend_cfg=dict(joint_names=['joint'+str(i) for i in range(1,7)],joint_timeout=2.,
                              motion_seconds=3.,arrival_timeout=4.,stop_timeout=12.)
        self.arm=FixedArm(self.backend_cfg);self.frames=deque(maxlen=150);self.lock=threading.RLock()
        self.last_received=None;self.graph_checked=None;self.deadline=time.monotonic()+180.
        self.selection_type=TargetSelection
        self.pub=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
        self.sub=rospy.Subscriber('/stage_c/observations',TargetObservations,self.observation,queue_size=10)
        self.stop_sub=rospy.Subscriber('/stage_c/fixed_grasp/stop',Bool,
                                     lambda m:cancel.set() if m.data else None,queue_size=1)
        self.arm.joint_cb(rospy.wait_for_message('/arm_left/joint_states',JointState,timeout=5.))

    def observation(self,msg):
        targets=[Observation(o.header.stamp.to_sec(),o.session_id,o.header.frame_id,
                             (o.position.x,o.position.y,o.position.z),o.color,o.label,
                             o.class_id,o.track_id,o.quality,o.valid) for o in msg.targets]
        frame=(msg.header.stamp.to_sec(),msg.session_id,msg.diagnostic,targets)
        with self.lock:self.frames.append(frame);self.last_received=(time.monotonic(),frame)

    def check_graph(self,force=False):
        now=time.monotonic()
        if not force and self.graph_checked is not None and now-self.graph_checked<.2:return
        import rosgraph
        pubs,subs,_=rosgraph.Master(self.rospy.get_name()).getSystemState();pubs,subs=dict(pubs),dict(subs)
        if pubs.get('/arm_left/joint_states')!=['/arm_left/arm_trajectory']:
            raise Failure('NOT_READY','fixed grasp requires exactly one canonical trajectory driver')
        if pubs.get('/arm_left/arm_joint_states'):
            raise Failure('NOT_READY','exit legacy arm_test and other legacy command publishers first')
        for topic in ('/cmd_vel','/car_cmd','/stage_c/base/command'):
            if pubs.get(topic) or subs.get(topic):raise Failure('NOT_READY','base controller must be stopped: '+topic)
        if pubs.get('/stage_c/base/status'):raise Failure('NOT_READY','micro base driver must be stopped')
        if not getattr(self,'releasing',False) and len(pubs.get('/stage_c/observations',[]))!=1:
            raise Failure('NOT_READY','one observer required')
        self.graph_checked=now

    def guard(self):
        if self.cancel.is_set() or self.rospy.is_shutdown():raise Failure('CANCELED','operator/shutdown stop')
        if time.monotonic()>self.deadline:raise Failure('TIMEOUT','fixed sequence deadline')
        self.arm.fresh_joints();self.check_graph()

    def calibration(self):
        from sensor_msgs.msg import CameraInfo
        p=self.rospy.get_param('/stage_c_observer/perception')
        profile=self.rospy.get_param('/stage_c_observer/execution/profiles/tabletop')
        info=self.rospy.wait_for_message(p['rgb_info_topic'],CameraInfo,timeout=3.)
        return dict(perception=p,tabletop=profile,K=list(info.K),D=list(info.D),
                    image_size=[info.width,info.height],camera_frame=info.header.frame_id)

    def ready(self):
        for k in ('stop_failed_latched','holding_latched'):
            if self.rospy.get_param('/stage_c_server/'+k,False):raise Failure('NOT_READY','C server '+k)
        if self.rospy.get_param('/stage_c/taught/recovery_required',False):
            raise Failure('NOT_READY','previous session requires explicit release')
        self.check_graph(force=True)
        for k in ('hardware_enabled','calibration_confirmed','workspace_clear_confirmed'):
            if self.rospy.get_param('/arm_left/arm_trajectory/'+k,False) is not True:
                raise Failure('NOT_READY','trajectory driver gate: '+k)
        expected=dict(servo_signs=list(SIGNS),servo_offsets_deg=list(OFFSETS_DEG),
                      servo_position_min=96,servo_position_max=4000,servo_write_mode='individual')
        for k,v in expected.items():
            if self.rospy.get_param('/arm_left/arm_trajectory/'+k,None)!=v:
                raise Failure('NOT_READY','fixed legacy-angle mapping mismatch: '+k)
        if not self.arm.client.wait_for_server(self.rospy.Duration(2.)):
            raise Failure('NOT_READY','trajectory server absent')
        self.guard()
        if calibration_digest(self.calibration())!=self.cfg['calibration_sha256']:
            raise Failure('NOT_READY','camera/table calibration changed since reference capture')
        self.select(True)
        end=time.monotonic()+3.
        while time.monotonic()<end:
            self.guard()
            with self.lock:latest=self.last_received
            if latest is not None and time.monotonic()-latest[0]<=.5:
                stamp,session,diagnostic,_=latest[1]
                if session==self.session and diagnostic=='ok' and -.05<=self.rospy.Time.now().to_sec()-stamp<=.5:return
            time.sleep(.02)
        raise Failure('SENSOR_INVALID','fresh current-session observation stream absent')

    def select(self,enabled):
        self.pub.publish(self.selection_type(session_id=self.session,target_color=self.cfg['target_color'],enabled=enabled))

    def validate_all(self,poses):
        lower=self.rospy.get_param('/arm_left/arm_trajectory/joint_lower_rad')
        upper=self.rospy.get_param('/arm_left/arm_trajectory/joint_upper_rad')
        # These are computed by the driver from its checked raw range/mapping.
        # Avoid importing executable-package scripts through an incidental PYTHONPATH.
        if not finite_vector(lower,6) or not finite_vector(upper,6) or any(a>=b for a,b in zip(lower,upper)):
            raise Failure('NOT_READY','invalid driver-derived joint limits')
        for pose in poses:
            q=[math.radians(v) for v in pose]
            if not finite_vector(q,6) or any(not a<=v<=b for a,v,b in zip(lower,q,upper)):
                raise Failure('NOT_READY','fixed pose outside driver-derived 96..4000 servo limits')

    def move(self,degrees):
        q=[math.radians(v) for v in degrees]
        self.guard()
        self.rospy.set_param('/stage_c/taught/recovery_required',True)
        self.rospy.set_param('/stage_c/fixed_grasp/session',self.session)
        self.arm.trajectory(q,self.guard,seconds=3.)
        if max(abs(a-b) for a,b in zip(self.arm.measured(),q))>.035:
            raise Failure('MOTION_FAILED','fixed pose not reached in measured feedback')

    def wait_reference(self):
        gate=ReferenceGate(self.cfg,self.session,self.rospy.Time.now().to_sec());end=time.monotonic()+60.
        while time.monotonic()<end:
            self.guard()
            with self.lock:frames=list(self.frames);latest=self.last_received
            if latest is None or time.monotonic()-latest[0]>.5:raise Failure('SENSOR_INVALID','observation stream lost')
            for frame in frames:
                p=gate.add(*frame,self.rospy.Time.now().to_sec())
                if p is not None:return p
            time.sleep(.02)
        raise Failure('NO_TARGET','fruit did not enter 5mm reference region for 1 second within 60 seconds')

    def dwell(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.guard();time.sleep(.02)

    def hold(self):return self.arm.hold(self.backend_cfg['stop_timeout'])

    def release(self,session):
        from actionlib_msgs.msg import GoalStatusArray
        if not session or self.rospy.get_param('/stage_c/fixed_grasp/session','')!=session:
            raise Failure('NOT_READY','fixed grasp recovery session mismatch')
        for k in ('stop_failed_latched','holding_latched'):
            if self.rospy.get_param('/stage_c_server/'+k,False):raise Failure('NOT_READY','recover C server '+k+' first')
        self.releasing=True;self.check_graph(force=True);self.guard()
        status=self.rospy.wait_for_message('/arm_left/follow_joint_trajectory/status',GoalStatusArray,timeout=3.)
        if any(s.status in (0,1,6,7) for s in status.status_list):raise Failure('NOT_READY','trajectory still active')
        if not self.hold():raise Failure('STOP_FAILED','recovery measured hold failed')
        self.rospy.set_param('/stage_c/fixed_grasp/session','')
        self.rospy.set_param('/stage_c/taught/recovery_required',False)

    def close(self):self.select(False)
