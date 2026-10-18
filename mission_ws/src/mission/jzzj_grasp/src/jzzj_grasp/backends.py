"""Simulation and ROS adapters. Neither backend sends base commands."""
import copy
import math
import time
from .core import Failure, Observation, distance, finite_vector

class SimBackend:
    def __init__(self,store,cfg,now=time.time):
        self.store,self.cfg,self.now=store,cfg,now
        self.position=tuple(cfg['sim_initial_tcp']);self.object_position=tuple(cfg['sim_object'])
        self.closed=False;self.attached=False;self.offset=(0.,0.,0.);self.color='red';self.session=''
        self.trace=[];self.drop=False;self.reject_motion=False

    def selected(self,o):pass

    def select(self,session,color):
        self.session,self.color=session,color

    def ready(self,guard):guard()

    def preflight(self,surface,profile,guard,remaining=False):guard()

    def prepare(self,surface,profile,guard,feedback):guard()

    def tcp(self,guard):guard();return self.position

    def refresh(self):
        if not self.session:return
        with self.store.lock:
            self.store.observations=[Observation(self.now(),self.session,self.cfg['arm_frame'],self.object_position,self.color)]
            self.store.diagnostic='ok'

    def move(self,point,orientation,guard):
        guard()
        if self.reject_motion:raise Failure('MOTION_FAILED','simulated actuator failure')
        self.position=tuple(point);self.trace.append(('move',self.position))
        if self.attached and not self.drop:
            self.object_position=tuple(a+b for a,b in zip(self.position,self.offset))

    def grip(self,value,guard):
        guard();self.trace.append(('grip',value))
        p=self.cfg['profiles'][self.cfg.get('active_profile','tabletop')]
        self.closed=abs(value-p['closed_rad'])<1e-6
        if self.closed:
            expected=tuple(a+b for a,b in zip(self.object_position,p['surface_to_grasp']))
            self.attached=distance(expected,self.position)<=self.cfg['position_tolerance']*2
            self.offset=tuple(a-b for a,b in zip(self.object_position,self.position))
        else:self.attached=False

    def hold(self,timeout):self.trace.append(('hold',));return True

class RosArmBackend:
    def __init__(self,cfg):
        import rospy, actionlib
        from control_msgs.msg import FollowJointTrajectoryAction
        from sensor_msgs.msg import JointState
        from jzzj_grasp_interfaces.srv import ComputeArmMotion
        self.rospy,self.cfg=rospy,cfg;self.joints=None;self.received=0.;self.gripper=None;self.sent=False;self.ik_poisoned=False
        self.client=actionlib.SimpleActionClient('/arm_left/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.compute=rospy.ServiceProxy('/stage_c/compute_arm_motion',ComputeArmMotion)
        from jzzj_grasp_interfaces.msg import TargetObservation
        self.observation_pub=rospy.Publisher('/stage_c/observation',TargetObservation,queue_size=1)
        self.sub=rospy.Subscriber('/arm_left/joint_states',JointState,self.joint_cb,queue_size=1)

    def select(self,session,color):pass

    def selected(self,o):
        from jzzj_grasp_interfaces.msg import TargetObservation
        m=TargetObservation();m.header.stamp=self.rospy.Time.from_sec(o.stamp);m.header.frame_id=o.frame
        m.session_id=o.session;m.track_id=o.track_id;m.class_id=o.class_id;m.label=o.label;m.color=o.color
        m.position.x,m.position.y,m.position.z=o.position;m.quality=o.quality;m.valid=o.valid
        m.roi.x_offset,m.roi.y_offset,m.roi.width,m.roi.height=o.roi
        self.observation_pub.publish(m)

    def joint_cb(self,msg):
        if (len(msg.name)!=6 or len(set(msg.name))!=6 or set(msg.name)!=set(self.cfg['joint_names']) or
                not finite_vector(msg.position,6)):
            return
        self.joints=msg;self.received=time.monotonic()

    def fresh_joints(self):
        m=self.joints
        if m is None or not 0< m.header.stamp.to_sec() or not -.05<=(self.rospy.Time.now()-m.header.stamp).to_sec()<=self.cfg['joint_timeout'] or time.monotonic()-self.received>self.cfg['joint_timeout']:
            raise Failure('MOTION_FAILED','missing/stale joint feedback')
        return m

    def ready(self,guard):
        guard()
        required=('real_motion_enabled','robot_calibration_confirmed','workspace_clear_confirmed','model_validated')
        if not all(self.cfg.get(k,False) for k in required):
            raise Failure('NOT_READY','explicit model/calibration/workspace/real-motion gates not confirmed')
        p=self.cfg['profiles'][self.cfg['active_profile']]
        if not p.get('calibrated',False):raise Failure('NOT_READY','grasp profile not calibrated')
        if not self.client.wait_for_server(self.rospy.Duration(1.)):
            raise Failure('NOT_READY','left arm trajectory server absent')
        try:self.rospy.wait_for_service('/stage_c/compute_arm_motion',timeout=1.)
        except self.rospy.ROSException:raise Failure('NOT_READY','kinematics service absent')
        self.fresh_joints();guard()

    def solve(self,point,orientation,guard,forward=False,seed=None):
        from jzzj_grasp_interfaces.srv import ComputeArmMotionRequest
        if self.ik_poisoned:raise Failure('MOTION_FAILED','IK worker unresponsive; restart required')
        guard();req=ComputeArmMotionRequest();req.forward_only=forward
        req.seed=self.fresh_joints() if seed is None else seed
        req.tcp_target.header.frame_id=self.cfg['arm_frame'];req.tcp_target.header.stamp=self.rospy.Time.now()
        if point is not None:
            req.tcp_target.pose.position.x,req.tcp_target.pose.position.y,req.tcp_target.pose.position.z=point
            (req.tcp_target.pose.orientation.x,req.tcp_target.pose.orientation.y,
             req.tcp_target.pose.orientation.z,req.tcp_target.pose.orientation.w)=orientation
        # A hung service must not prevent cancellation/context watchdogs in the execution thread.
        import threading
        done=threading.Event();box={}
        def call():
            try:box['value']=self.compute(req)
            except Exception as exc:box['error']=exc
            finally:done.set()
        threading.Thread(target=call,daemon=True).start()
        end=time.monotonic()+self.cfg['ik_timeout']
        while not done.wait(.02):
            try:guard()
            except Failure:
                self.ik_poisoned=True
                raise
            if time.monotonic()>end:
                self.ik_poisoned=True
                raise Failure('MOTION_FAILED','IK service timeout')
        guard()
        if 'error' in box:raise Failure('MOTION_FAILED',str(box['error']))
        reply=box['value']
        if not reply.valid:raise Failure('NEEDS_REPOSITION',reply.reason)
        return reply

    def tcp(self,guard):
        p=self.solve(None,None,guard,True).tcp_pose.pose.position
        return (p.x,p.y,p.z)

    def measured(self):
        m=self.fresh_joints();values=dict(zip(m.name,m.position))
        return tuple(values[n] for n in self.cfg['joint_names'])

    def planner(self,guard):
        from .planning import PathPlanner, Pose
        original_guard=guard
        def guard():
            original_guard();self.fresh_joints()
        template=copy.deepcopy(self.fresh_joints())
        def seed(q):
            m=copy.deepcopy(template);m.name=list(self.cfg['joint_names']);m.position=list(q)
            return m
        def forward(q):
            r=self.solve(None,None,guard,True,seed(q));p=r.tcp_pose.pose
            return Pose((p.position.x,p.position.y,p.position.z),
                        (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
        def inverse(point,orientation,q):
            r=self.solve(point,orientation,guard,seed=seed(q))
            if len(r.solution.name)!=6 or set(r.solution.name)!=set(self.cfg['joint_names']) or not finite_vector(r.solution.position,6):
                raise Failure('MOTION_FAILED','IK joint mapping mismatch')
            values=dict(zip(r.solution.name,r.solution.position))
            return tuple(values[n] for n in self.cfg['joint_names'])
        try:
            lower=self.rospy.get_param('/arm_left/arm_trajectory/joint_lower_rad')
            upper=self.rospy.get_param('/arm_left/arm_trajectory/joint_upper_rad')
        except KeyError as exc:
            raise Failure('NOT_READY','canonical /arm_left/arm_trajectory limits absent; use tools/stage_c/start_control.sh real') from exc
        return PathPlanner(self.cfg,lower,upper,forward,inverse,guard)

    def preflight(self,surface,profile,guard,remaining=False):
        # Full route validation issues only FK/IK requests, never trajectory goals.
        self.planner(guard).plan(self.measured(),surface,remaining)

    def execute_plan(self,planner,points,start,guard,feedback):
        expected=tuple(start)
        for point in points:
            guard();actual=self.measured()
            if max(abs(a-b) for a,b in zip(actual[:5],expected[:5]))>.035:
                raise Failure('MOTION_FAILED','arm deviated from preflight path')
            target=tuple(point.joints[:5])+(actual[5],)
            # Recheck the swept path from measured (quantized) feedback, not only
            # the ideal predecessor. Never replay a stale path blindly.
            planner.edge(actual,target,point.floor)
            feedback(point.step,planner.forward(target).position)
            self.trajectory(target,guard);expected=target

    def prepare(self,surface,profile,guard,feedback):
        planner=self.planner(guard);start=self.measured()
        points,_=planner.plan(start,surface)
        self.execute_plan(planner,points,start,guard,feedback)

    def trajectory(self,positions,guard):
        from control_msgs.msg import FollowJointTrajectoryGoal
        from trajectory_msgs.msg import JointTrajectoryPoint
        guard()
        if not finite_vector(positions,6):raise Failure('MOTION_FAILED','invalid joint solution')
        g=FollowJointTrajectoryGoal();g.trajectory.joint_names=self.cfg['joint_names']
        p=JointTrajectoryPoint();p.positions=positions;p.time_from_start=self.rospy.Duration(self.cfg['motion_seconds'])
        g.trajectory.points=[p];g.goal_time_tolerance=self.rospy.Duration(self.cfg['arrival_timeout'])
        self.client.send_goal(g)
        self.sent=True
        end=time.monotonic()+self.cfg['motion_seconds']+self.cfg['arrival_timeout']+1.
        while self.client.get_state() in (0,1,6,7):
            guard()
            if time.monotonic()>end:raise Failure('MOTION_FAILED','trajectory result timeout')
            time.sleep(.02)
        result=self.client.get_result()
        if self.client.get_state()!=3 or result is None or result.error_code!=0:
            raise Failure('MOTION_FAILED',result.error_string if result else 'trajectory failed')
        guard()
        self.fresh_joints()

    def move(self,point,orientation,guard):
        planner=self.planner(guard);start=self.measured()
        points,_=planner.segment(start,point,orientation,'VISUAL_CORRECTION')
        self.execute_plan(planner,points,start,guard,lambda *a:None)

    def grip(self,value,guard):
        m=self.fresh_joints();values=dict(zip(m.name,m.position))
        values[self.cfg['joint_names'][5]]=value
        self.trajectory([values[n] for n in self.cfg['joint_names']],guard)
        # Driver acceptance is not joint truth; confirm the measured claw position.
        m=self.fresh_joints()
        actual=dict(zip(m.name,m.position))[self.cfg['joint_names'][5]]
        if not math.isfinite(actual) or abs(actual-value)>self.cfg.get('gripper_closed_tolerance_rad',.05):
            raise Failure('MOTION_FAILED','measured gripper position deviates from command')


    def hold(self,timeout):
        from control_msgs.msg import FollowJointTrajectoryGoal
        from trajectory_msgs.msg import JointTrajectoryPoint
        def finish(ok,diagnostic):
            self.stop_diagnostic=diagnostic
            return ok
        end=time.monotonic()+timeout
        if self.sent:
            state=self.client.get_state()
            result=self.client.get_result()
            if result and 'STOP_FAILED' in result.error_string:return finish(False,result.error_string)
            if state in (0,1,6,7):
                self.client.cancel_goal()
                while time.monotonic()<end and self.client.get_state() in (0,1,6,7):
                    time.sleep(.02)
                state=self.client.get_state();result=self.client.get_result()
                if state==2 and result is not None and result.error_code==0:
                    return finish(True,'measured cancellation hold confirmed')
                if state in (0,1,6,7) or (result and 'STOP_FAILED' in result.error_string):
                    return finish(False,result.error_string if result else 'cancellation stop deadline exceeded')
        try:
            # Even an idle client must obtain explicit measured hold confirmation.
            m=self.fresh_joints();values=dict(zip(m.name,m.position))
            goal=FollowJointTrajectoryGoal();goal.trajectory.joint_names=self.cfg['joint_names']
            p=JointTrajectoryPoint();p.positions=[values[n] for n in self.cfg['joint_names']]
            p.time_from_start=self.rospy.Duration(.25);goal.trajectory.points=[p]
            # Four distinct scans are required (reference + three stable changes).
            # The SDK takes ~0.645s/scan; the driver's implicit 1s extra is too short.
            goal.goal_time_tolerance=self.rospy.Duration(self.cfg['arrival_timeout'])
            # Terminal state can arrive just before the driver's worker releases its busy flag.
            time.sleep(.05)
            if time.monotonic()+.25+self.cfg['arrival_timeout']>=end:
                return finish(False,'insufficient remaining hold confirmation budget')
            self.client.send_goal(goal);self.sent=True
            while time.monotonic()<end:
                if self.client.get_state() not in (0,1,6,7):
                    result=self.client.get_result()
                    ok=self.client.get_state()==3 and result is not None and result.error_code==0
                    return finish(ok,'measured hold confirmed' if ok else
                                  ('hold state='+str(self.client.get_state())+'; '+
                                   (result.error_string if result else 'missing result')))
                time.sleep(.02)
        except Exception as exc:
            return finish(False,str(exc))
        return finish(False,'hold result deadline exceeded')
