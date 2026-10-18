#!/usr/bin/env python3
"""Opt-in, single-point hardware trajectories; mutually exclusive with legacy driver."""
import math
import threading
import time
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import (FollowJointTrajectoryAction,FollowJointTrajectoryResult,FollowJointTrajectoryFeedback)
from trajectory_safety import validate_mapping,validate_target,encode,decode
from servo_protocol import joint_limits, FEEDBACK_POSITION_MIN, FEEDBACK_POSITION_MAX

class TrajectoryDriver:
    def __init__(self):
        if rospy.get_namespace()!='/arm_left/':raise ValueError('stage C trajectory mode is left-arm only')
        if not all(rospy.get_param('~'+k,False) for k in ('hardware_enabled','calibration_confirmed','workspace_clear_confirmed')):
            raise ValueError('hardware/calibration/workspace gates must be explicitly confirmed')
        self.names=['joint'+str(i) for i in range(1,7)]
        self.signs=rospy.get_param('~servo_signs');self.offsets=rospy.get_param('~servo_offsets_deg')
        self.position_min=rospy.get_param('~servo_position_min',96)
        self.position_max=rospy.get_param('~servo_position_max',4000)
        self.lower,self.upper=joint_limits(self.signs,self.offsets,self.position_min,self.position_max,left=True)
        self.fb_lower,self.fb_upper=joint_limits(self.signs,self.offsets,
            FEEDBACK_POSITION_MIN,FEEDBACK_POSITION_MAX,left=True,feedback=True)
        validate_mapping(self.signs,self.offsets,self.lower,self.upper)
        # Runtime diagnostics for the planner; the six servo limits have one
        # source of truth in raw position units, not a second degree table.
        rospy.set_param('~joint_lower_rad',self.lower);rospy.set_param('~joint_upper_rad',self.upper)
        self.tolerance=rospy.get_param('~position_tolerance_rad',.035)
        self.freshness=rospy.get_param('~feedback_timeout',1.5)
        self.read_period=rospy.get_param('~feedback_period',.1)
        self.stop_timeout=rospy.get_param('~stop_timeout',3.)
        self.settle_delta=rospy.get_param('~settle_delta_rad',.01)
        if not all(math.isfinite(v) and v>0 for v in (self.tolerance,self.freshness,self.stop_timeout,self.settle_delta,self.read_period)):
            raise ValueError('positive finite tolerances/timeouts required')
        # Refuse startup if any old hardware-state writer is already running.
        import rosgraph
        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
        if any(topic=='/arm_left/joint_states' and owners for topic,owners in pubs):
            raise ValueError('another left-arm state publisher exists; stop legacy driver first')
        self.io=threading.RLock();self.lock=threading.RLock();self.current=None;self.received=0.;self.sequence=0
        self.active=None;self.cancel=threading.Event();self.poisoned=False;self.worker=None
        from checked_arm_sdk import CheckedArmDevice
        from hardware_lease import HardwareLease
        self.lease=HardwareLease('left')
        self.device=CheckedArmDevice(lr='left',position_min=self.position_min,position_max=self.position_max)
        self.read_duration=0.
        self.pub=rospy.Publisher('/arm_left/joint_states',JointState,queue_size=1)
        self.initialize_hardware()
        rospy.on_shutdown(self.shutdown)
        self.timer=rospy.Timer(rospy.Duration(self.read_period),lambda _:self.read())
        self.server=actionlib.ActionServer('/arm_left/follow_joint_trajectory',FollowJointTrajectoryAction,
            self.goal,self.cancel_goal,auto_start=False)
        self.server.start()

    def initialize_hardware(self):
        if not self.read():raise RuntimeError('cannot read all six servos before enabling torque')
        # The receive domain is wider than the command domain. Never enable the
        # whole arm if even one current pose cannot be sent as a holding target.
        q,_=self.sample()
        validate_target(self.names,q,self.names,self.lower,self.upper)
        report=self.write(q,.25)
        if report is None or not report.complete:
            raise RuntimeError('startup hold incomplete; torque not enabled')
        with self.io:self.device.Arm_serial_set_torque(1)
        # A bus write acknowledges transport, not physical torque or fresh UART
        # feedback. No Action server is exposed until a full new scan succeeds.
        if not self.read():
            raise RuntimeError('post-enable feedback failed; arm not ready, physical stop/torque state must be checked')
        self.sample()

    def read(self):
        try:
            with self.io:
                started=time.monotonic();stamp=rospy.Time.now()
                raw=[self.device.Arm_serial_servo_read(i+1) for i in range(6)]
            if any(v is None or not math.isfinite(v) for v in raw):
                raise ValueError('invalid SDK feedback')
            q=decode(raw,self.signs,self.offsets)
            validate_target(self.names,q,self.names,self.fb_lower,self.fb_upper)
            elapsed=time.monotonic()-started
            with self.lock:
                self.current=q;self.received=started;self.sequence+=1;self.read_duration=elapsed
            # Sequential servo reads are not an instantaneous sample. Timestamp
            # their beginning so stale first-joint data cannot look newly acquired.
            m=JointState();m.header.stamp=stamp;m.name=self.names;m.position=q;self.pub.publish(m)
            if elapsed>self.read_period:
                rospy.logwarn_throttle(5.,'six-servo read %.3fs exceeds requested period %.3fs',elapsed,self.read_period)
            return True
        except Exception as exc:
            rospy.logerr_throttle(1.,'arm feedback rejected: %s',exc)
            return False

    def sample(self):
        with self.lock:
            if self.current is None or time.monotonic()-self.received>self.freshness:
                raise RuntimeError('arm feedback stale')
            return list(self.current),self.sequence

    def write(self,q,seconds):
        validate_target(self.names,q,self.names,self.lower,self.upper,check_limits=False)
        raw=encode(q,self.signs,self.offsets)
        with self.io:
            return self.device.Arm_serial_servo_write6(*raw,int(seconds*1000))

    def goal(self,gh):
        try:
            with self.lock:
                if self.active is not None or self.poisoned:raise ValueError('busy or STOP_FAILED latched')
                g=gh.get_goal();t=g.trajectory
                if len(t.points)!=1:raise ValueError('only one bounded point per goal is supported')
                if t.header.stamp.to_sec()!=0:raise ValueError('scheduled trajectories are not supported')
                if g.path_tolerance:raise ValueError('path tolerances unsupported; use incremental goals')
                p=t.points[0]
                if p.velocities or p.accelerations or p.effort:raise ValueError('position-only trajectories required')
                q=validate_target(t.joint_names,p.positions,self.names,self.lower,self.upper,check_limits=False)
                seconds=p.time_from_start.to_sec()
                if not math.isfinite(seconds) or not .2<=seconds<=5.:raise ValueError('duration must be 0.2..5 seconds')
                tolerance=[self.tolerance]*6
                for tol in g.goal_tolerance:
                    if tol.name not in self.names or tol.velocity!=0 or tol.acceleration!=0 or not math.isfinite(tol.position) or tol.position<0:
                        raise ValueError('unsupported joint tolerance')
                    if tol.position>0:tolerance[self.names.index(tol.name)]=min(tol.position,self.tolerance)
                extra=g.goal_time_tolerance.to_sec()
                if not math.isfinite(extra) or not 0<=extra<=5.:raise ValueError('invalid goal_time_tolerance')
                self.sample();self.cancel.clear();self.active=gh;gh.set_accepted()
                self.worker=threading.Thread(target=self.execute,args=(gh,q,seconds,max(1.,extra),tolerance),daemon=True)
                self.worker.start()
        except Exception as exc:
            gh.set_rejected(FollowJointTrajectoryResult(error_code=-1,error_string=str(exc)))

    def cancel_goal(self,gh):
        with self.lock:
            if self.active is not None and gh.get_goal_id().id==self.active.get_goal_id().id:self.cancel.set()
            else:gh.set_canceled(FollowJointTrajectoryResult(error_code=0,error_string='not active'))

    def converged(self,target,end,tolerances,allow_cancel,indices=None):
        last=-1;stable=0;previous=None
        indices=range(6) if indices is None else indices
        while time.monotonic()<end:
            if allow_cancel and self.cancel.is_set():return False
            q,seq=self.sample()
            if seq!=last:
                quiet=previous is not None and all(abs(a-b)<=self.settle_delta for a,b in zip(q,previous))
                stable=stable+1 if quiet and all(abs(q[i]-target[i])<=tolerances[i] for i in indices) else 0
                previous=q;last=seq
                if stable>=3:return True
            time.sleep(.02)
        return False

    def hold(self):
        try:
            q,_=self.sample();report=self.write(q,.25)
            if report is not None and not report.complete:return False
            return self.converged(q,time.monotonic()+self.stop_timeout,[self.tolerance]*6,False)
        except Exception:return False


    def execute(self,gh,q,seconds,extra,tolerances):
        terminal='aborted'
        try:
            report=self.write(q,seconds)
            partial=report is not None and not report.complete
            if partial:
                ok=self.converged(q,time.monotonic()+seconds+extra,tolerances,True,
                                  tuple(i-1 for i in report.sent_ids))
            else:ok=self.converged(q,time.monotonic()+seconds+extra,tolerances,True)
            if self.cancel.is_set():
                stopped=self.hold();self.poisoned=not stopped
                terminal='canceled' if stopped else 'aborted'
                result=FollowJointTrajectoryResult(error_code=0 if stopped else -5,
                    error_string='measured hold confirmed' if stopped else 'STOP_FAILED')
            elif ok and partial:
                result=FollowJointTrajectoryResult(error_code=-5,
                    error_string='partial command: sent servos {}; skipped servos {} outside position limits; measured settled'.format(
                        report.sent_ids,report.skipped_ids))
            elif ok:
                m,_=self.sample();f=FollowJointTrajectoryFeedback();f.header.stamp=rospy.Time.now();f.joint_names=self.names
                f.desired.positions=q;f.actual.positions=m;f.error.positions=[a-b for a,b in zip(q,m)];gh.publish_feedback(f)
                terminal='succeeded';result=FollowJointTrajectoryResult(error_code=0)
            else:
                stopped=self.hold();self.poisoned=not stopped
                result=FollowJointTrajectoryResult(error_code=-5,error_string='arrival timeout; '+('hold confirmed' if stopped else 'STOP_FAILED'))
        except Exception as exc:
            stopped=self.hold();self.poisoned=not stopped
            result=FollowJointTrajectoryResult(error_code=-5,error_string=str(exc)+('; hold confirmed' if stopped else '; STOP_FAILED'))
        # All device work is finished. Release the hardware goal slot BEFORE publishing
        # terminal status so a client reacting immediately to completion is not spuriously busy.
        # Never hold our lock while acquiring actionlib's callback lock.
        with self.lock:self.active=None
        getattr(gh,'set_'+terminal)(result)

    def shutdown(self):
        self.cancel.set()
        if self.worker and self.worker.is_alive():self.worker.join(self.stop_timeout+1.)
        elif not self.hold():rospy.logerr('STOP_FAILED on driver shutdown; use physical emergency stop')

if __name__=='__main__':
    rospy.init_node('arm_trajectory');TrajectoryDriver();rospy.spin()
