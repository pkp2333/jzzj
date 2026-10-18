#!/usr/bin/env python3
import copy
import threading
import time
import rospy
import actionlib
from jzzj_grasp_interfaces.msg import (VisualGraspAction, VisualGraspResult, VisualGraspFeedback,
    StageCContext, TargetSelection, TargetObservations)
from jzzj_grasp.core import Engine, Store, Context, Observation, CODES, validate_config
from jzzj_grasp.backends import SimBackend, RosArmBackend
from jzzj_grasp.execution_lease import ExecutionLease

class Server:
    def __init__(self):
        self.cfg=rospy.get_param('~execution');self.kind=rospy.get_param('~backend','sim')
        if self.kind not in ('sim','real'):raise ValueError('backend must be sim or real')
        validate_config(self.cfg)
        self.store=Store();self.lock=threading.RLock();self.active=None;self.engine=None;self.worker=None
        self.poisoned=False;self.used_sessions=set();self.holding_latched=False
        self.execution_lease=None;self.lease_factory=ExecutionLease
        self.publish_latches()
        self.backend=SimBackend(self.store,self.cfg,lambda:rospy.Time.now().to_sec()) if self.kind=='sim' else RosArmBackend(self.cfg)
        self.selection=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
        self.mode=rospy.Publisher('/stage_c/backend',__import__('std_msgs.msg',fromlist=['String']).String,queue_size=1,latch=True)
        self.mode.publish(self.kind)
        rospy.Subscriber('/stage_c/context',StageCContext,self.context_cb,queue_size=1)
        if self.kind=='real':rospy.Subscriber('/stage_c/observations',TargetObservations,self.observation_cb,queue_size=2)
        else:rospy.Timer(rospy.Duration(.02),lambda _:self.backend.refresh())
        self.server=actionlib.ActionServer('/stage_c/grasp',VisualGraspAction,self.goal_cb,self.cancel_cb,auto_start=False)
        self.server.start();rospy.on_shutdown(self.shutdown)

    def context_cb(self,m):
        with self.store.lock:
            self.store.context=Context(m.header.stamp.to_sec(),m.session_id,m.base_locked,m.arm_control_granted,m.stop_requested,time.monotonic())

    def publish_latches(self):
        try:
            rospy.set_param('~stop_failed_latched',self.poisoned)
            rospy.set_param('~holding_latched',self.holding_latched)
        except Exception as exc:
            rospy.logwarn('cannot publish stage C latch diagnostics: %s',exc)

    def observation_cb(self,m):
        # Atomic frame replacement: an empty detection removes old targets.
        with self.store.lock:
            self.store.observations=[Observation(o.header.stamp.to_sec(),o.session_id,o.header.frame_id,
                (o.position.x,o.position.y,o.position.z),o.color,o.label,o.class_id,o.track_id,o.quality,o.valid,(o.roi.x_offset,o.roi.y_offset,o.roi.width,o.roi.height))
                for o in m.targets if o.session_id==m.session_id and o.header.stamp==m.header.stamp]
            self.store.diagnostic=m.diagnostic

    @staticmethod
    def result(o):
        return VisualGraspResult(code=CODES[o.code],message=o.message,grasp_confirmed=o.grasp_confirmed,
            holding_known=o.holding_known,holding_object=o.holding_object,
            ready_for_transport=o.ready_for_transport,stop_confirmed=o.stop_confirmed)

    def reject(self,gh,text):
        gh.set_rejected(VisualGraspResult(code=CODES['NOT_READY'],message=text),text)

    def goal_cb(self,gh):
        from jzzj_grasp.core import Failure
        with self.lock:
            if rospy.get_param('/stage_c/taught/recovery_required',False):
                self.reject(gh,'taught session requires operator stop/hand recovery');return
            if self.active is not None or self.poisoned or getattr(self,'holding_latched',False):
                self.reject(gh,'busy, holding/unknown object or STOP_FAILED; clear handoff and operator restart required');return
            g=gh.get_goal();cfg=copy.deepcopy(self.cfg)
            if g.session_id in self.used_sessions:
                self.reject(gh,'session already used; upstream must create a new handoff session');return
            if g.grasp_profile not in cfg['profiles'] or g.target_color not in cfg['supported_colors'] or not g.session_id or not 0<g.timeout.to_sec()<=cfg['max_goal_seconds']:
                self.reject(gh,'invalid session/colour/profile/timeout');return
            cfg['active_profile']=g.grasp_profile
            self.backend.cfg=cfg
            e=Engine(self.backend,self.store,cfg,now=lambda:rospy.Time.now().to_sec(),feedback=lambda *a:self.feedback(gh,*a))
            e.session=g.session_id;e.deadline=time.monotonic()+g.timeout.to_sec()
            try:
                self.execution_lease=self.lease_factory()
                e.guard();self.backend.ready(e.guard)
            except Exception as exc:
                if self.execution_lease:self.execution_lease.close();self.execution_lease=None
                self.reject(gh,str(exc));return
            self.active=gh;self.engine=e
            self.used_sessions.add(g.session_id)
            gh.set_accepted('stage C accepted')
            self.backend.select(g.session_id,g.target_color)
            self.selection.publish(TargetSelection(session_id=g.session_id,target_color=g.target_color,enabled=True))
            self.worker=threading.Thread(target=self.run,args=(gh,e,g),daemon=True);self.worker.start()

    def run(self,gh,e,g):
        try:
            o=e.execute(g.session_id,g.target_color,g.grasp_profile,g.timeout.to_sec())
            with self.lock:
                if o.code=='STOP_FAILED':self.poisoned=True
                if o.holding_object or e.closed:self.holding_latched=True
            self.publish_latches()
            # Do not acquire ActionServer's lock while holding our callback lock.
            self.selection.publish(TargetSelection(session_id=g.session_id,target_color=g.target_color,enabled=False))
            if o.code=='SUCCESS':gh.set_succeeded(self.result(o))
            elif o.code=='CANCELED':gh.set_canceled(self.result(o))
            else:gh.set_aborted(self.result(o))
            with self.lock:self.active=self.engine=None
            if not self.poisoned and not self.holding_latched and self.execution_lease:
                self.execution_lease.close();self.execution_lease=None
        except Exception as exc:
            rospy.logerr('stage C worker failed: %s',exc)
            with self.lock:self.poisoned=True
            self.publish_latches()
            gh.set_aborted(VisualGraspResult(code=CODES['STOP_FAILED'],message=str(exc)))
            with self.lock:self.active=self.engine=None

    def feedback(self,gh,step,target,error,diagnostic):
        f=VisualGraspFeedback(step=step,position_error=error,diagnostic=diagnostic)
        f.target.header.stamp=rospy.Time.now();f.target.header.frame_id=self.cfg['arm_frame']
        if target:f.target.point.x,f.target.point.y,f.target.point.z=target
        gh.publish_feedback(f)

    def cancel_cb(self,gh):
        with self.lock:
            if self.active is not None and gh.get_goal_id().id==self.active.get_goal_id().id:self.engine.cancel.set()
            else:gh.set_canceled(VisualGraspResult(code=CODES['CANCELED'],message='not active'))

    def shutdown(self):
        with self.lock:
            if self.engine:self.engine.cancel.set()
        if self.worker:self.worker.join(self.cfg['stop_timeout']+1.)

if __name__=='__main__':
    rospy.init_node('stage_c_server');Server();rospy.spin()
