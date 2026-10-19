#!/usr/bin/env python3
"""Stage-C relative-body adapter. Idle until session acquisition; no Tag/map input."""
import json
import math
import os
from pathlib import Path
import threading
import time
from jzzj_grasp.fixed_base import MicroNavigator,FruitTranslation,validate_base
from jzzj_grasp.core import Observation
from jzzj_grasp.handoff import validate_envelope
from jzzj_grasp.execution_lease import ExecutionLease


class Navigation:
    def __init__(self,r):
        from geometry_msgs.msg import PoseStamped,Twist
        from user_msgs.msg import CarCmd
        from jzzj_grasp_interfaces.msg import TargetObservations,VisualBaseFeedback
        from jzzj_grasp_interfaces.srv import StageCControl,StageCControlResponse
        self.r=r;self.lock=threading.RLock();self.core=MicroNavigator();self.feedback=None;self.tracker=None
        self.cfg=validate_base(json.loads(Path(r.get_param('~config')).read_text()))
        self.reference=json.loads(Path(r.get_param('~reference')).read_text())
        validate_base(self.cfg,self.reference)
        self.lease=ExecutionLease('/tmp/jzzj_fixed_navigation.lock')
        self.state_path=Path(r.get_param('~state_path'));self.state={'session':'','used':[],'requests':{}}
        if self.state_path.exists():self.state=json.loads(self.state_path.read_text())
        self.session=self.state['session'];self.recovering=bool(self.session);self.fault=''
        r.set_param('~active_session',self.session)
        self.twist,self.cmd,self.response,self.feedback_type=Twist,CarCmd,StageCControlResponse,VisualBaseFeedback
        self.vel=None;self.mode=None;self.activated_at=0.;self.checked=-1.
        self.feedback_pub=r.Publisher(self.cfg['feedback_topic'],VisualBaseFeedback,queue_size=1)
        self.subs=[r.Subscriber('/stage_c/observations',TargetObservations,self.observation,queue_size=1),
                   r.Subscriber('/car_cmd',CarCmd,self.command,queue_size=1),
                   r.Subscriber('/car_target',PoseStamped,self.target,queue_size=1)]
        self.service=r.Service('/stage_c/base/control',StageCControl,self.control)
        r.set_param('~completed_seq',-1);r.set_param('~bounded_micro_navigation',True)
        r.on_shutdown(self.stop);self.timer=r.Timer(r.Duration(.05),self.tick)

    def save(self):
        self.state['session']=self.session;self.state_path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.state_path.with_name(self.state_path.name+'.tmp')
        with temporary.open('w') as f:json.dump(self.state,f);f.flush();os.fsync(f.fileno())
        os.replace(temporary,self.state_path)

    def outputs(self):
        if self.vel is None:
            self.vel=self.r.Publisher('/cmd_vel',self.twist,queue_size=1)
            self.mode=self.r.Publisher('/car_cmd',self.cmd,queue_size=1)

    def graph(self,active=True):
        import rosgraph
        pubs,subs,_=rosgraph.Master(self.r.get_name()).getSystemState();pubs,subs=dict(pubs),dict(subs)
        own=self.r.get_name();main='/stage_c_fixed_grasp'
        allowed={own} if active else set()
        if set(pubs.get('/cmd_vel',[]))-allowed or set(subs.get('/cmd_vel',[]))-{'/chassis'}:
            raise ValueError('another controller owns base velocity')
        if set(pubs.get('/car_cmd',[]))-{main,own} or set(pubs.get('/car_target',[]))-{main}:
            raise ValueError('another task owns base command topics')
        if set(subs.get('/car_target',[]))-{own} or set(subs.get('/car_cmd',[]))-{main,own,'/chassis'}:
            raise ValueError('legacy navigation subscribers must release shared interfaces')
        if pubs.get('/stage_c/base/status') or pubs.get('/stage_c/base/command'):
            raise ValueError('old base controller conflicts')

    def control(self,req):
        with self.lock:
            try:
                if getattr(req,'_connection_header',{}).get('callerid')!='/stage_c_fixed_grasp':
                    raise ValueError('only the stage-C owner may acquire/release this adapter')
                validate_envelope(req.session_id,int(req.request_id),req.issued_at.to_sec(),req.expires_at.to_sec(),
                                  self.r.Time.now().to_sec(),self.state['requests'].get(req.session_id,0))
                if req.operation==4:
                    if self.session or req.session_id in self.state['used']:raise ValueError('busy/recovery/replayed session')
                    if not req.confirmed_stopped:raise ValueError('upstream stop acknowledgement missing')
                    self.graph(active=False)
                    self.session=req.session_id;self.state['used'].append(self.session);self.save()
                    self.activated_at=self.r.Time.now().to_sec();self.core=MicroNavigator()
                    self.tracker=FruitTranslation(self.cfg,self.reference,self.session,self.activated_at)
                    self.feedback=None;self.fault='';self.outputs()
                elif req.operation==5:
                    if self.session!=req.session_id or not req.confirmed_stopped:
                        raise ValueError('matching session and explicit external stop confirmation required')
                    if self.core.goal is not None:raise ValueError('active goal cannot be handed off')
                    if self.recovering and not req.confirmed_empty_and_clear:
                        raise ValueError('restarted adapter requires stopped, empty and clear recovery')
                    self.core.stop('explicit release');self.outputs();self.publish((0.,0.),self.cmd.HOLD)
                    time.sleep(.1)
                    self.vel.unregister();self.mode.unregister();self.vel=self.mode=None
                    self.session='';self.tracker=None;self.feedback=None;self.recovering=False;self.fault=''
                else:raise ValueError('unsupported base control operation')
                self.state['requests'][req.session_id]=int(req.request_id);self.save()
                self.r.set_param('~active_session',self.session)
                return self.response(True,'OWNED' if self.session else 'RELEASED','OK','session processed')
            except Exception as exc:return self.response(False,'FAULT' if self.fault or self.recovering else 'IDLE','REJECTED',str(exc))

    def authorized(self,msg):
        return bool(self.session) and getattr(msg,'_connection_header',{}).get('callerid')=='/stage_c_fixed_grasp'

    def observation(self,msg):
        with self.lock:
            if self.tracker is None or self.fault:return
            try:
                targets=[Observation(o.header.stamp.to_sec(),o.session_id,o.header.frame_id,
                    (o.position.x,o.position.y,o.position.z),o.color,o.label,o.class_id,o.track_id,o.quality,o.valid)
                    for o in msg.targets]
                value=self.tracker.add(msg.header.stamp.to_sec(),msg.session_id,msg.diagnostic,targets,self.r.Time.now().to_sec())
                if value is None:return
                self.feedback=(value,time.monotonic());out=self.feedback_type()
                out.header.stamp=msg.header.stamp;out.header.frame_id='stage_c_visual_local'
                out.session_id=self.session;out.track_id=value['track_id'];out.x=value['x'];out.y=value['y']
                out.fruit_position_m=value['fruit_position_m'];out.valid=True;out.diagnostic=value['diagnostic']
                self.feedback_pub.publish(out)
            except Exception as exc:
                self.fault=str(exc);self.core.stop(self.fault);self.publish((0.,0.),self.cmd.HOLD)
                out=self.feedback_type();out.header.stamp=self.r.Time.now();out.header.frame_id='stage_c_visual_local'
                out.session_id=self.session;out.valid=False;out.diagnostic=self.fault;self.feedback_pub.publish(out)

    def current(self):
        if self.fault or self.feedback is None or time.monotonic()-self.feedback[1]>.5:
            raise ValueError(self.fault or 'fruit-relative feedback absent/stale')
        value=self.feedback[0]
        if not -.05<=self.r.Time.now().to_sec()-value['stamp']<=.5:raise ValueError('fruit feedback timestamp stale')
        return (value['x'],value['y']),value['stamp']

    def command(self,msg):
        with self.lock:
            if not self.authorized(msg):return
            if msg.cmd==self.cmd.HOLD:self.core.stop('HOLD');self.publish((0.,0.),self.cmd.HOLD)
            elif msg.cmd==self.cmd.NAVIGATION and not self.fault and not self.recovering:
                self.core.command(True,time.monotonic())
            else:self.core.stop('unexpected/unready mode');self.publish((0.,0.),self.cmd.HOLD)

    def target(self,msg):
        with self.lock:
            if not self.authorized(msg):return
            try:
                self.graph();stamp=msg.header.stamp.to_sec()
                p,q=msg.pose.position,msg.pose.orientation
                if (msg.header.frame_id!='stage_c_body' or stamp<=self.activated_at or
                        not all(math.isfinite(v) for v in (stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or
                        not -.05<=self.r.Time.now().to_sec()-stamp<=.3 or
                        any(abs(v)>1e-9 for v in (p.z,q.x,q.y,q.z)) or abs(q.w-1)>1e-9 or self.fault or self.recovering):
                    raise ValueError('fresh translation-only stage_c_body goal required; map goals are rejected')
                position,_=self.current();self.core.accept((p.x,p.y),position,msg.header.seq,time.monotonic())
            except Exception as exc:self.fault=str(exc);self.core.stop(self.fault);self.publish((0.,0.),self.cmd.HOLD)

    def publish(self,velocity,mode):
        if self.vel is None:return
        self.mode.publish(self.cmd(cmd=mode));out=self.twist();out.linear.x,out.linear.y=velocity;self.vel.publish(out)

    def tick(self,event=None):
        with self.lock:
            if not self.session:return
            try:
                if self.vel is None:self.graph(active=False);self.outputs()
                if time.monotonic()-self.checked>.2:self.graph();self.checked=time.monotonic()
                if self.core.goal is None:self.publish((0.,0.),self.cmd.HOLD);return
                position,stamp=self.current();seq=self.core.last_seq
                velocity,arrived=self.core.tick(position,stamp,self.r.Time.now().to_sec()-stamp,time.monotonic())
                if arrived:self.r.set_param('~completed_seq',seq)
                elif self.core.goal is None:self.fault=self.core.reason
                self.publish(velocity,self.cmd.ARRIVE if arrived else self.cmd.NAVIGATION if self.core.goal else self.cmd.HOLD)
            except Exception as exc:self.fault=str(exc);self.core.stop(self.fault);self.publish((0.,0.),self.cmd.HOLD)
            self.r.set_param('~diagnostic',self.fault or self.core.reason)

    def stop(self):
        with self.lock:
            self.core.stop('shutdown');self.publish((0.,0.),self.cmd.HOLD)


if __name__=='__main__':
    import rospy
    rospy.init_node('stage_c_fixed_navigation');node=Navigation(rospy);rospy.spin()
