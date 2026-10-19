#!/usr/bin/env python3
"""Navigation layer for <=50mm goals, using the existing car_target/car_cmd API."""
import json
from pathlib import Path
import threading
import time
from jzzj_grasp.fixed_base import MicroNavigator,map_pose,validate_base
from jzzj_grasp.execution_lease import ExecutionLease


class Navigation:
    def __init__(self,rospy):
        from geometry_msgs.msg import PoseStamped,Twist
        from user_msgs.msg import CarCmd
        self.r=rospy;self.lock=threading.RLock();self.core=MicroNavigator();self.pose=None
        self.cfg=validate_base(json.loads(Path(rospy.get_param('~config')).read_text()))
        self.lease=ExecutionLease('/tmp/jzzj_fixed_navigation.lock')
        self.twist,self.cmd=Twist,CarCmd
        self.vel=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.mode=rospy.Publisher('/car_cmd',CarCmd,queue_size=1)
        rospy.set_param('~completed_seq',-1)
        rospy.set_param('~bounded_micro_navigation',True)
        self.subs=[rospy.Subscriber(self.cfg['pose_topic'],PoseStamped,self.position,queue_size=1),
                   rospy.Subscriber('/car_cmd',CarCmd,self.command,queue_size=1),
                   rospy.Subscriber('/car_target',PoseStamped,self.target,queue_size=1)]
        self.checked=-1.;self.last_reason='';rospy.on_shutdown(self.stop)
        self.timer=rospy.Timer(rospy.Duration(.05),self.tick)

    def authorized(self,msg):
        return getattr(msg,'_connection_header',{}).get('callerid')=='/stage_c_fixed_grasp'

    def position(self,msg):
        with self.lock:
            try:
                pose,stamp=map_pose(msg,self.r.Time.now().to_sec())
                if self.pose is None or stamp>self.pose[1]:self.pose=(pose,stamp,time.monotonic())
            except Exception:self.pose=None

    def current(self):
        if self.pose is None or time.monotonic()-self.pose[2]>.3:raise ValueError('localization absent/stale')
        pose,stamp,_=self.pose
        if not -.05<=self.r.Time.now().to_sec()-stamp<=.3:raise ValueError('localization clock stale')
        return pose,stamp

    def command(self,msg):
        with self.lock:
            if not self.authorized(msg):return
            if msg.cmd not in (self.cmd.NAVIGATION,self.cmd.HOLD):
                self.core.stop('unexpected external chassis mode');return
            self.core.command(msg.cmd==self.cmd.NAVIGATION,time.monotonic())
            if msg.cmd==self.cmd.HOLD:self.publish((0.,0.),self.cmd.HOLD)

    def target(self,msg):
        with self.lock:
            try:
                if not self.authorized(msg):raise ValueError('unowned car_target publisher')
                self.graph(force=True)
                target,_=map_pose(msg,self.r.Time.now().to_sec());pose,_=self.current()
                self.core.accept(target,pose,msg.header.seq,time.monotonic())
            except Exception as exc:
                self.core.stop(str(exc));self.publish((0.,0.),self.cmd.HOLD)

    def graph(self,force=False):
        if not force and time.monotonic()-self.checked<.2:return
        import rosgraph
        pubs,subs,_=rosgraph.Master(self.r.get_name()).getSystemState();pubs,subs=dict(pubs),dict(subs)
        own=self.r.get_name();allowed={own,'/stage_c_fixed_grasp'}
        if pubs.get('/cmd_vel')!=[own] or subs.get('/cmd_vel')!=['/chassis']:
            raise ValueError('exclusive navigation -> chassis cmd_vel graph required')
        if pubs.get(self.cfg['pose_topic'])!=['/car_localization_node']:
            raise ValueError('canonical map localization publisher absent')
        if set(pubs.get('/car_cmd',[]))-allowed or set(pubs.get('/car_target',[]))-{'/stage_c_fixed_grasp'}:
            raise ValueError('another task owns navigation topics')
        if pubs.get('/stage_c/base/status') or pubs.get('/stage_c/base/command'):
            raise ValueError('legacy micro-base controller conflicts')
        self.checked=time.monotonic()

    def publish(self,velocity,mode):
        self.mode.publish(self.cmd(cmd=mode))
        msg=self.twist();msg.linear.x,msg.linear.y=velocity
        self.vel.publish(msg)

    def tick(self,event=None):
        with self.lock:
            try:
                self.graph()
                if self.core.goal is None:self.publish((0.,0.),self.cmd.HOLD);return
                pose,stamp=self.current();seq=self.core.last_seq
                velocity,arrived=self.core.tick(pose,stamp,self.r.Time.now().to_sec()-stamp,time.monotonic())
                if arrived:self.r.set_param('~completed_seq',seq)
                mode=self.cmd.ARRIVE if arrived else self.cmd.NAVIGATION if self.core.goal else self.cmd.HOLD
                self.publish(velocity,mode)
            except Exception as exc:
                self.core.stop(str(exc));self.publish((0.,0.),self.cmd.HOLD)
            if self.core.reason!=self.last_reason:
                self.last_reason=self.core.reason;self.r.set_param('~diagnostic',self.last_reason)
                self.r.loginfo('fixed navigation: %s',self.last_reason)

    def stop(self):
        with self.lock:
            self.core.stop('shutdown');self.publish((0.,0.),self.cmd.HOLD)


if __name__=='__main__':
    import rospy
    rospy.init_node('stage_c_fixed_navigation')
    node=Navigation(rospy)
    rospy.spin()
