"""Exercise the shared chassis driver and exclusive lease without serial hardware."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
from test_stage_c import ROOT

SCRIPTS=ROOT/'catkin_ws/src/actuator/chassis/scripts'
sys.path.insert(0,str(SCRIPTS))
from stage_c_base_safety import BaseLease


def lease():return BaseLease(True,True,True)


class LeaseTests(unittest.TestCase):
    def test_disabled_or_unverified_base_cannot_move(self):
        for gates in ((False,True,True),(True,False,True),(True,True,False),('false',True,True)):
            with self.assertRaises(ValueError):BaseLease(*gates).accept('s',1,0,0,0,1.,True)
    def test_lease_rejects_other_sessions_replays_diagonal_and_excess_speed(self):
        b=lease();b.accept('s',1,0,0,0,1.,True)
        for session,seq,op,vx,vy in [('other',2,1,.01,0),('s',1,1,.01,0),('s',2,1,.03,0),('s',2,1,.01,.01)]:
            with self.assertRaises(ValueError):b.accept(session,seq,op,vx,vy,1.1,True)
        self.assertEqual(b.owner,'s');self.assertEqual(b.velocity,(0,0))
    def test_deadman_latches_before_late_callback_and_never_replays_motion(self):
        b=lease();b.accept('s',1,0,0,0,0.,True);b.accept('s',2,1,.02,0,.1,True)
        with self.assertRaises(ValueError):b.accept('s',3,1,.02,0,.41,True)
        self.assertEqual(b.phase,'FAULT');self.assertEqual(b.tick(.5),(0,0))
        with self.assertRaises(ValueError):b.accept('s',4,0,0,0,.5,True)
    def test_lock_never_accepts_motion_and_release_is_explicit(self):
        b=lease();b.accept('s',1,0,0,0,1.,True);b.accept('s',2,3,0,0,1.1,True)
        with self.assertRaises(ValueError):b.accept('s',3,1,.01,0,1.2,True)
        with self.assertRaises(ValueError):b.accept('s',4,4,0,0,1.2,True)
        b.accept('s',5,4,0,0,1.2,True,True)
        with self.assertRaises(ValueError):b.accept('s',6,0,0,0,1.3,True)
        b.accept('new',1,0,0,0,5.,True)
        self.assertEqual(b.phase,'ALIGN')


class Message:
    def __init__(self,**kwargs):self.__dict__.update(kwargs)


class Serial:
    def __init__(self):self.writes=[]
    def write(self,data):self.writes.append(list(data));return len(data)


class DriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy=types.ModuleType('rospy');rospy.logwarn_throttle=lambda *a:None
        geom=types.ModuleType('geometry_msgs.msg');geom.Twist=Message
        std=types.ModuleType('std_msgs.msg');std.Bool=Message
        user=types.ModuleType('user_msgs.msg');user.CarCmd=Message(HOLD=0,ARRIVE=3,REMOTE=1,OpenGate=6,CloseGate=5)
        spec=importlib.util.spec_from_file_location('taught_chassis_driver',SCRIPTS/'chassis_control_node.py')
        cls.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'rospy':rospy,'serial':types.ModuleType('serial'),
                'std_msgs.msg':std,'geometry_msgs.msg':geom,'user_msgs.msg':user}):spec.loader.exec_module(cls.module)
    def driver(self):
        d=self.module.ChassisNode.__new__(self.module.ChassisNode);d.io_lock=threading.RLock()
        d.micro=lease();d.micro_stop_written=False;d.ser=Serial();d.task=True;d.delled=False
        d.head=[0xa5];d.tail=[0,0x5a];d.vel_mode=2;d.stop_data=[0xa5,2,0,0,0,0,0x5a]
        d.car_cmd=0;d.chassis_cmd_data=list(d.stop_data);d.last_chassis_cmd_data=list(d.stop_data)
        d.xy_dead_zone=0;d.xy_activate_zone=.06;d.max_vel_linear=.5
        d.z_dead_zone=d.z_activate_zone=0;d.max_vel_angular=1.;d.vel_2_0x40=64/.45;d.omega_2_0x40=64.
        return d
    def twist(self,x,y=0,z=0):return Message(linear=Message(x=x,y=y),angular=Message(z=z))
    def test_ordinary_mode_keeps_velocity_boost_and_hold_clears_old_cache(self):
        d=self.driver();d.car_cmd=1;d.cmd_vel_cb(self.twist(.02))
        self.assertEqual(d.chassis_cmd_data[3],(-int(.08*64/.45))&255)
        d.car_cmd=0;d.controller();self.assertEqual(d.chassis_cmd_data,d.stop_data)
        d.car_cmd=1;d.controller();self.assertTrue(all(v==d.stop_data for v in d.ser.writes))
    def test_micro_velocity_bypasses_boost_and_driver_stops_after_sender_disappears(self):
        d=self.driver();d.micro.accept('s',1,0,0,0,1.,True);d.micro.accept('s',2,1,.02,0,1.1,True)
        with patch.object(self.module.time,'monotonic',return_value=1.2):d.controller()
        self.assertEqual(d.ser.writes[-1][3],(-int(.02*64/.45))&255)
        with patch.object(self.module.time,'monotonic',return_value=1.5):d.controller()
        self.assertEqual(d.ser.writes[-1],d.stop_data);self.assertEqual(d.micro.phase,'FAULT')
        d.cmd_vel_cb(self.twist(.5));d.car_cmd_cb(Message(cmd=1))
        with patch.object(self.module.time,'monotonic',return_value=1.6):d.controller()
        self.assertEqual(d.ser.writes[-1],d.stop_data)
    def test_external_stop_always_overrides_micro_owner(self):
        d=self.driver();d.micro.accept('s',1,0,0,0,1.,True);d.micro.accept('s',2,1,.02,0,1.1,True)
        d.cmd_vel_cb(self.twist(0));self.assertEqual(d.micro.phase,'FAULT')
        self.assertEqual(d.ser.writes[-1],d.stop_data)
    def test_serial_failure_never_claims_stop_confirmation(self):
        d=self.driver();d.micro.accept('s',1,0,0,0,1.,True)
        d.ser.write=lambda data:0
        with patch.object(self.module.time,'monotonic',return_value=1.1):d.controller()
        self.assertFalse(d.micro_stop_written);self.assertEqual(d.micro.phase,'FAULT')


if __name__=='__main__':unittest.main()
