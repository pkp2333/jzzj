"""Offline regressions against actual navigation callbacks and RGB-D projection."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'mission_ws/src/mission/jzzj_visual_servo/src'))
from jzzj_visual_servo.perception import Candidate, DetectorAdapter, project_depth_to_rgb

class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'mission_ws/src/mission/jzzj_grasp/params/stage_c_fruit6.yaml').read_text())['perception']
        self.k=np.array([[60.,0,30.],[0,65.,20.],[0,0,1.]])
        self.flip=np.array([[1.,0,0],[0,-1,47],[0,0,1.]])

    def test_registered_flip_preserves_physical_position(self):
        rgb=np.zeros((48,64,3),np.uint8);rgb[5:25,10:30,2]=255
        depth=np.full((48,64),.4)
        c=Candidate(1,0,'cherry',.9,(10,5,30,25))
        adapter=DetectorAdapter(self.cfg)
        before=adapter.extract(rgb,depth,self.k,[c],'red')
        after=adapter.extract(rgb[::-1],depth[::-1],self.flip@self.k,
            [Candidate(1,0,'cherry',.9,(10,48-25,30,48-5))],'red')
        self.assertEqual(len(before),1);self.assertEqual(len(after),1)
        np.testing.assert_allclose(before[0][1],after[0][1],atol=1e-12)

    def test_unregistered_flip_preserves_registration(self):
        depth=np.zeros((48,64));depth[5:25,10:30]=400
        transform=np.eye(4);transform[0,3]=.01;transform[1,3]=-.005
        before=project_depth_to_rgb(depth,self.k,self.k,transform,(48,64),.001)
        after=project_depth_to_rgb(depth[::-1],self.flip@self.k,self.flip@self.k,transform,(48,64),.001)
        np.testing.assert_allclose(before[::-1],after,atol=1e-12)

    def test_zero_focal_length_still_rejected(self):
        k=self.k.copy();k[1,1]=0
        with self.assertRaises(ValueError):
            project_depth_to_rgb(np.ones((48,64)),k,k,np.eye(4),(48,64),1.)

class Stamp:
    def __init__(self,seconds=0.): self.seconds=seconds
    def to_sec(self): return self.seconds

class Twist:
    def __init__(self):
        self.linear=types.SimpleNamespace(x=0.,y=0.,z=0.)
        self.angular=types.SimpleNamespace(x=0.,y=0.,z=0.)

class Pose:
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=Stamp(),seq=0)
        self.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
             orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))

class CarCmd:
    HOLD=0;NAVIGATION=1;ARRIVE=2
    def __init__(self): self.cmd=self.HOLD

class Publisher:
    def __init__(self,*args,**kwargs): self.sent=[]
    def publish(self,msg): self.sent.append(copy.deepcopy(msg))

class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.now=[100.];cls.wall=[100.]
        ros=types.ModuleType('rospy');ros.get_param=lambda name,default:default
        ros.Time=types.SimpleNamespace(now=lambda:Stamp(cls.now[0]))
        ros.Publisher=Publisher
        for name in ('Subscriber','Timer','Duration','on_shutdown','loginfo','logwarn'):
            setattr(ros,name,lambda *a,**kw:None)
        cv=types.ModuleType('cv2')
        cv.putText=cv.imshow=cv.destroyAllWindows=lambda *a:None;cv.waitKey=lambda *a:255
        std=types.ModuleType('std_msgs.msg');std.Bool=object
        user=types.ModuleType('user_msgs.msg');user.CarCmd=CarCmd;user.PID=object
        geometry=types.ModuleType('geometry_msgs.msg');geometry.Twist=Twist;geometry.PoseStamped=Pose
        # Navigation tests use identity orientations; SciPy is supplied on robot.
        transform=types.ModuleType('scipy.spatial.transform')
        transform.Rotation=types.SimpleNamespace(from_quat=lambda q:types.SimpleNamespace(as_euler=lambda *a,**kw:np.zeros(3)))
        sys.path.insert(0,str(ROOT/'mission_ws/src/mission/wp_nav_pid/scripts'))
        spec=importlib.util.spec_from_file_location('nav_under_test',ROOT/'mission_ws/src/mission/wp_nav_pid/scripts/wp_nav_pid_node.py')
        cls.mod=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'rospy':ros,'cv2':cv,'std_msgs.msg':std,'user_msgs.msg':user,'geometry_msgs.msg':geometry,'scipy.spatial.transform':transform}):
            spec.loader.exec_module(cls.mod)

    def setUp(self):
        self.now[0]=self.wall[0]=100.
        self.patcher=patch.object(self.mod.time,'monotonic',lambda:self.wall[0]);self.patcher.start();self.addCleanup(self.patcher.stop)
        with patch.object(self.mod.time,'sleep'):
            self.node=self.mod.WaypointNavigationPIDNode()

    def pose(self,stamp=None):
        pose=Pose();pose.header.stamp=Stamp(self.now[0] if stamp is None else stamp);return pose

    def assert_stopped(self):
        self.assertEqual(self.node.car_cmd_pub.sent[-1].cmd,CarCmd.HOLD)
        velocity=self.node.cmd_vel_pub.sent[-1]
        self.assertEqual(vars(velocity.linear),dict(x=0.,y=0.,z=0.))
        self.assertEqual(vars(velocity.angular),dict(x=0.,y=0.,z=0.))

    def begin_navigation(self):
        self.node.front_car_pose_cb(self.pose())
        target=self.pose();target.header.seq=1;target.pose.position.x=1.
        self.node.car_target_cb(target);self.node.kp_x=1.
        self.node.car_pid_vertical.update_param(p=1.,i=0.,d=0.,i_a=0.,o_t=.2)
        self.node.car_navigator()
        self.assertEqual(self.node.car_cmd_pub.sent[-1].cmd,CarCmd.NAVIGATION)

    def test_moving_then_detection_lost_stops_every_tick(self):
        self.begin_navigation()
        self.node.tag_det_cb_front(types.SimpleNamespace(data=False))
        self.node.car_navigator();self.assert_stopped()
        count=len(self.node.cmd_vel_pub.sent)
        self.node.car_navigator();self.assert_stopped()
        self.assertEqual(len(self.node.cmd_vel_pub.sent),count+1)

    def test_bool_true_cannot_keep_stale_pose_alive(self):
        self.begin_navigation();self.now[0]=self.wall[0]=100.6
        self.node.tag_det_cb_front(types.SimpleNamespace(data=True))
        self.node.car_navigator();self.assert_stopped()

    def test_duplicate_cannot_refresh_reception_timeout(self):
        self.node.front_car_pose_cb(self.pose());self.wall[0]=100.6
        self.node.front_car_pose_cb(self.pose())
        self.node.car_navigator();self.assert_stopped()

    def test_invalid_stamps_and_quaternion_do_not_enable_pose(self):
        for stamp in (0.,99.,101.,float('nan')):
            self.node.front_car_pose_cb(self.pose(stamp))
            self.assertFalse(self.node.get_car_pose_front)
        p=self.pose();p.pose.orientation.w=0.
        self.node.front_car_pose_cb(p);self.assertFalse(self.node.get_car_pose_front)

    def test_other_camera_fallback_and_recovery(self):
        self.begin_navigation()
        self.now[0]=self.wall[0]=100.4;self.node.back_car_pose_cb(self.pose())
        self.now[0]=self.wall[0]=100.6;self.node.car_navigator()
        self.assertEqual(self.node.car_cmd_pub.sent[-1].cmd,CarCmd.NAVIGATION)
        self.now[0]=self.wall[0]=101.;self.node.car_navigator();self.assert_stopped()
        self.assertEqual(self.node.arrive_cnt,0)
        self.now[0]=self.wall[0]=101.1;self.node.front_car_pose_cb(self.pose());self.node.car_navigator()
        self.assertEqual(self.node.car_cmd_pub.sent[-1].cmd,CarCmd.NAVIGATION)
        self.assertFalse(self.node.wait_for_acquire)

    def test_shutdown_and_disabled_navigation_stop(self):
        self.begin_navigation();self.node.task=False;self.node.car_navigator();self.assert_stopped()
        self.node.publish_stop();self.assert_stopped()

if __name__=='__main__': unittest.main()
