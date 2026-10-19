"""No-hardware regressions for calibrated car_target/car_cmd micro alignment."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
from test_stage_c import ROOT
from test_fixed_grasp import config
from jzzj_grasp.fixed_base import validate_base,correction,MicroNavigator,map_pose
from jzzj_grasp.core import Failure
import test_taught_base
import test_fixed_startup

sys.path.insert(0,str(ROOT/'tools/stage_c'))
spec=importlib.util.spec_from_file_location('base_calibration',ROOT/'tools/stage_c/calibrate_fixed_base.py')
calibration=importlib.util.module_from_spec(spec);spec.loader.exec_module(calibration)


def base_config():
    cfg=json.loads((ROOT/'deploy/stage_c_fixed_base.example.yaml').read_text())
    samples=[dict(base_delta_m=d,before_m=[0,0,-.05],after_m=[-d[0],-d[1],-.05])
             for d in ([.01,0],[-.01,0],[0,.01],[0,-.01])]
    cfg.update(calibration_confirmed=True,base_matrix=[[-1,0],[0,-1]],base_calibration_samples=samples,
               reference_calibration_sha256=config()['calibration_sha256'])
    return cfg


class MathTests(unittest.TestCase):
    def test_missing_calibration_rejected_and_calibrated_mapping_accepted(self):
        raw=json.loads((ROOT/'deploy/stage_c_fixed_base.example.yaml').read_text())
        with self.assertRaises(ValueError):validate_base(raw,config())
        validate_base(base_config(),config())
        for change in (dict(base_matrix=[[1,0],[0,1]]),dict(step_m=.051),dict(total_m=-1),
                       dict(calibration_confirmed=False),dict(reference_calibration_sha256='changed')):
            cfg=base_config();cfg.update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):validate_base(cfg,config())

    def test_correction_uses_measured_signs_and_current_yaw_not_assumed_map_axes(self):
        cfg=base_config();goal,body=correction(cfg,[.08,0,-.05],[0,0,-.05],(1,2,math.pi/2))
        np.testing.assert_allclose(body,[.05,0],atol=1e-10)
        np.testing.assert_allclose(goal,[1,2.05,math.pi/2],atol=1e-10)
        self.assertIsNone(correction(cfg,[.019,0,-.05],[0,0,-.05],(0,0,0)))
        self.assertIsNotNone(correction(cfg,[.021,0,-.05],[0,0,-.05],(0,0,0)))
        self.assertIsNotNone(correction(cfg,[.0198,0,-.046],[0,0,-.05],(0,0,0)))
        with self.assertRaises(Failure):correction(cfg,[0,0,-.04],[0,0,-.05],(0,0,0))

    def test_four_measured_pairs_fit_body_matrix_without_arm_frame_guess(self):
        records=[];reference=config()
        for delta in ([.01,0],[-.01,0],[0,.01],[0,-.01]):
            a=dict(reference_calibration_sha256=reference['calibration_sha256'],identity=reference['identity'],
                   arm_frame=reference['arm_frame'],arm_positions_rad=np.radians([-90,0,90,90,0,60]).tolist(),
                   map_pose=[0,0,math.pi/2],fruit_position_m=[0,0,-.05])
            b=copy.deepcopy(a);b['map_pose']=[-delta[1],delta[0],math.pi/2]
            b['fruit_position_m']=[-delta[0],-delta[1],-.05];records.append((a,b))
        fitted=calibration.samples_from_pairs(records,reference)
        from jzzj_grasp.taught import fit_base_matrix
        matrix,residual=fit_base_matrix(fitted,[0,0,1]);np.testing.assert_allclose(matrix,[[-1,0],[0,-1]],atol=1e-9)
        self.assertLess(residual,1e-9)
        records[0][1]['map_pose'][2]+=.1
        with self.assertRaises(ValueError):calibration.samples_from_pairs(records,reference)


class NavigationTests(unittest.TestCase):
    def armed(self):
        nav=MicroNavigator();nav.command(True,1.);nav.accept((.01,0,0),(0,0,0),1,1.01);return nav

    def test_goal_requires_arm_heartbeat_unique_sequence_and_50mm_bound(self):
        for goal in ((.051,0,0),(.04,.04,0),(.01,0,.02),(math.nan,0,0)):
            n=MicroNavigator();n.command(True,1.)
            with self.assertRaises(ValueError):n.accept(goal,(0,0,0),1,1.01)
        n=MicroNavigator()
        with self.assertRaises(ValueError):n.accept((.01,0,0),(0,0,0),1,1.)
        n=self.armed();n.stop('HOLD');n.command(True,1.1)
        with self.assertRaises(ValueError):n.accept((.01,0,0),(0,0,0),1,1.12)

    def test_speed_bound_and_map_to_body_rotation(self):
        n=self.armed();vel,done=n.tick((0,0,0),1.02,.02,1.05)
        self.assertLessEqual(math.hypot(*vel),.02);self.assertFalse(done);self.assertGreater(vel[0],0)
        n=MicroNavigator();n.command(True,1.);n.accept((0,.01,math.pi/2),(0,0,math.pi/2),1,1.01)
        vel,_=n.tick((0,0,math.pi/2),1.02,.02,1.05)
        self.assertGreater(vel[0],0);self.assertAlmostEqual(vel[1],0,places=8)

    def test_stale_pose_missing_heartbeat_and_overshoot_stop_without_replay(self):
        for pose,age,now in (((0,0,0),.4,1.1),((0,0,0),.01,1.5),((.056,0,0),.01,1.1),((0,0,.1),.01,1.1)):
            n=self.armed();vel,done=n.tick(pose,1.,age,now)
            self.assertEqual(vel,(0.,0.));self.assertFalse(done);self.assertIsNone(n.goal)
            n.command(True,now+.01);self.assertEqual(n.tick((0,0,0),1.,0,now+.02)[0],(0,0))

    def test_arrival_requires_four_distinct_measured_positions(self):
        n=self.armed()
        for _ in range(10):self.assertFalse(n.tick((.01,0,0),1.1,.01,1.11)[1])
        for stamp in (1.15,1.2):self.assertFalse(n.tick((.01,0,0),stamp,0,stamp)[1])
        self.assertTrue(n.tick((.01,0,0),1.25,0,1.25)[1]);self.assertIsNone(n.goal)

    def test_explicit_hold_cancels_navigation(self):
        n=self.armed();n.command(False,1.1)
        self.assertIsNone(n.goal);self.assertEqual(n.tick((0,0,0),1.1,0,1.1)[0],(0,0))

    def test_invalid_map_frame_stamp_or_quaternion_rejected(self):
        M=types.SimpleNamespace
        msg=M(header=M(frame_id='map',stamp=M(to_sec=lambda:1.)),pose=M(position=M(x=0,y=0,z=0),orientation=M(x=0,y=0,z=0,w=1)))
        map_pose(msg,1.1)
        msg.header.frame_id='odom'
        with self.assertRaises(Failure):map_pose(msg,1.1)
        msg.header.frame_id='map';msg.pose.orientation.w=0
        with self.assertRaises(Failure):map_pose(msg,1.1)


class IntegrationTests(unittest.TestCase):
    def test_navigation_layer_rejects_unowned_target_and_honors_owned_hold(self):
        spec=importlib.util.spec_from_file_location('fixed_navigation_node',ROOT/'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_navigation.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        n=module.Navigation.__new__(module.Navigation)
        import threading
        n.lock=threading.RLock();n.core=MicroNavigator();n.cmd=types.SimpleNamespace(HOLD=0,NAVIGATION=2)
        events=[];n.publish=lambda velocity,mode:events.append((velocity,mode))
        n.command(types.SimpleNamespace(cmd=2,_connection_header={'callerid':'/another_task'}))
        self.assertEqual(n.core.heartbeat,-math.inf)
        n.command(types.SimpleNamespace(cmd=2,_connection_header={'callerid':'/stage_c_fixed_grasp'}))
        self.assertTrue(math.isfinite(n.core.heartbeat))
        n.command(types.SimpleNamespace(cmd=0,_connection_header={'callerid':'/stage_c_fixed_grasp'}))
        self.assertEqual(events[-1],((0.,0.),0));self.assertIsNone(n.core.goal)
        n.target(types.SimpleNamespace(_connection_header={'callerid':'/another_task'}))
        self.assertIsNone(n.core.goal);self.assertEqual(events[-1],((0.,0.),0))

    def test_missing_measured_stop_blocks_arm_hold_success(self):
        from jzzj_grasp.fixed_grasp_ros import FixedRos
        io=FixedRos.__new__(FixedRos);events=[]
        def stop():events.append('base_HOLD');raise Failure('STOP_FAILED','pose stale')
        io.base=types.SimpleNamespace(stop=stop)
        io.arm=types.SimpleNamespace(hold=lambda timeout:events.append('arm_hold') or True)
        io.backend_cfg={'stop_timeout':12.}
        self.assertFalse(io.hold());self.assertEqual(events,['base_HOLD','arm_hold'])

    def test_stationary_pose_guard_rejects_base_drift_during_arm_motion(self):
        from jzzj_grasp.fixed_base_ros import NavigationAlignment
        alignment=NavigationAlignment.__new__(NavigationAlignment)
        alignment.held=True;alignment.anchor=(0,0,0)
        alignment.current=lambda:((.004,0,0),1.)
        emitted=[];alignment.cmd=lambda **kw:kw
        alignment.cmd.HOLD=0;alignment.mode=types.SimpleNamespace(publish=emitted.append)
        with self.assertRaises(Failure):alignment.guard()
        self.assertEqual(emitted,[{'cmd':0}])

    def test_alignment_exceeds_old_100mm_and_15step_budgets_when_unlimited(self):
        from jzzj_grasp.fixed_base_ros import NavigationAlignment
        cfg=base_config();cfg['step_m']=.01
        alignment=NavigationAlignment.__new__(NavigationAlignment)
        alignment.cfg=cfg;alignment.trace=[];alignment.pose_xy=[0.,0.];alignment.steps=0
        alignment.io=types.SimpleNamespace(arm=types.SimpleNamespace(measured=lambda:[0.]*6),
            cfg={'reference_position_m':[0.,0.,-.05]},guard=lambda:None,deadline=0.)
        alignment.stop=lambda:None
        alignment.current=lambda:((alignment.pose_xy[0],alignment.pose_xy[1],0.),1.)
        alignment.observe=lambda *a:(.3-alignment.pose_xy[0],0.,-.05)
        def navigate(target):
            alignment.pose_xy[:]=target[:2];alignment.steps+=1
        alignment.navigate=navigate
        point=alignment.align()
        self.assertLessEqual(math.hypot(*point[:2]),.02)
        self.assertGreater(alignment.steps,15);self.assertGreater(alignment.pose_xy[0],.1)
        self.assertFalse(alignment.aligning)

    def test_scheduler_never_publishes_cmd_vel(self):
        import ast
        tree=ast.parse((ROOT/'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_base_ros.py').read_text())
        topics=[n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                and n.func.attr=='Publisher' and n.args and isinstance(n.args[0],ast.Constant)]
        self.assertEqual(set(topics),{'/car_target','/car_cmd'})

    def test_new_mode_refuses_unbounded_existing_navigation_and_chassis(self):
        startup=test_fixed_startup.startup;graph=test_fixed_startup.graph
        flags=startup.launch_flags(graph(),base_adjust=True)
        self.assertTrue(flags['start_localization'])
        flags=startup.launch_flags(graph(pubs=[('/front_car_pose_orig_map',['/car_localization_node'])]),True)
        self.assertFalse(flags['start_localization'])
        for state in (graph(['/chassis']),graph(['/wp_nav_pid_node']),graph(subs=[('/car_target',['/other'])])):
            with self.assertRaises(RuntimeError):startup.launch_flags(state,True)

    def test_uncalibrated_one_click_refuses_before_any_ros_or_exec(self):
        startup=test_fixed_startup.startup
        with tempfile.TemporaryDirectory() as tmp:
            ref=Path(tmp)/'ref.json';ref.write_text(json.dumps(config()))
            with patch.object(startup,'online_plan') as online,patch.object(startup.os,'execvp') as execute:
                with self.assertRaises(ValueError):startup.main(['--reference',str(ref),'--base-adjust',
                    '--base-config',str(ROOT/'deploy/stage_c_fixed_base.example.yaml')])
                online.assert_not_called();execute.assert_not_called()

    @classmethod
    def setUpClass(cls):test_taught_base.DriverTests.setUpClass()

    def test_chassis_watchdog_stops_cached_velocity_when_navigator_dies(self):
        harness=test_taught_base.DriverTests();d=harness.driver();d.fixed_navigation_watchdog=.3;d.fixed_velocity_received=1.
        d.car_cmd=2;d.cmd_vel_cb(harness.twist(.02))
        with patch.object(harness.module.time,'monotonic',return_value=d.fixed_velocity_received+.31):d.controller()
        self.assertEqual(d.ser.writes[-1],d.stop_data);self.assertEqual(d.car_cmd,0)

    def test_chassis_rejects_excess_speed_nonfinite_or_rotation_in_fixed_mode(self):
        harness=test_taught_base.DriverTests()
        for velocity in ((.03,0,0),(math.nan,0,0),(0,0,.1)):
            d=harness.driver();d.fixed_navigation_watchdog=.3
            d.cmd_vel_cb(harness.twist(*velocity))
            self.assertEqual(d.ser.writes[-1],d.stop_data);self.assertEqual(d.car_cmd,0)


if __name__=='__main__':unittest.main()
