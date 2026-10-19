"""Fruit-relative stage C, no Tag/map/odom. All transports and movement are in-memory."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch
import numpy as np
from test_stage_c import ROOT
from test_fixed_grasp import config,target,Adapter
from jzzj_grasp.fixed_base import validate_base,correction,MicroNavigator,FruitTranslation
from jzzj_grasp.fixed_base_ros import NavigationAlignment
from jzzj_grasp.fixed_grasp import FixedSequence,REAR_DEG,OPEN_DEG,CLOSE_DEG
from jzzj_grasp.handoff import Handoff,validate_envelope
from jzzj_grasp.core import Failure
import test_taught_base
import test_fixed_startup
sys.path.insert(0,str(ROOT/'tools/stage_c'))
spec=importlib.util.spec_from_file_location('base_calibration',ROOT/'tools/stage_c/calibrate_fixed_base.py')
calibration=importlib.util.module_from_spec(spec);spec.loader.exec_module(calibration)
M=types.SimpleNamespace


def base_config():
    cfg=json.loads((ROOT/'deploy/stage_c_fixed_base.example.yaml').read_text())
    samples=[dict(base_delta_m=d,before_m=[0,0,-.05],after_m=[-d[0],-d[1],-.05],measurement_source='manual_ruler')
             for d in ([.01,0],[-.01,0],[0,.01],[0,-.01])]
    cfg.update(calibration_confirmed=True,stationary_fruit_confirmed=True,rigid_camera_confirmed=True,
               translation_only_confirmed=True,base_matrix=[[-1,0],[0,-1]],base_calibration_samples=samples,
               reference_calibration_sha256=config()['calibration_sha256'])
    return cfg


class MathTests(unittest.TestCase):
    def test_no_tag_pose_or_odometry_required_for_valid_calibration(self):
        cfg=base_config();validate_base(cfg,config())
        self.assertNotIn('pose_topic',cfg);self.assertNotIn('pose_frame',cfg)
        for change in (dict(version=1),dict(base_matrix=[[1,0],[0,1]]),dict(step_m=.051),
                       dict(stationary_fruit_confirmed=False),dict(rigid_camera_confirmed=False),
                       dict(reference_calibration_sha256='changed'),dict(total_m=-1),
                       dict(body_axes='camera_x_back_y_right'),dict(drive_interface='legacy')):
            c=base_config();c.update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):validate_base(c,config())

    def test_relative_body_signs_50mm_norm_and_20mm_3d_tolerance(self):
        cfg=base_config();delta=correction(cfg,[.08,0,-.05],[0,0,-.05])
        np.testing.assert_allclose(delta,[.05,0],atol=1e-10)
        diagonal=correction(cfg,[.08,.08,-.05],[0,0,-.05]);self.assertAlmostEqual(math.hypot(*diagonal),.05)
        self.assertIsNone(correction(cfg,[.019,0,-.05],[0,0,-.05]))
        self.assertIsNotNone(correction(cfg,[.021,0,-.05],[0,0,-.05]))
        self.assertIsNotNone(correction(cfg,[.0198,0,-.046],[0,0,-.05]))
        with self.assertRaises(Failure):correction(cfg,[0,0,-.04],[0,0,-.05])

    def test_ruler_pairs_fit_without_circular_visual_displacement_or_tag(self):
        reference=config();pairs=[];deltas=([.01,0],[-.01,0],[0,.01],[0,-.01])
        for delta in deltas:
            a=dict(sample_kind='stationary_fruit_no_tag_v2',reference_calibration_sha256=reference['calibration_sha256'],
                   identity=reference['identity'],arm_frame=reference['arm_frame'],
                   arm_positions_rad=np.radians(REAR_DEG).tolist(),fruit_position_m=[0,0,-.05])
            b=copy.deepcopy(a);b['fruit_position_m']=[-delta[0],-delta[1],-.05];pairs.append((a,b))
        fitted=calibration.samples_from_pairs(pairs,reference,deltas)
        matrix,residual=calibration.fit_base_matrix(fitted,[0,0,1]);np.testing.assert_allclose(matrix,[[-1,0],[0,-1]])
        self.assertLess(residual,1e-9)
        with self.assertRaises(ValueError):calibration.samples_from_pairs(pairs,reference,[])
        pairs[0][1]['sample_kind']='old_map_sample'
        with self.assertRaises(ValueError):calibration.samples_from_pairs(pairs,reference,deltas)


class VisualFeedbackTests(unittest.TestCase):
    def tracker(self):return FruitTranslation(base_config(),config(),'s',1.)
    def test_fruit_change_measures_relative_translation_without_any_pose_provider(self):
        tracker=self.tracker();p=list(config()['reference_position_m'])
        value=tracker.add(2.,'s','ok',[target(2.,p)],2.01);self.assertEqual(value['x'],0.)
        p[0]-=.004
        value=tracker.add(2.2,'s','ok',[target(2.2,p)],2.21)
        self.assertAlmostEqual(value['x'],.004);self.assertIn('yaw unobserved',value['diagnostic'])
    def test_empty_stale_ambiguous_track_change_and_jump_fail_closed(self):
        for kind in ('empty','stale','ambiguous','track','jump'):
            t=self.tracker();t.add(2.,'s','ok',[target(2.)],2.01)
            point=list(config()['reference_position_m']);point[0]+=.03
            choices=[] if kind=='empty' else [target(2.1),target(2.1)] if kind=='ambiguous' else [target(2.1,point if kind=='jump' else None,2 if kind=='track' else 1)]
            with self.subTest(kind=kind),self.assertRaises(Failure):
                t.add(2.1,'s','ok',choices,3. if kind=='stale' else 2.11)
    def test_wrong_session_and_old_frame_never_update_feedback(self):
        t=self.tracker();self.assertIsNone(t.add(2.,'wrong','ok',[target(2.)],2.01))
        t.add(2.,'s','ok',[target(2.)],2.01)
        self.assertIsNone(t.add(2.,'s','ok',[target(2.)],2.02))


class NavigationTests(unittest.TestCase):
    def armed(self,delta=(.01,0)):
        n=MicroNavigator();n.command(True,1.);n.accept(delta,(0,0),1,1.01);return n
    def test_50mm_maximum_and_relative_origin(self):
        n=self.armed((.05,0));self.assertEqual(n.goal,(.05,0))
        for delta in ((.051,0),(.04,.04),(math.nan,0)):
            with self.assertRaises(ValueError):self.armed(delta)
        n=MicroNavigator();n.command(True,1.);n.accept((.01,0),(.4,.2),1,1.01)
        np.testing.assert_allclose(n.goal,(.41,.2))
    def test_speed_and_arrival_use_distinct_visual_frames(self):
        n=self.armed();velocity,_=n.tick((0,0),1.02,.02,1.05);self.assertLessEqual(math.hypot(*velocity),.02)
        for _ in range(10):self.assertFalse(n.tick((.01,0),1.1,.01,1.11)[1])
        for stamp in (1.15,1.2):self.assertFalse(n.tick((.01,0),stamp,0,stamp)[1])
        self.assertTrue(n.tick((.01,0),1.25,0,1.25)[1])
    def test_loss_timeout_overshoot_explicit_hold_and_replay(self):
        for position,age,now in (((0,0),.6,1.1),((0,0),.01,1.5),((.056,0),.01,1.1)):
            n=self.armed();self.assertEqual(n.tick(position,1.,age,now)[0],(0,0));self.assertIsNone(n.goal)
            n.command(True,now+.01);self.assertEqual(n.tick((0,0),1.,0,now+.02)[0],(0,0))
        n=self.armed();n.command(False,1.1);self.assertIsNone(n.goal);n.command(True,1.2)
        with self.assertRaises(ValueError):n.accept((.01,0),(0,0),1,1.21)


class HandoffTests(unittest.TestCase):
    def request(self,m,op='TAKEOVER',session='s',seq=1,**kw):
        m.request(op,session,seq,10.,13.,10.1,**kw)
    def test_complete_state_sequence_and_explicit_transfer(self):
        m=Handoff();self.request(m,confirmed_stopped=True)
        for state in ('STOP_CONFIRM','OBSERVE','BASE_ADJUST','STOP_CONFIRM','OBSERVE','STOP_CONFIRM','ARM_GRASP'):
            m.transition(state)
        m.finish(dict(status='SUCCESS',motion_complete=True,success=True))
        self.assertEqual(m.state,'DONE_WAIT_HANDOFF')
        with self.assertRaises(ValueError):m.released()
        self.request(m,'HANDOFF_ACK',seq=2,confirmed_stopped=True,next_owner='tag_navigation')
        m.released();self.assertEqual(m.state,'RELEASED');self.assertEqual(m.owner,'tag_navigation')
        with self.assertRaises(ValueError):self.request(m,seq=3,confirmed_stopped=True)
    def test_replay_stale_busy_and_wrong_session(self):
        m=Handoff();self.request(m,confirmed_stopped=True)
        for op,session,seq in (('CANCEL','s',1),('CANCEL','other',2),('TAKEOVER','other',2)):
            with self.assertRaises(ValueError):self.request(m,op,session,seq,confirmed_stopped=True)
        for issued,expires,now in ((1.,2.,10.),(20.,21.,10.),(1.,100.,2.)):
            with self.assertRaises(ValueError):validate_envelope('s',2,issued,expires,now)
    def test_cancel_fault_and_recovery_require_external_confirmation(self):
        m=Handoff();self.request(m,confirmed_stopped=True);self.request(m,'CANCEL',seq=2)
        self.assertTrue(m.cancel_requested)
        m.finish(dict(status='CANCELED',motion_complete=False,error='operator canceled'))
        with self.assertRaises(ValueError):self.request(m,'RECOVER',seq=3,confirmed_stopped=True)
        self.request(m,'RECOVER',seq=3,confirmed_stopped=True,confirmed_empty_and_clear=True)
        m.released(recovered=True);self.assertEqual(m.state,'RELEASED')
    def test_restart_preserves_used_sessions_and_latches_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'state.json';m=Handoff(path);self.request(m,confirmed_stopped=True);m.transition('OBSERVE')
            restored=Handoff(path);self.assertEqual(restored.state,'FAULT');self.assertTrue(restored.recovery_required)
            with self.assertRaises(ValueError):self.request(restored,session='new',seq=2,confirmed_stopped=True)
            self.request(restored,'RECOVER',seq=2,confirmed_stopped=True,confirmed_empty_and_clear=True)
            restored.released(recovered=True)
            self.assertEqual(Handoff(path).state,'RELEASED')


class IntegrationTests(unittest.TestCase):
    def test_unlimited_alignment_exceeds_old_100mm_and_15step_limits_without_tag(self):
        cfg=base_config();cfg['step_m']=.01;a=NavigationAlignment.__new__(NavigationAlignment)
        a.cfg=cfg;a.trace=[];a.xy=[0.,0.];a.steps=0;a.set_state=lambda *args:None
        a.io=M(arm=M(measured=lambda:[0.]*6),cfg={'reference_position_m':[0.,0.,-.05]},guard=lambda:None,deadline=0.)
        a.stop=lambda:None;a.current=lambda:(tuple(a.xy),1.)
        a.observe=lambda *args:(.3-a.xy[0],0.,-.05)
        def navigate(delta):a.xy[:]=[x+d for x,d in zip(a.xy,delta)];a.steps+=1
        a.navigate=navigate
        point=a.align();self.assertLessEqual(math.hypot(*point[:2]),.02)
        self.assertGreater(a.steps,15);self.assertGreater(a.xy[0],.1);self.assertFalse(a.aligning)

    def test_complete_fixed_grasp_after_visual_alignment_reports_success(self):
        events=[]
        class IO(Adapter):
            def wait_reference(self):
                cfg=base_config();point=[.04,0,-.05];reference=[0,0,-.05]
                while True:
                    delta=correction(cfg,point,reference)
                    if delta is None:return point
                    events.append(('relative_step',delta));point[:2]=np.asarray(point[:2])+np.asarray(cfg['base_matrix'])@delta
            def prepare_grasp(self):events.append(('visual_stop_confirmed',1.))
        io=IO();result=FixedSequence(io,config()).run()
        self.assertEqual(result['status'],'SUCCESS');self.assertTrue(result['success'])
        self.assertEqual(events[0][0],'relative_step');self.assertEqual(events[-1][0],'visual_stop_confirmed')
        self.assertIn(('move',OPEN_DEG),io.events);self.assertIn(('move',CLOSE_DEG),io.events)

    def test_no_tag_or_pose_subscription_in_stage_c_runtime(self):
        for rel in ('src/jzzj_grasp/fixed_base.py','src/jzzj_grasp/fixed_base_ros.py','scripts/stage_c_fixed_navigation.py'):
            source=(ROOT/'mission_ws/src/mission/jzzj_grasp'/rel).read_text()
            self.assertNotIn('/front_car_pose_orig_map',source);self.assertNotIn('map_pose(',source)
        import xml.etree.ElementTree as ET
        root=ET.parse(ROOT/'mission_ws/src/mission/jzzj_grasp/launch/stage_c_fixed_base.launch').getroot()
        self.assertFalse(root.findall('.//include'))

    def test_scheduler_uses_only_existing_target_and_mode_interfaces(self):
        tree=ast.parse((ROOT/'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_base_ros.py').read_text())
        topics=[n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                and n.func.attr=='Publisher' and n.args and isinstance(n.args[0],ast.Constant)]
        self.assertEqual(set(topics),{'/car_target','/car_cmd'})

    def test_waiting_handoff_does_not_publish_velocity_and_wrong_frame_rejected(self):
        spec=importlib.util.spec_from_file_location('visual_navigation',ROOT/'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_navigation.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        n=module.Navigation.__new__(module.Navigation);n.lock=threading.RLock();n.session='';n.vel=None
        n.tick();self.assertIsNone(n.vel)
        n.session='s';n.core=MicroNavigator();n.fault='';n.recovering=False;n.activated_at=1.;n.graph=lambda:None
        n.cmd=M(HOLD=0);n.r=M(Time=M(now=lambda:M(to_sec=lambda:2.)))
        out=[];n.publish=lambda velocity,mode:out.append((velocity,mode))
        msg=M(_connection_header={'callerid':'/stage_c_fixed_grasp'},header=M(frame_id='map',stamp=M(to_sec=lambda:2.)),
              pose=M(position=M(x=.01,y=0,z=0),orientation=M(x=0,y=0,z=0,w=1.)))
        n.target(msg);self.assertIn('stage_c_body',n.fault);self.assertEqual(out[-1],((0.,0.),0))

    def test_old_navigation_conflict_and_uncalibrated_start_refused(self):
        startup=test_fixed_startup.startup;graph=test_fixed_startup.graph
        flags=startup.launch_flags(graph(),True);self.assertNotIn('start_localization',flags)
        for state in (graph(['/chassis']),graph(['/wp_nav_pid_node']),graph(subs=[('/car_target',['/other'])])):
            with self.assertRaises(RuntimeError):startup.launch_flags(state,True)
        with tempfile.TemporaryDirectory() as tmp:
            ref=Path(tmp)/'ref.json';ref.write_text(json.dumps(config()))
            with patch.object(startup,'online_plan') as online,patch.object(startup.os,'execvp') as execute:
                with self.assertRaises(ValueError):startup.main(['--reference',str(ref),'--base-adjust',
                    '--base-config',str(ROOT/'deploy/stage_c_fixed_base.example.yaml')])
                online.assert_not_called();execute.assert_not_called()

    def test_visual_stop_is_not_faked_by_hold_or_delay(self):
        a=NavigationAlignment.__new__(NavigationAlignment);a.phase='VISUAL';a.r=M(Time=M(now=lambda:M(to_sec=lambda:1.)),is_shutdown=lambda:False)
        a.cmd=lambda **kw:kw;a.cmd.HOLD=0;a.mode=M(publish=lambda m:None)
        a.current=lambda:(_ for _ in ()).throw(Failure('SENSOR_INVALID','no fruit'))
        with self.assertRaises(Failure):a.stop()

    @classmethod
    def setUpClass(cls):test_taught_base.DriverTests.setUpClass()
    def test_driver_watchdog_stops_if_adapter_disappears(self):
        h=test_taught_base.DriverTests();d=h.driver();d.fixed_navigation_watchdog=.3;d.fixed_velocity_received=1.
        d.car_cmd=2;d.cmd_vel_cb(h.twist(.02))
        with patch.object(h.module.time,'monotonic',return_value=d.fixed_velocity_received+.31):d.controller()
        self.assertEqual(d.ser.writes[-1],d.stop_data);self.assertEqual(d.car_cmd,0)

    def test_calib_d_cyy_channels_and_signs_are_used_only_in_stage_c(self):
        h=test_taught_base.DriverTests();d=h.driver();d.fixed_navigation_watchdog=.3;d.fixed_calib_interface=True
        d.xy_activate_zone=0.;d.xy_dead_zone=0.;d.max_vel_linear=.02
        for vx,vy in ((.02,0),(-.02,0),(0,.02),(0,-.02),(.01,.01)):
            d.cmd_vel_cb(h.twist(vx,vy))
            expected=[int(vx*d.vel_2_0x40)&255,-int(vy*d.vel_2_0x40)&255,0]
            self.assertEqual(d.chassis_cmd_data[2:5],expected)
        d.fixed_calib_interface=False
        d.cmd_vel_cb(h.twist(.02))
        self.assertEqual(d.chassis_cmd_data[2:5],[0,-int(.02*d.vel_2_0x40)&255,0])

    def test_calib_axis_mapping_does_not_bypass_speed_yaw_or_watchdog_limits(self):
        h=test_taught_base.DriverTests();d=h.driver();d.fixed_navigation_watchdog=.3;d.fixed_calib_interface=True
        for vx,vy,wz in ((.021,0,0),(.02,.02,0),(0,0,.01),(float('nan'),0,0)):
            d.cmd_vel_cb(h.twist(vx,vy,wz))
            self.assertEqual(d.car_cmd,0);self.assertEqual(d.chassis_cmd_data,d.stop_data)

    def test_zero_write_evidence_requires_full_serial_write(self):
        h=test_taught_base.DriverTests();d=h.driver();published=[]
        d.fixed_status_pub=M(publish=published.append);d.fixed_status_type=lambda:M(header=M())
        with patch.object(h.module.rospy,'Time',M(now=lambda:1.),create=True):
            d.fixed_write_stop();self.assertTrue(published[-1].serial_stop_written)
            d.ser.write=lambda data:0;d.fixed_write_stop();self.assertFalse(published[-1].serial_stop_written)
        self.assertIn('no encoder',published[-1].diagnostic)

    def test_driver_profile_restores_normal_limits_in_hold_without_resuming_navigation(self):
        h=test_taught_base.DriverTests();d=h.driver();d.RA2DE=180/math.pi;d.fixed_navigation_watchdog=.3
        d.fixed_handoff_defaults=dict(dead_zone_xy=0,activate_zone_xy=.06,dead_zone_z=0,activate_zone_z=11,max_vel_linear=.5,max_vel_angular=60)
        d.fixed_profile_response=lambda success,message:M(success=success,message=message)
        request=M(data=False,_connection_header={'callerid':'/other'})
        self.assertFalse(d.fixed_profile(request).success)
        request._connection_header['callerid']='/stage_c_fixed_grasp'
        with patch.object(h.module.rospy,'set_param',lambda *args:None,create=True):
            self.assertTrue(d.fixed_profile(request).success)
            self.assertEqual(d.fixed_navigation_watchdog,0.);self.assertEqual(d.car_cmd,0)
            self.assertEqual(d.max_vel_linear,.5);self.assertEqual(d.xy_activate_zone,.06)
            self.assertFalse(d.fixed_calib_interface)
            request.data=True;self.assertTrue(d.fixed_profile(request).success)
            self.assertEqual(d.max_vel_linear,.02);self.assertEqual(d.fixed_navigation_watchdog,.3)
            self.assertTrue(d.fixed_calib_interface)
        self.assertTrue(all(packet==d.stop_data for packet in d.ser.writes))


if __name__=='__main__':unittest.main()
