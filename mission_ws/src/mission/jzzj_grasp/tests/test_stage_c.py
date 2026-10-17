"""Run using standard unittest, NumPy, no ROS/camera/Arm_Lib/model needed."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[5]
for p in ('mission_ws/src/mission/jzzj_grasp/src','mission_ws/src/mission/jzzj_visual_servo/src',
          'catkin_ws/src/actuator/arm/scripts','mission_ws/src/mission/jzzj_grasp/scripts'):
    sys.path.insert(0,str(ROOT/p))
from jzzj_grasp.core import (Engine,Store,Context,Observation,Failure,step_toward,validate_config)
from jzzj_grasp.backends import SimBackend
from jzzj_visual_servo.perception import (Candidate,DetectorAdapter,colour_mask,project_depth_to_rgb,validate_transform)
from trajectory_safety import encode,decode,validate_mapping,validate_target
from check_model import check
BASE=json.loads((ROOT/'mission_ws/src/mission/jzzj_grasp/params/stage_c.yaml').read_text())

class Harness:
    def __init__(self):
        self.t=100.;self.cfg=copy.deepcopy(BASE['execution']);self.cfg['search_timeout']=.15
        self.store=Store();self.backend=SimBackend(self.store,self.cfg,lambda:self.t)
        self.fault=None;self.steps=[];self.heartbeat=True;self.frames=True
        self.engine=Engine(self.backend,self.store,self.cfg,now=lambda:self.t,monotonic=lambda:self.t,sleep=self.sleep,
                           feedback=lambda step,*a:self.steps.append(step))
        self.backend.select('s','red');self.refresh()
    def refresh(self):
        if self.heartbeat:self.store.context=Context(self.t,'s',True,True,False,self.t)
        if self.frames:self.backend.refresh()
        if self.fault:self.fault(self)
    def sleep(self,seconds):self.t+=seconds;self.refresh()
    def run(self):return self.engine.execute('s','red','tabletop',30.)

class EngineTests(unittest.TestCase):
    def test_full_simulation(self):
        h=Harness();o=h.run();self.assertEqual(o.code,'SUCCESS',o.message)
        self.assertTrue(o.grasp_confirmed and o.ready_for_transport and o.holding_object)
        self.assertIn('VERIFY_GRASP',h.steps)
        self.assertNotIn('cmd_vel',str(h.backend.trace))
        moves=[x[1] for x in h.backend.trace if x[0]=='move']
        for a,b in zip([tuple(h.cfg['sim_initial_tcp'])]+moves,moves):self.assertLessEqual(math.dist(a,b),h.cfg['max_step_m']+1e-9)
    def test_stale_context(self):
        h=Harness();h.heartbeat=False;h.t+=2
        self.assertEqual(h.run().code,'NOT_READY');self.assertFalse(any(x[0]=='move' for x in h.backend.trace))
    def test_wrong_session(self):
        h=Harness();h.store.context=Context(h.t,'other',True,True,False,h.t)
        self.assertEqual(h.run().code,'NOT_READY')
    def test_no_target(self):
        h=Harness();h.frames=False;h.store.observations=[]
        self.assertEqual(h.run().code,'NO_TARGET');self.assertNotIn('grip',[x[0] for x in h.backend.trace])
    def test_unreachable(self):
        h=Harness();h.backend.object_position=(2.,0.,.1);h.refresh()
        self.assertEqual(h.run().code,'NEEDS_REPOSITION')
    def test_cancel(self):
        h=Harness();h.engine.cancel.set();o=h.run()
        self.assertEqual(o.code,'CANCELED');self.assertTrue(o.stop_confirmed)
    def test_cancel_mid_close(self):
        h=Harness()
        def fault(h):
            if h.backend.closed:h.engine.cancel.set()
        h.fault=fault;o=h.run();self.assertEqual(o.code,'CANCELED');self.assertFalse(o.holding_known)
    def test_stop_failure_latched_result(self):
        h=Harness();h.backend.hold=lambda t:False;h.engine.cancel.set()
        self.assertEqual(h.run().code,'STOP_FAILED')
    def test_drop_cannot_succeed(self):
        h=Harness();h.backend.drop=True
        self.assertEqual(h.run().code,'GRASP_UNCONFIRMED')
    def test_duplicate_frame_not_stable(self):
        h=Harness();h.frames=False;o=h.run()
        self.assertNotEqual(o.code,'SUCCESS');self.assertNotIn('CLOSE_GRIPPER',h.steps)
    def test_wrong_frame_or_color(self):
        for field,value in [('frame','wrong'),('color','blue'),('quality',float('nan'))]:
            h=Harness();h.frames=False;d=h.store.observations[0].__dict__.copy();d[field]=value
            h.store.observations=[Observation(**d)];self.assertEqual(h.run().code,'NO_TARGET')
    def test_timeout(self):
        h=Harness();h.frames=False;h.store.observations=[]
        self.assertEqual(h.engine.execute('s','red','tabletop',.03).code,'TIMEOUT')
    def test_motion_failure(self):
        h=Harness();h.backend.reject_motion=True
        self.assertEqual(h.run().code,'MOTION_FAILED')
    def test_sensor_diagnostic(self):
        h=Harness();h.frames=False;h.store.observations=[];h.store.diagnostic='depth not calibrated'
        self.assertEqual(h.run().code,'SENSOR_INVALID')
    def test_broken_profile_or_nan(self):
        for field in ('max_step_m','search_timeout','safe_tcp_min_height_m'):
            c=copy.deepcopy(BASE['execution']);c[field]=float('nan')
            with self.assertRaises(ValueError):validate_config(c)
    def test_step_limit_and_finite(self):
        self.assertAlmostEqual(math.dist((0,0,0),step_toward((0,0,0),(1,1,1),.01)),.01)
        with self.assertRaises(Failure):step_toward((0,0,float('nan')),(1,1,1),.01)


    def test_context_lost_while_moving(self):
        h=Harness()
        def fault(h):
            if any(x[0]=='move' for x in h.backend.trace):h.store.context=Context(h.t,'s',False,True,False,h.t)
        h.fault=fault;self.assertEqual(h.run().code,'NOT_READY')
    def test_future_or_old_observation(self):
        for stamp in (0.,99.,101.):
            h=Harness();h.frames=False;d=h.store.observations[0].__dict__.copy();d['stamp']=stamp
            h.store.observations=[Observation(**d)];self.assertEqual(h.run().code,'NO_TARGET')
    def test_class_change_cannot_relock(self):
        h=Harness()
        def fault(h):
            if any(x[0]=='move' for x in h.backend.trace):
                d=h.store.observations[0].__dict__.copy();d.update(class_id=9,label='other')
                h.store.observations=[Observation(**d)]
        h.fault=fault;self.assertNotEqual(h.run().code,'SUCCESS')
    def test_grasp_profile_height_gate_before_open(self):
        h=Harness();h.backend.object_position=(.15,0.,.3);h.refresh()
        self.assertEqual(h.run().code,'NEEDS_REPOSITION');self.assertNotIn('grip',[x[0] for x in h.backend.trace])
    def test_transport_drop_not_success(self):
        h=Harness()
        def fault(h):
            if 'MOVE_TO_TRANSPORT_POSE' in h.steps:
                h.backend.attached=False;h.backend.object_position=(.15,0.,.1)
        h.fault=fault;self.assertEqual(h.run().code,'GRASP_UNCONFIRMED')
    def test_same_id_not_sufficient_proof(self):
        h=Harness();h.backend.drop=True
        # Simulation keeps the same ID even when the object stays on the table.
        self.assertEqual(h.run().code,'GRASP_UNCONFIRMED')
    def test_ambiguous_verification_fails(self):
        h=Harness()
        def fault(h):
            if h.backend.attached:
                a=h.store.observations[0];d=a.__dict__.copy();d['track_id']=99
                h.store.observations.append(Observation(**d))
        h.fault=fault;self.assertEqual(h.run().code,'GRASP_UNCONFIRMED')
    def test_tilted_arm_frame_uses_calibrated_table_normal(self):
        h=Harness()
        # Swap arm X and Z: physical upward is now arm X, not arm Z.
        swap=lambda p:(p[2],p[1],p[0])
        pr=h.cfg['profiles']['tabletop'];pr['vertical_axis']=[1.,0.,0.]
        pr['surface_to_grasp']=list(swap(pr['surface_to_grasp']))
        pr['transport_position']=list(swap(pr['transport_position']))
        h.backend.position=swap(h.backend.position);h.backend.object_position=swap(h.backend.object_position)
        h.cfg['workspace_min']=[.04,-.28,-.28];h.cfg['workspace_max']=[.40,.28,.28]
        h.refresh();self.assertEqual(h.run().code,'SUCCESS')
    def test_invalid_goal(self):
        h=Harness();self.assertEqual(h.engine.execute('s','purple','tabletop',30).code,'NOT_READY')

class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.c=copy.deepcopy(BASE['perception']);self.c['allowed_labels']=['object']
        self.adapter=DetectorAdapter(self.c)
        self.rgb=np.zeros((40,40,3),dtype=np.uint8);self.rgb[5:35,5:35,2]=255
        self.depth=np.full((40,40),.3);self.k=[100,0,20,0,100,20,0,0,1]
        self.det=Candidate(1,0,'object',.9,(5,5,35,35))
    def extract(self,det=None):return self.adapter.extract(self.rgb,self.depth,self.k,[det or self.det],'red')
    def test_box_depth_and_projection(self):
        out=self.extract();self.assertEqual(len(out),1);self.assertAlmostEqual(out[0][1][2],.3)
    def test_no_yolo_no_detection(self):
        self.assertEqual(self.adapter.extract(self.rgb,self.depth,self.k,[],'red'),[])
    def test_wrong_color(self):
        self.assertEqual(self.adapter.extract(self.rgb,self.depth,self.k,[self.det],'blue'),[])
    def test_invalid_depth(self):
        for v in (0,float('nan'),3.):self.depth[:]=v;self.assertEqual(self.extract(),[])
    def test_outlier_depth(self):
        self.depth[10,10]=1.;self.assertAlmostEqual(self.extract()[0][1][2],.3)
    def test_low_confidence_label_and_bad_box(self):
        for det in (Candidate(1,0,'other',.9,(5,5,35,35)),Candidate(1,0,'object',.1,(5,5,35,35)),
                    Candidate(1,0,'object',.9,(35,5,5,35)),Candidate(1,0,'object',.9,(float('nan'),5,35,35))):
            self.assertEqual(self.extract(det),[])
    def test_mismatched_shape(self):
        with self.assertRaises(ValueError):self.adapter.extract(self.rgb,self.depth[:2],self.k,[self.det],'red')
    def test_depth_registration_identity(self):
        out=project_depth_to_rgb(self.depth,self.k,self.k,np.eye(4),(40,40),1.)
        np.testing.assert_allclose(out,self.depth)
    def test_invalid_transform(self):
        t=np.eye(4);t[0,0]=2
        with self.assertRaises(ValueError):validate_transform(t)
    def test_allowlist_is_mandatory(self):
        self.c['allowed_labels']=[]
        with self.assertRaises(ValueError):DetectorAdapter(self.c).extract(self.rgb,self.depth,self.k,[self.det],'red')
    def test_hue_wrap(self):
        rgb=np.zeros((10,10,3),dtype=np.uint8);rgb[:,:,2]=255
        self.assertGreater(colour_mask(rgb,self.c['colors']['red']).sum(),0)

class DriverAndModelTests(unittest.TestCase):
    def test_round_trip_all_six_signs_offsets(self):
        q=[.1,-.2,.3,-.4,.5,1.];signs=[1,-1,1,1,1,-1];off=[2,3,4,5,6,7]
        np.testing.assert_allclose(decode(encode(q,signs,off),signs,off),q)
    def test_mapping_and_joint_names(self):
        n=['joint'+str(i) for i in range(1,7)]
        validate_mapping([1]*6,[0]*6,[-2]*6,[2]*6)
        self.assertEqual(validate_target(n[::-1],[.1]*6,n,[-2]*6,[2]*6),[.1]*6)
        with self.assertRaises(ValueError):validate_target(['joint1']*6,[0]*6,n,[-2]*6,[2]*6)
        with self.assertRaises(ValueError):validate_target(n,[float('nan')]*6,n,[-2]*6,[2]*6)
    def test_model_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'m.rknn';labels=Path(tmp)/'labels.txt';model.write_bytes(b'fake model; not ABI validated')
            labels.write_text('2\nbox\ncan\n');self.assertEqual(check(model,labels,2)['labels'],['box','can'])
            labels.write_text('2\nbox\nbox\n')
            with self.assertRaises(ValueError):check(model,labels,2)

if __name__=='__main__':unittest.main()
