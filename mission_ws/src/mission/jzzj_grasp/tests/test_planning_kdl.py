"""Same planner + actual URDF/KDL. Set STAGE_C_KDL_PROBE to the test binary."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET
from test_stage_c import ROOT, Harness
from jzzj_grasp.core import Failure
from jzzj_grasp.planning import PathPlanner, Pose


@unittest.skipUnless(os.environ.get('STAGE_C_KDL_PROBE'),'native KDL probe not configured')
class ActualKdlPlanningTests(unittest.TestCase):
    def setUp(self):
        self.process=subprocess.Popen([os.environ['STAGE_C_KDL_PROBE'],
            str(ROOT/'catkin_ws/src/actuator/arm/urdf/arm.urdf'),'--query'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        self.cfg=copy.deepcopy(Harness().cfg);self.cfg['active_profile']='tabletop'
        robot=ET.parse(ROOT/'catkin_ws/src/actuator/arm/urdf/arm.urdf')
        limits=[robot.find("./joint[@name='joint%d']/limit"%i) for i in range(1,7)]
        self.lower=[float(l.get('lower')) for l in limits]
        self.upper=[float(l.get('upper')) for l in limits]
    def tearDown(self):
        self.process.stdin.close();self.process.wait(timeout=5)
        self.process.stdout.close();self.process.stderr.close()
    def query(self,op,q,point=(),rotation=()):
        values=list(q[:5])+list(point)+list(rotation)
        self.process.stdin.write(op+' '+' '.join(repr(v) for v in values)+'\n');self.process.stdin.flush()
        line=self.process.stdout.readline().strip()
        if not line.startswith('OK '):raise Failure('NEEDS_REPOSITION',line)
        data=[float(x) for x in line.split()[1:]]
        return Pose(tuple(data[:3]),tuple(data[3:7])),tuple(data[7:])+(q[5],)
    def planner(self):
        return PathPlanner(self.cfg,self.lower,self.upper,
            lambda q:self.query('F',q)[0],lambda p,r,q:self.query('I',q,p,r)[1],lambda:None)
    def test_field_pose_rejects_before_any_open_or_arm_command(self):
        self.cfg=json.loads((ROOT/'deploy/stage_c.yaml').read_text(encoding='utf-8'))['execution']
        self.cfg['active_profile']='tabletop'
        q=tuple(math.radians(v) for v in [89,-3,90,90,-1,58])
        h=Harness()
        h.backend.preflight=lambda *a,**k:self.planner().plan(q,(-.2719288,.1058806,-.05))
        result=h.run()
        self.assertEqual(result.code,'NEEDS_REPOSITION',result.message)
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(any(x[0] in ('grip','move') for x in h.backend.trace))
    def test_actual_urdf_reachable_descent_lift_transport_pass(self):
        q=(0.,.3,.5,.3,.2,1.)
        first=self.query('F',q)[0];p=self.cfg['profiles']['tabletop']
        p.update(orientation_xyzw=list(first.orientation),surface_to_grasp=[0.,0.,0.],
                 approach_height=.005,lift_height=.01,transport_position=list(first.position),
                 table_plane_offset_m=first.position[2]-.2,transit_clearance_height_m=.15)
        self.cfg['workspace_min']=[v-.15 for v in first.position]
        self.cfg['workspace_max']=[v+.15 for v in first.position]
        surface=tuple(first.position[:2])+(first.position[2]-.005,)
        prefix,route=self.planner().plan(q,surface)
        self.assertEqual(prefix,[])
        self.assertGreaterEqual(len(route),3)
        self.assertTrue(all(x.joints[5]==q[5] for x in route))


if __name__=='__main__':unittest.main()
