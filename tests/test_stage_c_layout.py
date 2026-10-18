"""Layout/path regression checks; never starts ROS or hardware."""
import ast
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

class LayoutTests(unittest.TestCase):
    def test_c_tools_and_launches_have_no_retired_source_paths(self):
        bases=[ROOT/'tools/stage_c',ROOT/'deploy',ROOT/'mission_ws/src/mission/jzzj_grasp',ROOT/'mission_ws/src/mission/jzzj_visual_servo']
        for base in bases:
            for p in base.rglob('*'):
                if p.is_file() and p.suffix in ('.py','.sh','.yaml','.launch'):
                    text=p.read_text()
                    for old in ('/home/orangepi/jzzj-stagec','/home/orangepi/stagec_verify_ws'):
                        self.assertNotIn(old,text,str(p))
    def test_tools_parse_without_importing_hardware(self):
        for p in (ROOT/'tools/stage_c').glob('*.py'):ast.parse(p.read_text())
        for p in (ROOT/'mission_ws/src/mission/jzzj_grasp/launch').glob('*.launch'):ET.parse(p)
    def test_new_entry_points_do_not_kill_unrelated_tasks(self):
        for p in (ROOT/'tools/stage_c').glob('*.sh'):
            self.assertNotIn('pkill',p.read_text(),str(p))
    def test_measured_calibration_points_are_preserved(self):
        points=json.loads((ROOT/'deploy/calib_points.json').read_text())
        self.assertEqual(len(points),3)
        self.assertTrue(all(set(('u','v','x','y','z'))<=set(p) for p in points))

if __name__=='__main__':unittest.main()
