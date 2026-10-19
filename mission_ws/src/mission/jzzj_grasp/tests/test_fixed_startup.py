"""Startup orchestration checks without ROS or hardware."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from test_fixed_grasp import ROOT, config
from jzzj_grasp.core import Failure
from jzzj_grasp.fixed_grasp_ros import wait_startup

spec = importlib.util.spec_from_file_location('startup', ROOT / 'tools/stage_c/start_fixed_grasp.py')
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)

runner_spec = importlib.util.spec_from_file_location('fixed_runner', ROOT / 'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_grasp.py')
runner = importlib.util.module_from_spec(runner_spec)
runner_spec.loader.exec_module(runner)


def graph(nodes=(), pubs=(), subs=()):
    return [list(pubs), list(subs), [('/service' + str(i), [n]) for i, n in enumerate(nodes)]]


class StartupTests(unittest.TestCase):
    def test_runner_accepts_roslaunch_remaps_using_ros_parser(self):
        with tempfile.TemporaryDirectory() as tmp:
            ref=Path(tmp)/'reference.json';ref.write_text(json.dumps(config()))
            argv=['check','--reference',str(ref),'__name:=stage_c_fixed_grasp','__log:=/tmp/fixed.log']
            ros_parser=unittest.mock.Mock(side_effect=lambda argv: [a for a in argv if ':=' not in a])
            with patch.dict('sys.modules',{'rospy':types.SimpleNamespace(myargv=ros_parser)}):
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    runner.main(argv)
            ros_parser.assert_called_once_with(argv=['stage_c_fixed_grasp.py']+argv)
            self.assertIn('No movement.',output.getvalue())

    def test_runner_still_rejects_unknown_cli_flags(self):
        with patch.dict('sys.modules',{'rospy':types.SimpleNamespace(myargv=lambda argv:[a for a in argv if ':=' not in a])}):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
                runner.main(['check','--reference','unused.json','--bogus','__name:=fixed'])
            self.assertEqual(error.exception.code,2)

    def test_runner_offline_check_does_not_require_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            ref=Path(tmp)/'reference.json';ref.write_text(json.dumps(config()))
            with patch.dict('sys.modules',{'rospy':None}),contextlib.redirect_stdout(io.StringIO()) as output:
                runner.main(['check','--reference',str(ref)])
            self.assertIn('50mm',output.getvalue())

    def test_empty_graph_starts_dependencies(self):
        self.assertTrue(all(startup.launch_flags(graph()).values()))

    def test_complete_graph_reuses_dependencies(self):
        nodes = ('/ascamera_hp60c', '/stage_c_observer', '/yolov5_rknn_rc/yolov5_rknn_stream',
                 '/arm_left/arm_trajectory')
        flags = startup.launch_flags(graph(nodes, [('/arm_left/joint_states', ['/arm_left/arm_trajectory'])]))
        self.assertFalse(any(flags.values()))

    def test_partial_vision_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'Partial vision'):
            startup.launch_flags(graph(['/stage_c_observer']))

    def test_base_and_legacy_conflicts(self):
        for state in (graph(subs=[('/cmd_vel', ['/chassis'])]),
                      graph(pubs=[('/arm_left/arm_joint_states', ['/debug'])]),
                      graph(['/arm_left/arm_test_node']), graph(['/stage_c_server']),
                      graph(['/stage_c_fixed_grasp_123']),
                      graph(pubs=[('/arm_left/joint_states', ['/unknown'])])):
            with self.subTest(state=state), self.assertRaises(RuntimeError):
                startup.launch_flags(state)

    def test_master_absent_only_without_device_owners(self):
        proxy = types.SimpleNamespace(getSystemState=lambda *a: (_ for _ in ()).throw(ConnectionRefusedError()))
        with patch.object(startup.xmlrpc.client, 'ServerProxy', return_value=proxy):
            with patch.object(startup, 'device_processes', return_value=[]):
                self.assertTrue(all(startup.online_plan().values()))
            with patch.object(startup, 'device_processes', return_value=['arm_trajectory_node.py']):
                with self.assertRaisesRegex(RuntimeError, 'processes still exist'):
                    startup.online_plan()

    def test_recovery_blocks_startup(self):
        proxy = types.SimpleNamespace(getSystemState=lambda *a: (1, '', graph()),
                                      getParam=lambda *a: (1, '', True))
        with patch.object(startup.xmlrpc.client, 'ServerProxy', return_value=proxy):
            with self.assertRaisesRegex(RuntimeError, 'Recovery required'):
                startup.online_plan()

    def test_offline_and_explicit_real_gates(self):
        with tempfile.TemporaryDirectory() as tmp:
            ref = Path(tmp) / 'reference.json'
            cfg = config()
            ref.write_text(json.dumps(cfg))
            with patch.object(startup, 'online_plan') as online, patch.object(startup.os, 'execvp') as execute:
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    startup.main(['--reference', str(ref)])
                    with self.assertRaises(SystemExit):
                        startup.main(['--reference', str(ref), '--real'])
                    with self.assertRaises(SystemExit):
                        startup.main(['--reference', str(ref), '--real', '--confirm-base-immobilized', '--confirm-fixed-path-clear'])
                online.assert_not_called()
                execute.assert_not_called()

    def test_real_launch_is_one_managed_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            ref = Path(tmp) / 'reference.json'
            cfg = config(); cfg['real_motion_enabled'] = True
            ref.write_text(json.dumps(cfg))
            flags = dict(start_arm=True, start_observe=False, start_camera=False)
            with patch.object(startup, 'online_plan', return_value=flags), patch.object(startup.os, 'execvp') as execute:
                with contextlib.redirect_stdout(io.StringIO()):
                    startup.main(['--reference', str(ref), '--real', '--confirm-base-immobilized', '--confirm-fixed-path-clear'])
                argv = execute.call_args.args[1]
                self.assertIn('stage_c_fixed_full.launch', argv)
                self.assertIn('start_observe:=false', argv)
                self.assertIn('start_arm:=true', argv)

    def test_launch_keeps_motion_opt_in_and_driver_failure_fatal(self):
        root = ET.parse(ROOT / 'mission_ws/src/mission/jzzj_grasp/launch/stage_c_fixed_full.launch').getroot()
        args = {a.get('name'): a.get('default') for a in root.findall('arg')}
        self.assertEqual(args['real'], 'false')
        driver = [i for i in root.findall('include') if 'fixed_arm' in i.get('file')][0]
        self.assertIn(('required', 'true'), [(a.get('name'), a.get('value')) for a in driver])
        runner = root.find('node')
        self.assertEqual(runner.get('if'), '$(arg real)')
        self.assertIsNone(runner.get('respawn'))
        self.assertIsNone(runner.get('required'))

    def test_startup_wait_validates_real_feedback(self):
        stamp = types.SimpleNamespace(to_sec=lambda: 10.)
        message = types.SimpleNamespace(name=['joint' + str(i) for i in range(1, 7)],
                                        position=[0.] * 6, header=types.SimpleNamespace(stamp=stamp))
        ros = types.SimpleNamespace(is_shutdown=lambda: False, Time=types.SimpleNamespace(now=lambda: stamp),
                                    wait_for_message=lambda *a, **kw: message,
                                    get_param=lambda *a: {'rgb_info_topic': '/info'})
        dummy = types.SimpleNamespace(JointState=object, CameraInfo=object, TargetObservations=object, GoalStatusArray=object)
        with patch.dict('sys.modules', {k: dummy for k in ('sensor_msgs.msg', 'jzzj_grasp_interfaces.msg', 'actionlib_msgs.msg')}):
            wait_startup(ros, 1., threading.Event())
            cancel = threading.Event(); cancel.set()
            with self.assertRaises(Failure):
                wait_startup(ros, 1., cancel)
            with self.assertRaisesRegex(Failure, 'timeout'):
                wait_startup(ros, 0., threading.Event())


if __name__ == '__main__':
    unittest.main()
