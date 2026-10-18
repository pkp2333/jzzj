"""Control boundary regressions; never imports a real SDK or starts hardware."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
import ast
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'catkin_ws/src/actuator/arm/scripts'))
from checked_arm_sdk import CheckedArmDevice

spec = importlib.util.spec_from_file_location('control_preflight', ROOT/'tools/stage_c/check_control.py')
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class Bus:
    def __init__(self):
        self.fail = False
        self.calls = []
        self.selected = 1

    def write_i2c_block_data(self, *args):
        self.calls.append(args)
        if self.fail:
            raise OSError(121, 'remote I/O error')
        if args[1]==0x37:self.selected=args[2][0]

    def read_i2c_block_data(self,*args):
        if self.fail:raise OSError(121,'remote I/O error')
        return [0,9,105]+[0]*29  # position 2409, approximately 30 degrees for servo 1.


class SwallowingVendor:
    def __init__(self, lr):
        self.bus = Bus()

    def Arm_serial_servo_write6(self, *args):
        try:
            self.bus.write_i2c_block_data(0x10, 0x1d, list(args))
        except OSError:
            pass  # Reproduces the installed SDK's behavior.
        return 2366  # Position encoding, not a success code.

    def Arm_serial_set_torque(self, enabled):
        try:
            self.bus.write_i2c_block_data(0x10, 0x1a, [enabled])
        except OSError:
            pass

    def Arm_serial_servo_read(self, servo_id):
        try:
            self.bus.write_i2c_block_data(0x10, 0x37, [servo_id])
        except OSError:
            return None
        return 30


class SdkBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.sdk = CheckedArmDevice(factory=SwallowingVendor)

    def test_write_exception_is_not_hidden(self):
        self.sdk.bus.bus.fail = True
        with self.assertRaises(OSError):
            self.sdk.Arm_serial_servo_write6(0, 0, 0, 0, 0, 60, 800)

    def test_torque_failure_is_not_hidden(self):
        self.sdk.bus.bus.fail = True
        with self.assertRaises(OSError):
            self.sdk.Arm_serial_set_torque(1)

    def test_invalid_read_is_not_published_as_a_position(self):
        self.sdk.bus.bus.fail = True
        with self.assertRaises(OSError):
            self.sdk.Arm_serial_servo_read(1)

    def test_success_after_previous_failure_does_not_reuse_error(self):
        self.sdk.bus.bus.fail = True
        with self.assertRaises(OSError):
            self.sdk.Arm_serial_servo_read(1)
        self.sdk.bus.bus.fail = False
        self.assertAlmostEqual(self.sdk.Arm_serial_servo_read(1), 30, delta=.05)

    def test_joint_order_and_duration_are_encoded_in_the_expected_registers(self):
        args = (89, 3, 90, 90, 4, 60, 800)
        self.sdk.Arm_serial_servo_write6(*args)
        self.assertEqual(self.sdk.bus.bus.calls,[(0x10,0x1e,[3,32]),
            (0x10,0x1d,[12,62,7,171,3,132,3,132,8,41,9,63])])


class PreflightTests(unittest.TestCase):
    def setUp(self):
        arm = '/arm_left/arm_trajectory/'
        self.params = {arm+k: True for k in ('hardware_enabled','calibration_confirmed','workspace_clear_confirmed')}
        self.params.update({arm+'servo_signs':[1,-1,1,1,1,1], arm+'servo_offsets_deg':[0]*6,
                            arm+'servo_position_min':96, arm+'servo_position_max':4000,
                            arm+'feedback_timeout':1.5, arm+'stop_timeout':5.,
                            '/stage_c_server/backend':'real',
                            '/stage_c_kinematics/calibration_confirmed':True,
                            '/stage_c_server/execution/stop_timeout':12.,
                            '/stage_c_server/execution/arrival_timeout':4.})
        for key in ('real_motion_enabled','robot_calibration_confirmed','workspace_clear_confirmed','model_validated'):
            self.params['/stage_c_server/execution/'+key] = True

    def test_correct_launch_parameters_pass(self):
        preflight.validate_launch_parameters(self.params)

    def test_legacy_node_namespace_is_rejected_before_motion(self):
        p={k.replace('/arm_left/arm_trajectory/', '/arm_left/arm_trajectory_node/'):v for k,v in self.params.items()}
        with self.assertRaisesRegex(ValueError, 'private parameter'):
            preflight.validate_launch_parameters(p)

    def test_launch_driver_backend_and_readiness_share_canonical_name(self):
        launch=ET.parse(ROOT/'mission_ws/src/mission/jzzj_grasp/launch/stage_c.launch')
        node=next(n for n in launch.iter('node') if n.get('type')=='arm_trajectory_node.py')
        prefix='/'+node.get('ns')+'/'+node.get('name')+'/'
        driver=ast.parse((ROOT/'catkin_ws/src/actuator/arm/scripts/arm_trajectory_node.py').read_text(encoding='utf-8'))
        defaults=[n.args[0].value for n in ast.walk(driver) if isinstance(n,ast.Call) and
                  isinstance(n.func,ast.Attribute) and n.func.attr=='init_node']
        self.assertEqual(defaults,[node.get('name')])
        backend=ast.parse((ROOT/'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/backends.py').read_text(encoding='utf-8'))
        limits=[n.args[0].value for n in ast.walk(backend) if isinstance(n,ast.Call) and
                isinstance(n.func,ast.Attribute) and n.func.attr=='get_param' and n.args and
                isinstance(n.args[0],ast.Constant) and 'joint_' in str(n.args[0].value)]
        self.assertEqual(set(limits),{prefix+'joint_lower_rad',prefix+'joint_upper_rad'})
        self.assertIn(prefix+'servo_signs',self.params)

    def test_residual_legacy_parameters_cannot_replace_canonical_parameters(self):
        stale={k.replace('/arm_left/arm_trajectory/','/arm_left/arm_trajectory_node/'):v for k,v in self.params.items()}
        stale['/arm_left/arm_trajectory_node/servo_position_min']=0
        both=dict(stale,**self.params)
        preflight.validate_launch_parameters(both)
        del both['/arm_left/arm_trajectory/servo_signs']
        with self.assertRaisesRegex(ValueError,'servo_signs'):preflight.validate_launch_parameters(both)

    def test_legacy_configuration_without_servo_mapping_is_rejected(self):
        del self.params['/arm_left/arm_trajectory/servo_signs']
        with self.assertRaisesRegex(ValueError, 'servo_signs'):
            preflight.validate_launch_parameters(self.params)

    def test_manager_cannot_time_out_before_driver_stop(self):
        self.params['/stage_c_server/execution/stop_timeout'] = 4.
        with self.assertRaisesRegex(ValueError, 'stop_timeout'):
            preflight.validate_launch_parameters(self.params)

    def test_old_eight_second_budget_cannot_cover_hold_failure_path(self):
        self.params['/stage_c_server/execution/stop_timeout'] = 8.
        with self.assertRaisesRegex(ValueError, 'stop_timeout'):
            preflight.validate_launch_parameters(self.params)

    def test_invalid_position_bounds_are_rejected(self):
        for value in (0,95,4000,96.):
            self.params['/arm_left/arm_trajectory/servo_position_min']=value
            with self.assertRaises(ValueError):preflight.validate_launch_parameters(self.params)

    def test_disabled_execution_blocks_real_launch(self):
        self.params['/stage_c_server/execution/real_motion_enabled'] = False
        with self.assertRaisesRegex(ValueError, 'real_motion_enabled'):
            preflight.validate_launch_parameters(self.params)

    def test_stale_or_old_session_cannot_trigger_goal(self):
        m=types.SimpleNamespace(session_id='old',base_locked=True,arm_control_granted=True,
                                stop_requested=False,header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:100.)))
        self.assertFalse(preflight.context_ready(m,'current',100.,1.))
        m.session_id='current'
        self.assertFalse(preflight.context_ready(m,'current',102.,1.))
        self.assertTrue(preflight.context_ready(m,'current',100.1,1.))
        m.stop_requested=True
        self.assertFalse(preflight.context_ready(m,'current',100.1,1.))


if __name__ == '__main__':
    unittest.main()
