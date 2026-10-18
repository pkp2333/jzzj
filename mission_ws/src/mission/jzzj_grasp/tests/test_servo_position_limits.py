"""Protocol regressions: a recording fake bus, never real hardware."""
import importlib.util
import ast
import math
import os
from pathlib import Path
import sys
import unittest
import re
import xml.etree.ElementTree as ET
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'catkin_ws/src/actuator/arm/scripts'))
from servo_protocol import angle_to_position,position_to_angle,joint_limits
if os.environ.get('STAGE_C_BOUNDARY_SOURCE'):
    spec=importlib.util.spec_from_file_location('old_boundary',os.environ['STAGE_C_BOUNDARY_SOURCE'])
    boundary=importlib.util.module_from_spec(spec);spec.loader.exec_module(boundary)
else:
    import checked_arm_sdk as boundary


class Bus:
    def __init__(self):
        self.calls=[];self.selected=1;self.positions=[2040,2000,2000,2000,2040,2000]
    def write_i2c_block_data(self,address,register,data):
        self.calls.append((address,register,list(data)))
        if register==0x37:self.selected=data[0]
    def read_i2c_block_data(self,address,register):
        value=self.positions[self.selected-1]
        return [0,(value>>8)&255,value&255]+[0]*29


class ClippingVendor:
    def __init__(self,lr):
        self.left=lr=='left';self.addr=0x10 if self.left else 0x11;self.bus=Bus();self.vendor_writes=0
    def Arm_serial_servo_write6(self,*args):
        self.vendor_writes+=1
        limits=[(-135,135),(-90,90),(-90,90),(-90,90),(-135,135),(0,100)]
        positions=[angle_to_position(i,min(hi,max(lo,v)),self.left)
                   for i,(v,(lo,hi)) in enumerate(zip(args[:6],limits),1)]
        self.bus.write_i2c_block_data(self.addr,0x1e,[args[6]>>8,args[6]&255])
        self.bus.write_i2c_block_data(self.addr,0x1d,[b for p in positions for b in (p>>8,p&255)])
    def Arm_serial_servo_read(self,servo_id):
        return -165 if self.bus.positions[servo_id-1]==0 else position_to_angle(servo_id,self.bus.positions[servo_id-1],self.left)


class ServoPositionTests(unittest.TestCase):
    def setUp(self):
        self.sdk=boundary.CheckedArmDevice(factory=ClippingVendor)
        self.calls=self.sdk.bus.bus.calls
        self.sleep=patch('time.sleep');self.sleep.start();self.addCleanup(self.sleep.stop)
        if hasattr(boundary,'LOGGER'):
            log=patch.object(boundary.LOGGER,'warning');log.start();self.addCleanup(log.stop)
    def test_all_six_positions_accept_both_inclusive_boundaries(self):
        for value in (96,4000):
            self.calls.clear();self.sdk.write_positions([value]*6,800)
            self.assertEqual(self.calls,[(0x10,0x1e,[3,32]),
                (0x10,0x1d,[b for _ in range(6) for b in (value>>8,value&255)])])
    def test_only_bad_servo_is_skipped_and_other_five_execute(self):
        for index in range(6):
            for bad in (0,95,4001,-1,65535,96.,True):
                self.calls.clear();values=[2040]*6;values[index]=bad
                result=self.sdk.write_positions(values,800)
                self.assertFalse(result.complete)
                self.assertEqual(result.skipped_ids,(index+1,))
                self.assertEqual(result.sent_ids,tuple(i for i in range(1,7) if i!=index+1))
                self.assertEqual(self.calls,[(0x10,0x19,[i,7,248,3,32]) for i in result.sent_ids])
    def test_extreme_angle_skips_only_its_servo_without_vendor_clipping(self):
        for index in range(6):
            for bad in (-10000.,10000.,float('nan'),float('inf')):
                self.calls.clear();values=[0.]*6;values[5]=60.;values[index]=bad
                result=self.sdk.Arm_serial_servo_write6(*values,800)
                self.assertEqual(result.skipped_ids,(index+1,))
                self.assertEqual(len(self.calls),5)
                self.assertTrue(all(c[1]==0x19 and c[2][0]!=index+1 for c in self.calls))
    def test_105_degree_joint4_is_encoded_without_90_degree_clipping(self):
        self.sdk.Arm_serial_servo_write6(0,0,0,105,0,60,800)
        data=self.calls[-1][2]
        self.assertEqual((data[6]<<8)|data[7],angle_to_position(4,105,True))
        self.assertNotEqual((data[6]<<8)|data[7],angle_to_position(4,90,True))
        self.assertEqual(self.sdk.device.vendor_writes,0)
    def test_single_servo_uses_same_bounds_and_no_send_on_rejection(self):
        for servo_id in range(1,7):
            self.calls.clear()
            result=self.sdk.Arm_serial_servo_write(servo_id,10000.,800)
            self.assertEqual(result.skipped_ids,(servo_id,));self.assertEqual(result.sent_ids,())
            self.assertEqual(self.calls,[])
            degrees=position_to_angle(servo_id,96,True)
            self.sdk.Arm_serial_servo_write(servo_id,degrees,800)
            self.assertEqual(self.calls[-1],(0x10,0x19,[servo_id,0,96,3,32]))
    def test_feedback_above_receive_limit_is_rejected_before_degree_conversion(self):
        for value in (4097,65535):
            self.sdk.bus.bus.positions[4]=value
            with self.assertRaisesRegex(OSError,'servo 5.*expected 0..4096'):
                self.sdk.Arm_serial_servo_read(5)
        self.assertTrue(all(call[1]==0x37 for call in self.calls))
    def test_feedback_boundaries_and_mapping_round_trip_for_both_arms(self):
        for left in (False,True):
            sdk=boundary.CheckedArmDevice(lr='left' if left else 'right',factory=ClippingVendor)
            for servo_id in range(1,7):
                for value in (0,95,96,1000,2000,4000,4001,4096):
                    sdk.bus.bus.positions[servo_id-1]=value
                    angle=sdk.Arm_serial_servo_read(servo_id)
                    self.assertEqual(angle_to_position(servo_id,angle,left),value)
    def test_receive_range_does_not_follow_narrowed_command_range(self):
        sdk=boundary.CheckedArmDevice(factory=ClippingVendor,position_min=1000,position_max=3000)
        for value in (0,999,3001,4096):
            sdk.bus.bus.positions[4]=value
            self.assertEqual(angle_to_position(5,sdk.Arm_serial_servo_read(5),True),value)
    def test_limits_derived_from_raw_units_and_calibrated_signs(self):
        signs=[1,-1,1,1,1,1];offsets=[0.]*6
        low,high=joint_limits(signs,offsets)
        self.assertGreater(high[3],math.radians(105))
        for i in range(6):
            for q in (low[i],high[i]):
                degrees=math.degrees(q)*signs[i]+offsets[i]
                self.assertIn(angle_to_position(i+1,degrees,True),(96,4000))
    def test_all_urdf_and_gripper_limits_match_deployed_raw_position_mapping(self):
        config=(ROOT/'deploy/arm.yaml').read_text(encoding='utf-8')
        def setting(text,key):
            return ast.literal_eval(re.search(r'^'+key+r':\s*(\[[^\n]+\])',text,re.M).group(1))
        signs=setting(config,'servo_signs');offsets=setting(config,'servo_offsets_deg')
        low,high=joint_limits(signs,offsets)
        robot=ET.parse(ROOT/'catkin_ws/src/actuator/arm/urdf/arm.urdf')
        for i in range(6):
            limits=robot.find("./joint[@name='joint%d']/limit"%(i+1))
            self.assertAlmostEqual(float(limits.get('lower')),low[i],places=12)
            self.assertAlmostEqual(float(limits.get('upper')),high[i],places=12)
        grip=setting((ROOT/'deploy/kinematics.yaml').read_text(encoding='utf-8'),'gripper_limits_rad')
        self.assertEqual(grip,[low[5],high[5]])
    def test_bad_duration_or_partial_batch_sends_nothing(self):
        for duration in (0,-1,65536,800.,True):
            with self.assertRaises(ValueError):self.sdk.write_positions([2040]*6,duration)
        with self.assertRaises(ValueError):self.sdk.write_positions([2040]*5,800)
        self.assertEqual(self.calls,[])
    def test_all_invalid_positions_send_nothing_and_report_all_skipped(self):
        result=self.sdk.write_positions([0,95,4001,-1,None,True],800)
        self.assertEqual(self.calls,[])
        self.assertEqual(result.sent_ids,())
        self.assertEqual(result.skipped_ids,(1,2,3,4,5,6))
    def test_two_invalid_positions_never_receive_a_replacement_target(self):
        result=self.sdk.write_positions([96,95,2000,4001,4000,1000],1200)
        self.assertEqual(result.sent_ids,(1,3,5,6));self.assertEqual(result.skipped_ids,(2,4))
        self.assertEqual(self.calls,[(0x10,0x19,[1,0,96,4,176]),
            (0x10,0x19,[3,7,208,4,176]),(0x10,0x19,[5,15,160,4,176]),
            (0x10,0x19,[6,3,232,4,176])])


if __name__=='__main__':unittest.main()
