#!/usr/bin/env python3
"""单关节jog标定: 用法 python3 jog_joint.py 关节号 角度增量(度)
读取当前值, 写入当前值+增量(单关节), 800ms到达, 回读验证"""
import sys, time
sys.path.insert(0,'/home/orangepi/jzzj/catkin_ws/src/actuator/arm/scripts')
from checked_arm_sdk import CheckedArmDevice as Arm_Device
from hardware_lease import HardwareLease

jid=int(sys.argv[1]);delta=float(sys.argv[2])
if not 1<=jid<=6:raise SystemExit('关节编号必须为1～6')
lease=HardwareLease('left')
arm=Arm_Device(lr='left')
before=[arm.Arm_serial_servo_read(i+1) for i in range(6)]
if any(v is None for v in before):raise SystemExit('读取失败')
cur=before[jid-1];target=cur+delta
# The shared transport validates the full six-servo packet against 96..4000.
vals=list(before);vals[jid-1]=target
print('jog joint%d: %.1f° -> %.1f° (其余保持)'%(jid,cur,target))
arm.Arm_serial_servo_write6(*vals,800)
time.sleep(1.2)
after=[arm.Arm_serial_servo_read(i+1) for i in range(6)]
print('回读:',after,' joint%d 实际变化: %+.1f°'%(jid,(after[jid-1] or 0)-cur))
