#!/usr/bin/env python3
"""Tag-free visual snapshots and independently ruler-measured +/- body XY calibration."""
import argparse
import datetime
import json
import math
from pathlib import Path
import sys
import time
import uuid
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'mission_ws/src/mission/jzzj_grasp/src'))
from jzzj_grasp.fixed_grasp import validate_reference,calibration_digest,REAR_DEG
from jzzj_grasp.taught import fit_base_matrix
from capture_fixed_grasp import capture


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def samples_from_pairs(pairs,reference,measured_deltas):
    if len(pairs)!=len(measured_deltas):raise ValueError('one independent ruler-measured --body-delta per before/after pair required')
    samples=[]
    for (before,after),delta in zip(pairs,measured_deltas):
        for record in (before,after):
            if (record.get('sample_kind')!='stationary_fruit_no_tag_v2' or
                    record['reference_calibration_sha256']!=reference['calibration_sha256'] or
                    record['identity']!=reference['identity'] or record['arm_frame']!=reference['arm_frame']):
                raise ValueError('sample kind, camera calibration or fruit identity changed')
            q=np.asarray(record['arm_positions_rad'],dtype=float)
            if q.shape!=(6,) or not np.isfinite(q).all() or np.max(np.abs(q-np.radians(REAR_DEG)))>.035:
                raise ValueError('samples require unchanged rear-ready arm pose')
        samples.append(dict(base_delta_m=list(delta),before_m=before['fruit_position_m'],after_m=after['fruit_position_m'],
                            measurement_source='manual_ruler'))
    return samples


def record(reference):
    from jzzj_grasp.execution_lease import ExecutionLease
    lease=ExecutionLease()
    import rospy
    from sensor_msgs.msg import JointState,CameraInfo
    from jzzj_grasp_interfaces.msg import TargetSelection
    rospy.init_node('fixed_base_calibration_sample',anonymous=True)
    joints=[]
    def joint_cb(msg):
        if len(msg.name)!=6 or set(msg.name)!={'joint'+str(i) for i in range(1,7)}:return
        values=dict(zip(msg.name,msg.position));q=[values['joint'+str(i)] for i in range(1,7)]
        stamp=msg.header.stamp.to_sec()
        if all(math.isfinite(v) for v in q) and stamp>0 and -.05<=rospy.Time.now().to_sec()-stamp<=2.:
            joints.append((stamp,q));joints[:]=joints[-100:]
    sub=rospy.Subscriber('/arm_left/joint_states',JointState,joint_cb)
    previous=None
    try:previous=rospy.wait_for_message('/stage_c/selection',TargetSelection,timeout=1.)
    except rospy.ROSException:pass
    pub=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
    session='base-calibration-'+uuid.uuid4().hex;end=time.monotonic()+5.
    while pub.get_num_connections()==0 and time.monotonic()<end:time.sleep(.05)
    if not pub.get_num_connections():raise ValueError('start the fruit observer first')
    try:
        pub.publish(TargetSelection(session_id=session,target_color=reference['target_color'],enabled=True))
        fruit=capture(rospy,reference,session);first,last=fruit['observation_stamps'][0],fruit['observation_stamps'][-1]
        measured=[q for stamp,q in joints if first-1.5<=stamp<=last+.3]
        if not measured or not joints or abs(joints[-1][0]-last)>1.5:raise ValueError('fresh arm feedback absent')
        if any(max(abs(v-math.radians(d)) for v,d in zip(q,REAR_DEG))>.035 for q in measured):
            raise ValueError('keep the arm in the rear-ready pose during calibration')
        p=rospy.get_param('/stage_c_observer/perception');profile=rospy.get_param('/stage_c_observer/execution/profiles/tabletop')
        info=rospy.wait_for_message(p['rgb_info_topic'],CameraInfo,timeout=3.)
        binding=dict(perception=p,tabletop=profile,K=list(info.K),D=list(info.D),
                     image_size=[info.width,info.height],camera_frame=info.header.frame_id)
        if calibration_digest(binding)!=reference['calibration_sha256']:raise ValueError('camera/arm reference calibration changed')
        return dict(sample_kind='stationary_fruit_no_tag_v2',fruit_position_m=fruit['reference_position_m'],
                    arm_positions_rad=list(measured[-1]),identity=fruit['identity'],arm_frame=reference['arm_frame'],
                    reference_calibration_sha256=reference['calibration_sha256'],observation_stamps=fruit['observation_stamps'],
                    maximum_sample_distance_m=fruit['maximum_sample_distance_m'],
                    recorded_at=datetime.datetime.now().astimezone().isoformat())
    finally:
        pub.publish(previous if previous is not None else TargetSelection(session_id=session,target_color=reference['target_color'],enabled=False))
        time.sleep(.1);lease.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=('record','fit'));p.add_argument('--reference',default='.local/fixed_grasp/orange_reference_run.json')
    p.add_argument('--output',required=True);p.add_argument('--pair',nargs=2,action='append',default=[])
    p.add_argument('--body-delta',type=float,nargs=2,action='append',default=[],help='ruler metres in PDF BODY axes: X forward (d+), Y left (cyy-); never camera coordinates; same order as --pair')
    args=p.parse_args();reference=validate_reference(read(args.reference));out=Path(args.output)
    if out.exists():p.error('output exists; preserve it and choose a new path')
    if args.command=='record':result=record(reference)
    else:
        samples=samples_from_pairs([(read(a),read(b)) for a,b in args.pair],reference,args.body_delta)
        matrix,residual=fit_base_matrix(samples,[0,0,1]);result=read(ROOT/'deploy/stage_c_fixed_base.example.yaml')
        result.update(base_matrix=matrix,base_calibration_samples=samples,calibration_residual_m=residual,
                      reference_calibration_sha256=reference['calibration_sha256'])
        print('Measured matrix:',matrix,'maximum residual metres:',residual)
        print('Body axes: X forward=d+, Y left=cyy-. Camera/arm observations keep their original frame; no guessed transform.')
        print('All confirmation flags stay false; review actual ruler measurements and physical assumptions before enabling.')
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False)
    print('Saved:',out.resolve(),'; no actuator command sent; no Tag/map input used')


if __name__=='__main__':main()
