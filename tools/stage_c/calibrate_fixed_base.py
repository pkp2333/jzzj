#!/usr/bin/env python3
"""Record stationary samples without actuator commands, or fit four +/- XY pairs offline."""
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
from jzzj_grasp.fixed_base import map_pose,angular
from jzzj_grasp.fixed_grasp import validate_reference,calibration_digest,REAR_DEG
from jzzj_grasp.taught import fit_base_matrix
from capture_fixed_grasp import capture


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def samples_from_pairs(pairs,reference):
    samples=[]
    for before,after in pairs:
        for record in (before,after):
            if (record['reference_calibration_sha256']!=reference['calibration_sha256'] or
                    record['identity']!=reference['identity'] or record['arm_frame']!=reference['arm_frame']):
                raise ValueError('sample/reference calibration or fruit identity changed')
            q=np.asarray(record['arm_positions_rad'],dtype=float)
            if q.shape!=(6,) or not np.isfinite(q).all() or np.max(np.abs(q-np.radians(REAR_DEG)))>.035:
                raise ValueError('calibration samples require unchanged rear-ready arm pose')
        a,b=np.asarray(before['map_pose'],dtype=float),np.asarray(after['map_pose'],dtype=float)
        if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(a).all() or not np.isfinite(b).all():
            raise ValueError('invalid measured map poses')
        if abs(angular(a[2],b[2]))>math.radians(.5):raise ValueError('base rotated during translation sample')
        c,s=math.cos(a[2]),math.sin(a[2]);delta=np.array([[c,s],[-s,c]])@(b[:2]-a[:2])
        samples.append(dict(base_delta_m=delta.tolist(),before_m=before['fruit_position_m'],after_m=after['fruit_position_m']))
    return samples


def record(reference):
    from jzzj_grasp.execution_lease import ExecutionLease
    lease=ExecutionLease()
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState,CameraInfo
    from jzzj_grasp_interfaces.msg import TargetSelection
    rospy.init_node('fixed_base_calibration_sample',anonymous=True)
    poses=[];joints=[]
    def pose_cb(msg):
        try:
            pose,stamp=map_pose(msg,rospy.Time.now().to_sec());poses.append((stamp,pose));poses[:]=poses[-1000:]
        except Exception:pass
    def joint_cb(msg):
        if len(msg.name)!=6 or set(msg.name)!={'joint'+str(i) for i in range(1,7)}:return
        values=dict(zip(msg.name,msg.position));q=[values['joint'+str(i)] for i in range(1,7)]
        if not all(math.isfinite(v) for v in q):return
        stamp=msg.header.stamp.to_sec()
        if stamp>0 and -.05<=rospy.Time.now().to_sec()-stamp<=2.:joints.append((stamp,q));joints[:]=joints[-100:]
    subs=[rospy.Subscriber('/front_car_pose_orig_map',PoseStamped,pose_cb),
          rospy.Subscriber('/arm_left/joint_states',JointState,joint_cb)]
    previous=None
    try:previous=rospy.wait_for_message('/stage_c/selection',TargetSelection,timeout=1.)
    except rospy.ROSException:pass
    pub=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
    session='base-calibration-'+uuid.uuid4().hex;end=time.monotonic()+5.
    while pub.get_num_connections()==0 and time.monotonic()<end:time.sleep(.05)
    if not pub.get_num_connections():raise ValueError('observer absent; start camera and observation launch first')
    try:
        pub.publish(TargetSelection(session_id=session,target_color=reference['target_color'],enabled=True))
        fruit=capture(rospy,reference,session);first,last=fruit['observation_stamps'][0],fruit['observation_stamps'][-1]
        stationary=[(stamp,p) for stamp,p in poses if first-.3<=stamp<=last+.3]
        if len(stationary)<4 or stationary[-1][0]-stationary[0][0]<.8 or abs(stationary[-1][0]-last)>.3:
            raise ValueError('fresh map localization did not cover the sample window')
        origin=stationary[0][1]
        if any(math.dist(p[:2],origin[:2])>.002 or abs(angular(p[2],origin[2]))>math.radians(.5) for _,p in stationary):
            raise ValueError('base was moving during recording; stop and retry')
        measured=[q for stamp,q in joints if first-1.5<=stamp<=last+.3]
        if not measured or not joints or abs(joints[-1][0]-last)>1.5:
            raise ValueError('fresh arm feedback absent')
        if any(max(abs(v-math.radians(d)) for v,d in zip(q,REAR_DEG))>.035 for q in measured):
            raise ValueError('arm must remain in the rear-ready pose during all calibration samples')
        p=rospy.get_param('/stage_c_observer/perception')
        profile=rospy.get_param('/stage_c_observer/execution/profiles/tabletop')
        info=rospy.wait_for_message(p['rgb_info_topic'],CameraInfo,timeout=3.)
        binding=dict(perception=p,tabletop=profile,K=list(info.K),D=list(info.D),
                     image_size=[info.width,info.height],camera_frame=info.header.frame_id)
        if calibration_digest(binding)!=reference['calibration_sha256']:raise ValueError('camera/reference calibration changed')
        return dict(map_pose=list(stationary[-1][1]),fruit_position_m=fruit['reference_position_m'],
                    arm_positions_rad=list(measured[-1]),identity=fruit['identity'],arm_frame=reference['arm_frame'],
                    reference_calibration_sha256=reference['calibration_sha256'],
                    recorded_at=datetime.datetime.now().astimezone().isoformat())
    finally:
        pub.publish(previous if previous is not None else TargetSelection(session_id=session,target_color=reference['target_color'],enabled=False))
        time.sleep(.1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=('record','fit'))
    p.add_argument('--reference',default='.local/fixed_grasp/orange_reference_run.json')
    p.add_argument('--output',required=True)
    p.add_argument('--pair',nargs=2,action='append',default=[])
    args=p.parse_args();reference=validate_reference(read(args.reference));out=Path(args.output)
    if out.exists():p.error('output already exists; choose a new path')
    if args.command=='record':result=record(reference)
    else:
        samples=samples_from_pairs([(read(a),read(b)) for a,b in args.pair],reference)
        matrix,residual=fit_base_matrix(samples,[0,0,1])
        result=read(ROOT/'deploy/stage_c_fixed_base.example.yaml')
        result.update(base_matrix=matrix,base_calibration_samples=samples,calibration_residual_m=residual,
                      reference_calibration_sha256=reference['calibration_sha256'])
        print('Measured matrix:',matrix,'maximum residual metres:',residual)
        print('calibration_confirmed stays false. Review the measured samples before enabling.')
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False)
    print('Saved:',out.resolve(),'; no actuator command sent')


if __name__=='__main__':main()
