#!/usr/bin/env python3
"""Capture a stable fruit reference plus fixed angles. Never command an actuator."""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import uuid


def separation(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def capture(rospy,cfg,session):
    from jzzj_grasp_interfaces.msg import TargetObservations
    after=rospy.Time.now().to_sec();last=after;identity=None;anchor=None;samples=[]
    deadline=time.monotonic()+20.
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        try:frame=rospy.wait_for_message('/stage_c/observations',TargetObservations,timeout=1.)
        except rospy.ROSException:continue
        stamp=frame.header.stamp.to_sec();now=rospy.Time.now().to_sec()
        if frame.session_id!=session or stamp<=last or not -.05<=now-stamp<=cfg['max_observation_age_seconds']:continue
        last=stamp
        if frame.diagnostic!='ok':samples=[];continue
        choices=[o for o in frame.targets if o.valid and o.color==cfg['target_color'] and
                 o.session_id==session and o.header.frame_id==cfg['arm_frame'] and
                 o.header.stamp.to_sec()==stamp and math.isfinite(o.quality) and .25<=o.quality<=1.]
        if len(choices)>1:raise RuntimeError('Multiple same-colour targets; leave one fruit in view and capture again')
        if not choices:samples=[];continue
        o=choices[0];point=(o.position.x,o.position.y,o.position.z);key=(o.class_id,o.label,o.color)
        if not all(math.isfinite(x) for x in point):samples=[];continue
        if identity is None:identity=key;anchor=point
        if key!=identity or separation(anchor,point)>.03:
            raise RuntimeError('Fruit identity/position changed during capture; reference not saved')
        if any(separation(p,point)>min(.005,cfg['trigger_radius_m']) for _,p in samples):samples=[]
        samples.append((stamp,point))
        if len(samples)>=cfg['min_frames'] and stamp-samples[0][0]>=cfg['stable_seconds']:
            centre=[sum(p[i] for _,p in samples)/len(samples) for i in range(3)]
            return dict(reference_position_m=centre,
                        identity=dict(class_id=identity[0],label=identity[1],color=identity[2]),
                        reference_frame=cfg['arm_frame'],capture_session=session,
                        observation_stamps=[s for s,_ in samples],sample_count=len(samples),
                        maximum_sample_distance_m=max(separation(p,centre) for _,p in samples))
    raise RuntimeError('No fresh stable single fruit within 20 seconds; reference not saved')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='deploy/stage_c_fixed_grasp.example.yaml')
    parser.add_argument('--output',help='new JSON file; existing files are never overwritten')
    args=parser.parse_args();cfg=json.loads(Path(args.config).read_text(encoding='utf-8'))
    if cfg.get('real_motion_enabled') is not False:parser.error('capture configuration must keep real_motion_enabled=false')
    for name,size in (('rear_ready_angles_deg',6),('grasp_open_angles_deg',6),('grasp_close_angles_deg',6)):
        if len(cfg.get(name,[]))!=size or not all(type(x) in (int,float) and math.isfinite(x) for x in cfg[name]):
            parser.error('invalid '+name)
    if cfg['grasp_open_angles_deg'][:5]!=cfg['grasp_close_angles_deg'][:5]:parser.error('closing must preserve first five targets')
    out=Path(args.output) if args.output else Path('.local/fixed_grasp')/('orange_reference_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    if out.exists():parser.error('output already exists; preserve it and choose a new path')
    import rospy
    from jzzj_grasp_interfaces.msg import TargetSelection
    from sensor_msgs.msg import CameraInfo
    rospy.init_node('stage_c_capture_fixed_reference',anonymous=True)
    previous=None
    try:previous=rospy.wait_for_message('/stage_c/selection',TargetSelection,timeout=1.)
    except rospy.ROSException:pass
    session='fixed-reference-'+uuid.uuid4().hex
    pub=rospy.Publisher('/stage_c/selection',TargetSelection,queue_size=1,latch=True)
    end=time.monotonic()+5.
    while pub.get_num_connections()==0 and time.monotonic()<end and not rospy.is_shutdown():time.sleep(.05)
    if pub.get_num_connections()==0:raise RuntimeError('observer selection subscriber absent; start the observer first')
    try:
        pub.publish(TargetSelection(session_id=session,target_color=cfg['target_color'],enabled=True))
        record=capture(rospy,cfg,session)
        perception=rospy.get_param('/stage_c_observer/perception')
        profile=rospy.get_param('/stage_c_observer/execution/profiles/tabletop')
        info=rospy.wait_for_message(perception['rgb_info_topic'],CameraInfo,timeout=3.)
        binding=dict(perception=perception,tabletop=profile,K=list(info.K),D=list(info.D),
                     image_size=[info.width,info.height],camera_frame=info.header.frame_id)
        cfg.update(record,captured_at=datetime.datetime.now().astimezone().isoformat(),
                   localization_mode=perception.get('localization_mode','depth'),
                   calibration_snapshot=binding,
                   calibration_sha256=hashlib.sha256(json.dumps(binding,sort_keys=True,allow_nan=False).encode()).hexdigest())
        out.parent.mkdir(parents=True,exist_ok=True)
        with out.open('x',encoding='utf-8') as f:json.dump(cfg,f,indent=2,ensure_ascii=False,allow_nan=False)
        print('REFERENCE_POSITION_M =',cfg['reference_position_m'])
        print('TARGET_IDENTITY =',cfg['identity'])
        print('OPEN_ANGLES_DEG =',cfg['grasp_open_angles_deg'])
        print('CLOSE_ANGLES_DEG =',cfg['grasp_close_angles_deg'])
        print('SAVED =',out.resolve())
        print('No actuator command sent. Real motion remains disabled.')
    finally:
        try:
            pub.publish(previous if previous is not None else TargetSelection(session_id=session,target_color=cfg['target_color'],enabled=False))
            time.sleep(.1)
        except Exception:pass


if __name__=='__main__':main()
