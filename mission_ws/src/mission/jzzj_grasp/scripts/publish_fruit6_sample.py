#!/usr/bin/env python3
"""One reference RGB frame on a quarantined probe topic; never a live camera/depth source."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time
from jzzj_grasp.fruit6 import load_manifest,sha256


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-dir',required=True);p.add_argument('--manifest',required=True)
    args,remaps=p.parse_known_args(argv)
    if any(':=' not in r for r in remaps):p.error('only ROS remaps may follow arguments')
    m=load_manifest(args.manifest);image_path=Path(args.model_dir)/'0.jpg'
    if not image_path.is_file() or sha256(image_path)!=m['provenance']['files']['0.jpg']['sha256']:
        raise ValueError('reference image missing or SHA256 mismatch')
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from user_msgs.msg import ObjDets
    rospy.init_node('fruit6_sample',argv=[sys.argv[0]]+remaps)
    rgb_topic='/stage_c/model_probe/sample_rgb';det_topic='/stage_c/model_probe/sample_detections'
    image=cv2.imread(str(image_path))
    if image is None or tuple(image.shape[1::-1])!=(640,480):raise ValueError('invalid reference image')
    # A single latched image is published only after overlay and detector subscribe.
    # The unique frame explicitly forbids treating this sample as a registered RGB-D stream.
    pub=rospy.Publisher(rgb_topic,Image,queue_size=1,latch=True)
    done=threading.Event();result={};stamp=None
    def receive(msg):
        if stamp is not None and msg.header.stamp==stamp and msg.header.frame_id=='fruit6_sample_only':
            result['message']=msg;done.set()
    sub=rospy.Subscriber(det_topic,ObjDets,receive,queue_size=1)
    deadline=time.monotonic()+30.
    while pub.get_num_connections()<2 and time.monotonic()<deadline and not rospy.is_shutdown():
        time.sleep(.05)
    if pub.get_num_connections()<2:raise RuntimeError('start sample probe launch first; detector and overlay must subscribe')
    msg=CvBridge().cv2_to_imgmsg(image,'bgr8');stamp=rospy.Time.now()
    if stamp.to_sec()<=0:raise RuntimeError('valid ROS time required')
    msg.header.stamp=stamp;msg.header.frame_id='fruit6_sample_only';pub.publish(msg)
    if not done.wait(10.):raise RuntimeError('no fresh sample detection result; inspect detector runtime logs')
    det=result['message']
    print(json.dumps({'source':'ONE_REFERENCE_RGB_FRAME; NOT LIVE CAMERA OR GRASP VALIDATION',
        'count':len(det.dets),'labels':[d.label for d in det.dets],
        'class_ids':[d.class_id for d in det.dets],'scores':[d.score for d in det.dets]},indent=2))
    sub.unregister()
    return 0

if __name__=='__main__':
    sys.exit(main())
