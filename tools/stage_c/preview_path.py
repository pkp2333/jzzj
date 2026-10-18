#!/usr/bin/env python3
"""Read-only C path preview. Never sends an Action goal, heartbeat or selection."""
import argparse
import json
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--surface',type=float,nargs=3,required=True,metavar=('X','Y','Z'))
    parser.add_argument('--profile',default='tabletop')
    args=parser.parse_args()
    import rospy
    from sensor_msgs.msg import JointState
    from jzzj_grasp.backends import RosArmBackend
    from jzzj_grasp.core import validate_config, Failure
    rospy.init_node('stage_c_readonly_path_preview',anonymous=True)
    cfg=rospy.get_param('/stage_c_server/execution');validate_config(cfg)
    if args.profile not in cfg['profiles']:parser.error('unknown grasp profile')
    cfg['active_profile']=args.profile
    backend=RosArmBackend(cfg)
    # Prime the subscriber without publishing anything to the robot.
    sample=rospy.wait_for_message('/arm_left/joint_states',JointState,timeout=5.)
    backend.joint_cb(sample)
    deadline=time.monotonic()+30.
    def guard():
        if rospy.is_shutdown() or time.monotonic()>deadline:
            raise Failure('TIMEOUT','read-only preview deadline exceeded')
        backend.fresh_joints()
    try:
        start=backend.measured()
        prefix,route=backend.planner(guard).plan(start,args.surface)
        print(json.dumps({'valid':True,'read_only':True,'surface_m':args.surface,
                          'start_rad':start,'approach_points':len(prefix),
                          'total_points':len(route),'steps':sorted(set(p.step for p in route)),
                          'scope':'joint limits, IK/FK, swept TCP workspace/table; no mesh/scene collision proof'},indent=2))
        return 0
    except Failure as exc:
        print(json.dumps({'valid':False,'read_only':True,'code':exc.code,'reason':str(exc)},indent=2))
        return 2


if __name__=='__main__':raise SystemExit(main())
