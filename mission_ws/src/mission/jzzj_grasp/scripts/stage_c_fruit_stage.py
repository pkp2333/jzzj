#!/usr/bin/env python3
"""Wait for stage-C takeover, visually align and execute fixed grasp, then wait for handoff."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time
import uuid
from jzzj_grasp.fixed_grasp import validate_reference
from jzzj_grasp.fixed_base import validate_base


def main():
    import rospy
    from jzzj_grasp.execution_lease import ExecutionLease
    from jzzj_grasp.fruit_stage_ros import FruitStage
    from jzzj_grasp_interfaces.srv import StageCControlRequest
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',required=True);p.add_argument('--base-config',required=True)
    p.add_argument('--real',action='store_true');p.add_argument('--confirm-base-path-clear',action='store_true')
    p.add_argument('--confirm-fixed-path-clear',action='store_true');p.add_argument('--mock-handoff',action='store_true')
    args=p.parse_args(rospy.myargv()[1:])
    reference=validate_reference(json.loads(Path(args.reference).read_text()))
    cfg=validate_base(json.loads(Path(args.base_config).read_text()),reference)
    if not (args.real and args.confirm_base_path_clear and args.confirm_fixed_path_clear and reference.get('real_motion_enabled') is True):
        p.error('real motion and both path confirmations plus private reference enabled flag required')
    rospy.init_node('stage_c_fixed_grasp',disable_signals=True)
    cancel=threading.Event();signal.signal(signal.SIGINT,lambda *a:(cancel.set(),rospy.signal_shutdown('operator stop')))
    lease=ExecutionLease();stage=FruitStage(rospy,reference,cfg,args.reference,cancel)
    rospy.on_shutdown(stage.shutdown)
    if args.mock_handoff:
        req=StageCControlRequest();req.operation=0;req.session_id='fixed-'+uuid.uuid4().hex
        req.request_id=time.time_ns();req.issued_at=rospy.Time.now();req.expires_at=req.issued_at+rospy.Duration(3.)
        req.confirmed_stopped=True
        response=stage.control(req)
        print('Mock upstream TAKEOVER:',response.accepted,response.diagnostic,flush=True)
        if not response.accepted:raise RuntimeError(response.diagnostic)
    try:stage.loop()
    finally:stage.shutdown();lease.close()


if __name__=='__main__':main()
