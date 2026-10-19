#!/usr/bin/env python3
"""Explicit upstream/mock handoff client. Never fabricates sensor feedback."""
import argparse
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('takeover','cancel','handoff','recover'))
    p.add_argument('--session',required=True);p.add_argument('--next-owner',default='')
    p.add_argument('--confirm-stopped',action='store_true');p.add_argument('--confirm-empty-and-clear',action='store_true')
    args=p.parse_args()
    if args.action in ('takeover','handoff','recover') and not args.confirm_stopped:p.error('explicit --confirm-stopped required')
    if args.action=='handoff' and not args.next_owner:p.error('handoff requires --next-owner')
    if args.action=='recover' and not args.confirm_empty_and_clear:p.error('recover requires --confirm-empty-and-clear')
    import rospy
    from jzzj_grasp_interfaces.srv import StageCControl,StageCControlRequest
    rospy.init_node('stage_c_control_client',anonymous=True)
    rospy.wait_for_service('/stage_c/handoff/control',timeout=5.)
    req=StageCControlRequest();req.operation={'takeover':0,'cancel':1,'handoff':2,'recover':3}[args.action]
    req.session_id=args.session;req.request_id=time.time_ns();req.issued_at=rospy.Time.now()
    req.expires_at=req.issued_at+rospy.Duration(3.);req.next_owner=args.next_owner
    req.confirmed_stopped=args.confirm_stopped;req.confirmed_empty_and_clear=args.confirm_empty_and_clear
    response=rospy.ServiceProxy('/stage_c/handoff/control',StageCControl)(req)
    print(response)
    if not response.accepted:raise SystemExit(2)


if __name__=='__main__':main()
