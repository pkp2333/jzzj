#!/usr/bin/env python3
"""Check offline by default; explicitly start one fixed grasp with --real."""
import argparse
import http.client
import json
import os
from pathlib import Path
import socket
import sys
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'mission_ws/src/mission/jzzj_grasp/src'))
from jzzj_grasp.fixed_grasp import validate_reference


class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=3.)


def launch_flags(state,base_adjust=False):
    pubs, subs, services = (dict(part) for part in state)
    nodes = {node for part in state for _, owners in part for node in owners}
    for topic in ('/cmd_vel', '/car_cmd', '/stage_c/base/command', '/car_target'):
        if pubs.get(topic) or subs.get(topic):
            raise RuntimeError('Stop the base controller first: ' + topic)
    if pubs.get('/stage_c/base/status') or pubs.get('/arm_left/arm_joint_states'):
        raise RuntimeError('Stop the legacy arm/base controller first')
    if any('/arm_left/' in n and ('arm_test' in n or 'arm_control' in n) for n in nodes):
        raise RuntimeError('Exit the legacy left-arm debug launch first')
    if any('stage_c_fixed_grasp' in n or n == '/stage_c_server' for n in nodes):
        raise RuntimeError('Another grasp controller is registered; stop it first')
    driver = '/arm_left/arm_trajectory' in nodes
    owners = pubs.get('/arm_left/joint_states', [])
    if owners and owners != ['/arm_left/arm_trajectory']:
        raise RuntimeError('Unexpected left-arm joint publisher: ' + repr(owners))
    detector = '/yolov5_rknn_rc/yolov5_rknn_stream' in nodes
    observer = '/stage_c_observer' in nodes
    if detector != observer:
        raise RuntimeError('Partial vision launch found; stop or complete that launch first')
    flags=dict(start_camera='/ascamera_hp60c' not in nodes,
               start_observe=not observer, start_arm=not driver)
    if base_adjust:
        if '/chassis' in nodes or '/wp_nav_pid_node' in nodes or '/stage_c_fixed_navigation' in nodes:
            raise RuntimeError('Stop the previous chassis/navigation launch before bounded base startup')
        pose_owners=pubs.get('/front_car_pose_orig_map',[])
        if pose_owners and pose_owners!=['/car_localization_node']:
            raise RuntimeError('expected one canonical car_localization_node publisher')
        if not pose_owners and any('localization' in n for n in nodes):
            raise RuntimeError('partial localization graph; inspect it before startup')
        flags['start_localization']=not bool(pose_owners)
    return flags


def device_processes():
    """Catch device owners which have not registered with the ROS master yet."""
    found = []
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            if int(entry.parent.name) == os.getpid():
                continue
            argv = entry.read_bytes().replace(b'\0', b' ').decode(errors='replace')
            needles = ('arm_trajectory_node.py', 'arm_control_node.py', 'arm_test_node.py',
                       'ascamera_node', 'yolov5_rknn_stream', 'stage_c_observer.py', 'rosmaster',
                       'chassis_control_node.py', 'wp_nav_pid_node.py', 'stage_c_fixed_navigation.py',
                       'car_localization_node.py', 'apriltag_localization_node')
            if any(word in argv for word in needles):
                found.append(argv)
        except (OSError, ValueError):
            continue
    return found


def online_plan(base_adjust=False):
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                      transport=TimedTransport())
    caller = '/fixed_grasp_startup_check'
    try:
        code, message, state = master.getSystemState(caller)
    except ConnectionRefusedError:
        if device_processes():
            raise RuntimeError('ROS master unavailable but device/master processes still exist; inspect them first')
        flags=dict(start_camera=True, start_observe=True, start_arm=True)
        if base_adjust:flags['start_localization']=True
        return flags
    except (OSError, socket.timeout, xmlrpc.client.Error) as exc:
        raise RuntimeError('Cannot inspect ROS master: ' + str(exc))
    if code != 1:
        raise RuntimeError('Cannot inspect ROS graph: ' + str(message))
    flags = launch_flags(state,base_adjust)
    for param in ('/stage_c/taught/recovery_required', '/stage_c_server/stop_failed_latched',
                  '/stage_c_server/holding_latched'):
        code, _, value = master.getParam(caller, param)
        if code == 1 and value:
            raise RuntimeError('Recovery required; keep the original session and release it first: ' + param)
    # An unregistered process may already own the device. Never launch a second owner.
    for argv in device_processes():
        if any(x in argv for x in ('chassis_control_node.py','wp_nav_pid_node.py','stage_c_fixed_navigation.py')):
            raise RuntimeError('A chassis/navigation process already exists; inspect it before startup')
        if flags['start_arm'] and any(x in argv for x in ('arm_trajectory_node.py', 'arm_control_node.py', 'arm_test_node.py')):
            raise RuntimeError('An arm process exists outside the expected ROS graph; inspect it first')
        if flags['start_camera'] and 'ascamera_node' in argv:
            raise RuntimeError('A camera process exists outside the expected ROS graph; inspect it first')
        if flags['start_observe'] and any(x in argv for x in ('stage_c_observer.py', 'yolov5_rknn_stream')):
            raise RuntimeError('A vision process exists outside the expected ROS graph; inspect it first')
    return flags


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', default='.local/fixed_grasp/orange_reference_run.json')
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--confirm-base-immobilized', action='store_true')
    parser.add_argument('--base-adjust',action='store_true')
    parser.add_argument('--base-config',default='.local/fixed_grasp/base_navigation.json')
    parser.add_argument('--confirm-base-path-clear',action='store_true')
    parser.add_argument('--confirm-fixed-path-clear', action='store_true')
    args = parser.parse_args(argv)
    reference = Path(args.reference).resolve()
    cfg = json.loads(reference.read_text(encoding='utf-8'))
    validate_reference(cfg)
    base_config=Path(args.base_config).resolve()
    if args.base_adjust:
        from jzzj_grasp.fixed_base import validate_base
        validate_base(json.loads(base_config.read_text(encoding='utf-8')),cfg)
    if not args.real:
        print('Reference valid:', reference)
        print('Offline check only: no ROS nodes started and no actuator commands sent.')
        if args.base_adjust:print('Base adjustment: calibrated map goals <=50mm; no total travel/step limit by default; stop and remeasure; align to <=20mm before grasp.')
        print('Real run: start/reuse camera, detector/observer and left-arm driver; rear -> visual gate -> approach -> wait 3s -> close once.')
        return
    base_gate=args.confirm_base_path_clear if args.base_adjust else args.confirm_base_immobilized
    if not (base_gate and args.confirm_fixed_path_clear):
        parser.error('--real requires --confirm-fixed-path-clear and --confirm-base-path-clear (base adjustment) or --confirm-base-immobilized (arm only)')
    if cfg.get('real_motion_enabled') is not True:
        parser.error('reference real_motion_enabled must be true in the reviewed private run copy')
    flags = online_plan(base_adjust=args.base_adjust)
    print('Startup plan:', ', '.join(k + '=' + ('start' if v else 'reuse') for k, v in flags.items()), flush=True)
    print('One grasp only. Driver errors stop startup. Keep this terminal open; Ctrl+C stops managed nodes and does not unload torque.', flush=True)
    launch = ['roslaunch', 'jzzj_grasp', 'stage_c_fixed_full.launch',
              'reference:=' + str(reference), 'config:=' + str(ROOT / 'deploy/stage_c.yaml'),
              'detector_config:=' + str(ROOT / 'deploy/detector.yaml'),
              'real:=true', 'calibration_confirmed:=true', 'workspace_clear_confirmed:=true',
              'target_color:=' + cfg['identity']['color']]
    if args.base_adjust:launch.extend(['base_adjust:=true','base_config:='+str(base_config)])
    launch.extend(k + ':=' + str(v).lower() for k, v in flags.items())
    os.execvp(launch[0], launch)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, OSError) as exc:
        print('Startup stopped: ' + str(exc), file=sys.stderr)
        raise SystemExit(2)
