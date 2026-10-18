#!/usr/bin/env python3
"""Read-only checks for the canonical real C-stage launch and runtime session."""
import argparse
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def validate_launch_parameters(params):
    arm = '/arm_left/arm_trajectory/'
    for key in ('hardware_enabled', 'calibration_confirmed', 'workspace_clear_confirmed'):
        if params.get(arm + key) is not True:
            raise ValueError('missing true private parameter: ' + arm + key)
    sys.path.insert(0, str(ROOT/'catkin_ws/src/actuator/arm/scripts'))
    from trajectory_safety import validate_mapping
    from servo_protocol import joint_limits
    def config(key):
        if arm + key not in params:
            raise ValueError('missing private parameter: ' + arm + key)
        return params[arm + key]
    signs=config('servo_signs');offsets=config('servo_offsets_deg')
    lower,upper=joint_limits(signs,offsets,config('servo_position_min'),config('servo_position_max'),left=True)
    validate_mapping(signs,offsets,lower,upper)
    if params.get('/stage_c_server/backend') != 'real':
        raise ValueError('real backend required')
    for key in ('real_motion_enabled', 'robot_calibration_confirmed',
                'workspace_clear_confirmed', 'model_validated'):
        if params.get('/stage_c_server/execution/' + key) is not True:
            raise ValueError('missing real execution confirmation: ' + key)
    if params.get('/stage_c_kinematics/calibration_confirmed') is not True:
        raise ValueError('kinematics calibration not confirmed')
    # An accepted IK solution must still satisfy the task's final precision.
    # Tighten the solver/configuration contract, never widen task limits to pass.
    for solver_key,task_key,default in (
            ('max_position_residual_m','position_tolerance',.003),
            ('max_orientation_residual_rad','orientation_tolerance_rad',.08)):
        solver=params.get('/stage_c_kinematics/'+solver_key,default)
        task=params.get('/stage_c_server/execution/'+task_key,default)
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v>0 for v in (solver,task)) or solver>task:
            raise ValueError('kinematics '+solver_key+' must be positive and no wider than execution/'+task_key)
    for path in ('feedback_timeout', 'stop_timeout'):
        value = config(path)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid ' + arm + path)
    arrival = params.get('/stage_c_server/execution/arrival_timeout', 0)
    if not math.isfinite(arrival) or not 1 <= arrival <= 5:
        raise ValueError('arrival_timeout must be 1..5 seconds')
    stop = params.get('/stage_c_server/execution/stop_timeout', 0)
    if not math.isfinite(stop) or stop < .25 + arrival + config('stop_timeout') + config('feedback_timeout'):
        raise ValueError('C stop_timeout must cover hold duration, arrival_timeout, driver stop_timeout and feedback_timeout')


def context_ready(message, session, now, max_age):
    age = now - message.header.stamp.to_sec()
    return (message.session_id == session and message.base_locked and
            message.arm_control_granted and not message.stop_requested and
            -.05 <= age <= max_age)


def configuration_check():
    import yaml
    args = ['roslaunch', '--dump-params', 'jzzj_grasp', 'stage_c.launch',
            'backend:=real', 'start_camera:=false', 'start_detector:=true',
            'start_arm:=true', 'start_kinematics:=true']
    args += [key + ':=' + str(ROOT/'deploy'/name) for key, name in
             (('config','stage_c.yaml'), ('arm_config','arm.yaml'),
              ('kinematics_config','kinematics.yaml'), ('detector_config','detector.yaml'))]
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True, timeout=20)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or 'launch parameter expansion failed')
    validate_launch_parameters(yaml.safe_load(result.stdout))
    print('C launch parameters and private node namespaces: OK')


def runtime_check(session):
    import rospy
    import rosgraph
    from sensor_msgs.msg import JointState
    from jzzj_grasp_interfaces.msg import StageCContext
    rospy.init_node('stage_c_control_check', anonymous=True, disable_signals=True)
    if rospy.get_param('/stage_c_server/backend', '') != 'real':
        raise RuntimeError('real stage C server unavailable')
    if (rospy.get_param('/stage_c_server/stop_failed_latched',True) is not False or
            rospy.get_param('/stage_c_server/holding_latched',True) is not False):
        raise RuntimeError('stage C stop/holding latch active or unknown; operator recovery required')
    pubs, _, services = rosgraph.Master(rospy.get_name()).getSystemState()
    topics = {name: owners for name, owners in pubs}
    owners = topics.get('/arm_left/joint_states', [])
    if owners != ['/arm_left/arm_trajectory']:
        raise RuntimeError('expected one canonical joint-state owner; found ' + repr(owners))
    for topic in ('/arm_left/follow_joint_trajectory/status', '/stage_c/grasp/status'):
        if len(topics.get(topic, [])) != 1:
            raise RuntimeError('missing or duplicate action server: ' + topic)
    if len(dict(services).get('/stage_c/compute_arm_motion', [])) != 1:
        raise RuntimeError('missing or duplicate C kinematics service')
    profile=rospy.get_param('/stage_c_server/execution/active_profile','tabletop')
    if rospy.get_param('/stage_c_server/execution/profiles/'+profile+'/transit_mode','cartesian')=='joint':
        if len(dict(services).get('/stage_c/compute_arm_endpoint',[]))!=1:
            raise RuntimeError('missing or duplicate read-only endpoint kinematics service')
    joints = rospy.wait_for_message('/arm_left/joint_states', JointState, timeout=5)
    names = ['joint'+str(i) for i in range(1,7)]
    if (len(joints.name) != 6 or set(joints.name) != set(names) or
            len(joints.position) != 6 or not all(math.isfinite(v) for v in joints.position)):
        raise RuntimeError('invalid six-joint feedback')
    max_age = rospy.get_param('/stage_c_server/execution/joint_timeout', 2.)
    if not -.05 <= (rospy.Time.now()-joints.header.stamp).to_sec() <= max_age:
        raise RuntimeError('joint feedback is stale')
    if session:
        end = time.monotonic() + 5
        max_age = rospy.get_param('/stage_c_server/execution/context_timeout', 1.)
        while time.monotonic() < end:
            try:
                context = rospy.wait_for_message('/stage_c/context', StageCContext, timeout=1)
            except rospy.ROSException:
                if rospy.is_shutdown():
                    raise
                continue
            if context_ready(context, session, rospy.Time.now().to_sec(), max_age):
                print('C runtime and current handoff session: OK')
                return
        raise RuntimeError('no fresh locked/granted heartbeat for session ' + session)
    print('C runtime interfaces and joint feedback: OK')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ready', action='store_true')
    parser.add_argument('--session')
    args = parser.parse_args()
    if args.ready:
        runtime_check(args.session)
    else:
        configuration_check()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('C control check failed: ' + str(exc), file=sys.stderr)
        sys.exit(1)
