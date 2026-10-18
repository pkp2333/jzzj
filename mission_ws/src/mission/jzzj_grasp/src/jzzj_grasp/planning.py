"""Read-only, bounded path validation. Kinematics callbacks never send commands.

Checks the swept TCP against the configured workspace/table, not arbitrary
obstacles or mesh self-collision. Joint escape waypoints require separate field
calibration; no unverified home pose is invented here.
"""
import math
from dataclasses import dataclass
from .core import Failure, finite_vector, distance, dot, add_scaled


def quaternion_angle(a, b):
    return 2*math.acos(min(1., abs(dot(a, b))))


def slerp(a, b, t):
    cosine = dot(a, b)
    if cosine < 0:
        b = tuple(-v for v in b); cosine = -cosine
    if cosine > .9995:
        q = tuple(x+t*(y-x) for x, y in zip(a, b))
        norm = math.sqrt(dot(q, q))
        return tuple(v/norm for v in q)
    angle = math.acos(min(1., cosine))
    return tuple((math.sin((1-t)*angle)*x+math.sin(t*angle)*y)/math.sin(angle)
                 for x, y in zip(a, b))


@dataclass(frozen=True)
class Pose:
    position: tuple
    orientation: tuple


@dataclass(frozen=True)
class Waypoint:
    step: str
    joints: tuple
    floor: float


class PathPlanner:
    def __init__(self, cfg, lower, upper, forward, inverse, guard):
        self.cfg, self.lower, self.upper = cfg, lower, upper
        self.forward, self.inverse, self.guard = forward, inverse, guard
        self.profile = cfg['profiles'][cfg['active_profile']]
        self.normal = self.profile['vertical_axis']
        self.floor = self.profile['table_plane_offset_m']+cfg['safe_tcp_min_height_m']
        self.max_joint_step = cfg.get('planning_joint_step_rad', .35)
        self.max_rotation_step = cfg.get('planning_rotation_step_rad', .1)
        self.count = 0

    def valid_joints(self, q):
        if (not finite_vector(q, 6) or len(self.lower) != 6 or len(self.upper) != 6 or
                any(not lo <= v <= hi for v, lo, hi in zip(q, self.lower, self.upper))):
            raise Failure('NEEDS_REPOSITION', 'path contains invalid/out-of-limit joints')

    def valid_pose(self, pose, floor):
        if (not finite_vector(pose.position, 3) or
                not finite_vector(pose.orientation, 4) or
                abs(dot(pose.orientation, pose.orientation)-1.) > 1e-4 or
                any(not lo <= v <= hi for v, lo, hi in zip(
                    pose.position, self.cfg['workspace_min'], self.cfg['workspace_max'])) or
                dot(pose.position, self.normal) < max(self.floor, floor)):
            raise Failure('NEEDS_REPOSITION', 'swept TCP leaves workspace/table clearance')

    def edge(self, start, end, floor=None):
        """Validate the joint interpolation used by the one-point driver."""
        self.guard(); self.valid_joints(start); self.valid_joints(end)
        if abs(start[5]-end[5]) > 1e-9:
            raise Failure('MOTION_FAILED', 'arm plan must preserve gripper')
        delta = max(abs(a-b) for a, b in zip(start, end))
        if delta > self.max_joint_step+1e-12:
            raise Failure('NEEDS_REPOSITION', 'path joint step exceeds continuity limit')
        floor = self.floor if floor is None else floor
        # <=0.01 rad samples include both endpoints, even for in-place rotation.
        n = max(1, math.ceil(delta/.01))
        previous = None
        for i in range(n+1):
            self.guard()
            q = tuple(a+(b-a)*i/n for a, b in zip(start, end))
            pose = self.forward(q); self.valid_pose(pose, floor)
            if previous and distance(previous.position, pose.position) > self.cfg['max_step_m']:
                raise Failure('NEEDS_REPOSITION', 'swept TCP sampling step too large')
            previous = pose
        return previous

    def append(self, points, start, end, step, floor):
        if self.count >= self.cfg['max_motion_steps']:
            raise Failure('NEEDS_REPOSITION', 'preflight path exceeds motion budget')
        pose = self.edge(start, end, floor)
        points.append(Waypoint(step, tuple(end), floor)); self.count += 1
        return tuple(end), pose

    def segment(self, start, target, orientation, step, floor=None):
        """Interpolate translation AND rotation; solve using the predicted seed."""
        self.guard(); self.valid_joints(start)
        first = self.forward(start)
        desired = Pose(tuple(target), tuple(orientation))
        floor = self.floor if floor is None else floor
        self.valid_pose(first, floor); self.valid_pose(desired, floor)
        n = max(1, math.ceil(distance(first.position, target)/self.cfg['max_step_m']),
                math.ceil(quaternion_angle(first.orientation, orientation)/self.max_rotation_step))
        points = []; q = tuple(start)
        if distance(first.position, target) < 1e-9 and quaternion_angle(first.orientation, orientation) < 1e-9:
            return points, q
        for i in range(1, n+1):
            self.guard()
            position = tuple(a+(b-a)*i/n for a, b in zip(first.position, target))
            rotation = slerp(first.orientation, orientation, i/n)
            try:
                solved = tuple(self.inverse(position, rotation, q))
            except Failure as exc:
                raise Failure(exc.code, 'preflight '+step+': '+str(exc)) from exc
            q, actual = self.append(points, q, solved, step, floor)
            position_error=distance(actual.position,position)
            orientation_error=quaternion_angle(actual.orientation,rotation)
            orientation_limit=self.cfg.get('orientation_tolerance_rad',.08)
            if position_error>self.cfg['position_tolerance'] or orientation_error>orientation_limit:
                raise Failure('NEEDS_REPOSITION',
                    'preflight {}: FK residual position={:.6f}m (limit {:.6f}), rotation={:.6f}rad (limit {:.6f})'.format(
                        step,position_error,self.cfg['position_tolerance'],orientation_error,orientation_limit))
        return points, q

    def plan(self, start, surface, remaining=False):
        """Validate through transport before opening; return approach prefix only."""
        p = self.profile; q = tuple(start); self.valid_joints(q)
        self.valid_pose(self.forward(q), self.floor)
        prefix = []; all_points = []
        def add(target, step, floor=None):
            nonlocal q
            points, q = self.segment(q, target, p['orientation_xyzw'], step, floor)
            all_points.extend(points)
        grasp = tuple(a+b for a, b in zip(surface, p['surface_to_grasp']))
        pre = add_scaled(grasp, self.normal, p['approach_height'])
        if not remaining:
            escapes = p.get('transition_joint_waypoints_rad', [])
            if escapes and not p.get('transition_calibrated', False):
                raise Failure('NOT_READY', 'joint transition is not calibrated')
            for target in escapes:
                if not finite_vector(target, 5):
                    raise Failure('NOT_READY', 'transition requires five finite arm joints')
                target = tuple(target)+(q[5],)
                self.valid_joints(target)
                first = q
                n = max(1, math.ceil(max(abs(a-b) for a, b in zip(first, target))/min(.1,self.max_joint_step)))
                for i in range(1, n+1):
                    end = tuple(a+(b-a)*i/n for a, b in zip(first, target))
                    q, _ = self.append(all_points, q, end, 'PREPARE_TRANSITION', self.floor)
            current = self.forward(q).position
            clearance = max(dot(current, self.normal), dot(pre, self.normal),
                            p['table_plane_offset_m']+p['transit_clearance_height_m'])
            # No lateral sweep below clearance; the raise must not dip below its start.
            add(add_scaled(current, self.normal, clearance-dot(current, self.normal)),
                'RAISE_CLEARANCE', max(self.floor, dot(current, self.normal)-.003))
            add(add_scaled(pre, self.normal, clearance-dot(pre, self.normal)),
                'ALIGN_ABOVE_TARGET', max(self.floor, clearance-.003))
            add(pre, 'SERVO_PREGRASP')
            prefix = list(all_points)
        add(grasp, 'DESCEND')
        lift = add_scaled(grasp, self.normal, p['lift_height'])
        add(lift, 'LIFT')
        transport = tuple(p['transport_position'])
        clearance = max(dot(lift, self.normal), dot(transport, self.normal),
                        p['table_plane_offset_m']+p['transit_clearance_height_m'])
        add(add_scaled(lift, self.normal, clearance-dot(lift, self.normal)), 'RAISE_FOR_TRANSPORT')
        add(add_scaled(transport, self.normal, clearance-dot(transport, self.normal)),
            'MOVE_TO_TRANSPORT_POSE', max(self.floor, clearance-.003))
        add(transport, 'MOVE_TO_TRANSPORT_POSE')
        return prefix, all_points
