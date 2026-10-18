"""Read-only, bounded path validation. Kinematics callbacks never send commands.

Checks the swept TCP against the configured workspace/table, not arbitrary
obstacles or mesh self-collision. Joint escape waypoints require separate field
calibration; no unverified home pose is invented here.
"""
import math
import copy
import time
from dataclasses import dataclass
from .core import Failure, finite_vector, distance, dot, add_scaled


def quaternion_angle(a, b):
    return 2*math.acos(min(1., abs(dot(a, b))))


def tool_x_rotation(q, angle):
    """Postmultiply by local-X rotation; the tool approach axis stays fixed."""
    x,y,z,w=q; s,c=math.sin(angle/2),math.cos(angle/2)
    return (x*c+w*s, y*c+z*s, z*c-y*s, w*c-x*s)


class _SearchDeadline(Exception):
    pass


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
    def __init__(self, cfg, lower, upper, forward, inverse, guard, endpoint_inverse=None):
        self.cfg, self.lower, self.upper = cfg, lower, upper
        self.forward, self.inverse, self.guard = forward, inverse, guard
        self.endpoint_inverse=endpoint_inverse
        self.profile = cfg['profiles'][cfg['active_profile']]
        self.normal = self.profile['vertical_axis']
        self.floor = self.profile['table_plane_offset_m']+cfg['safe_tcp_min_height_m']
        self.max_joint_step = cfg.get('planning_joint_step_rad', .35)
        self.max_rotation_step = cfg.get('planning_rotation_step_rad', .1)
        self.count = 0
        self.selected_orientation = None
        self.orientation_search = None

    def search_grasp_orientation(self, start, surface):
        """Try complete routes, never just endpoint IK. No command callbacks."""
        p=self.profile; samples=p.get('grasp_yaw_samples',12)
        deadline=time.monotonic()+p.get('grasp_yaw_search_timeout',25.)
        attempted=0; rejected=[]; best=None; expired=False
        def guard():
            self.guard()  # Cancellation, feedback and outer deadline take priority.
            if time.monotonic()>deadline:raise _SearchDeadline()
        # Closest to the configured orientation first, then cover a full turn.
        indices=sorted(range(samples),key=lambda i:min(i,samples-i))
        for i in indices:
            try:
                guard(); attempted+=1
                rotation=tool_x_rotation(p['orientation_xyzw'],2*math.pi*i/samples)
                cfg=copy.deepcopy(self.cfg);profile=cfg['profiles'][cfg['active_profile']]
                profile['orientation_xyzw']=list(rotation);profile['_grasp_yaw_selected']=True
                trial=PathPlanner(cfg,self.lower,self.upper,self.forward,self.inverse,guard,self.endpoint_inverse)
                prefix,route=trial.plan(start,surface)
                previous=start;travel=0.;margin=1.
                for point in route:
                    travel+=math.sqrt(sum((a-b)**2 for a,b in zip(previous[:5],point.joints[:5])))
                    margin=min(margin,*(min(v-lo,hi-v)/(hi-lo) for v,lo,hi in
                                      zip(point.joints[:5],self.lower[:5],self.upper[:5])))
                    previous=point.joints
                # Prefer little motion and clearance from hard limits.
                score=travel+.1/(margin+.02)
                if best is None or score<best[0]:best=(score,rotation,prefix,route,travel,margin)
            except _SearchDeadline:
                expired=True;break
            except Failure as exc:
                if exc.code!='NEEDS_REPOSITION':raise
                rejected.append(str(exc))
        self.guard()
        if best is None:
            code='TIMEOUT' if expired else 'NEEDS_REPOSITION'
            raise Failure(code,'downward yaw search: no complete validated route ({}/{} candidates{}); {}'.format(
                attempted,samples,', search budget reached' if expired else '',
                rejected[-1] if rejected else 'no candidate completed'))
        _,rotation,prefix,route,travel,margin=best
        self.selected_orientation=rotation;self.count=len(route)
        self.orientation_search={'attempted':attempted,'samples':samples,'budget_reached':expired,
                                 'orientation_xyzw':list(rotation),'travel_rad':travel,
                                 'minimum_limit_margin_fraction':margin,'route_points':len(route)}
        return prefix,route

    def valid_joints(self, q):
        if (not finite_vector(q, 6) or len(self.lower) != 6 or len(self.upper) != 6 or
                any(not lo <= v <= hi for v, lo, hi in zip(q, self.lower, self.upper))):
            bad = ['j{}={:.3f} not in [{:.3f},{:.3f}]'.format(i+1, v, lo, hi)
                   for i, (v, lo, hi) in enumerate(zip(q, self.lower, self.upper)) if not lo <= v <= hi]
            raise Failure('NEEDS_REPOSITION', 'path contains invalid/out-of-limit joints: ' +
                          ('; '.join(bad) if bad else 'invalid joints/limits q={}'.format(q)))

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
        pending=[(i/n,0) for i in range(n,0,-1)];previous_fraction=0.
        refinement=self.cfg.get('planning_refinements',4)
        while pending:
            fraction,depth=pending.pop()
            self.guard()
            position = tuple(a+(b-a)*fraction for a, b in zip(first.position, target))
            rotation = slerp(first.orientation, orientation, fraction)
            try:
                solved = tuple(self.inverse(position, rotation, q))
                actual=self.edge(q,solved,floor)
                position_error=distance(actual.position,position)
                orientation_error=quaternion_angle(actual.orientation,rotation)
                orientation_limit=self.cfg.get('orientation_tolerance_rad',.08)
                if position_error>self.cfg['position_tolerance'] or orientation_error>orientation_limit:
                    raise Failure('NEEDS_REPOSITION',
                        'FK residual position={:.6f}m (limit {:.6f}), rotation={:.6f}rad (limit {:.6f})'.format(
                            position_error,self.cfg['position_tolerance'],orientation_error,orientation_limit))
                # Refine geometric continuity failures, never motion/stop errors.
            except Failure as exc:
                if exc.code=='NEEDS_REPOSITION' and depth<refinement:
                    pending.append((fraction,depth+1))
                    pending.append(((previous_fraction+fraction)/2,depth+1))
                    continue
                raise Failure(exc.code, 'preflight '+step+': '+str(exc)) from exc
            if self.count>=self.cfg['max_motion_steps']:raise Failure('NEEDS_REPOSITION','preflight path exceeds motion budget')
            points.append(Waypoint(step,tuple(solved),floor));self.count+=1;q=tuple(solved);previous_fraction=fraction
        return points, q

    def joint_transit(self,start,target,orientation,step,floor):
        """Search a distant endpoint, then validate every bounded joint edge.

        Cartesian pose interpolation is unsuitable for changing the full pose of
        a five-DOF arm in place. No endpoint is sent directly to the driver.
        """
        if self.endpoint_inverse is None:raise Failure('NOT_READY','read-only endpoint solver absent')
        self.guard();self.valid_joints(start);self.valid_pose(self.forward(start),floor)
        self.valid_pose(Pose(tuple(target),tuple(orientation)),floor)
        try:end=tuple(self.endpoint_inverse(target,orientation,start))
        except Failure as exc:raise Failure(exc.code,'preflight '+step+' endpoint: '+str(exc)) from exc
        self.valid_joints(end)
        if abs(end[5]-start[5])>1e-9:raise Failure('MOTION_FAILED','transit endpoint changed gripper')
        pose=self.forward(end)
        if (distance(pose.position,target)>self.cfg['position_tolerance'] or
                quaternion_angle(pose.orientation,orientation)>self.cfg.get('orientation_tolerance_rad',.08)):
            raise Failure('NEEDS_REPOSITION','preflight '+step+': endpoint FK residual')
        n=max(1,math.ceil(max(abs(a-b) for a,b in zip(start,end))/min(.1,self.max_joint_step)))
        points=[];q=tuple(start)
        for i in range(1,n+1):
            next_q=tuple(a+(b-a)*i/n for a,b in zip(start,end))
            q,_=self.append(points,q,next_q,step,floor)
        return points,q

    def plan(self, start, surface, remaining=False):
        """Validate through transport before opening; return approach prefix only."""
        if self.profile.get('grasp_yaw_search_enabled',False) and not self.profile.get('_grasp_yaw_selected',False):
            if remaining:raise Failure('NOT_READY','grasp orientation must be selected before descent')
            return self.search_grasp_orientation(start,surface)
        p = self.profile; q = tuple(start); self.valid_joints(q)
        self.valid_pose(self.forward(q), self.floor)
        prefix = []; all_points = []
        def add(target, step, floor=None):
            nonlocal q
            points, q = self.segment(q, target, p['orientation_xyzw'], step, floor)
            all_points.extend(points)
        grasp = tuple(a+b for a, b in zip(surface, p['surface_to_grasp']))
        pre = add_scaled(grasp, self.normal, p['approach_height'])
        joint_mode=p.get('transit_mode','cartesian')=='joint'
        transport_orientation=p.get('transport_orientation_xyzw',p['orientation_xyzw'])
        if joint_mode and not remaining:
            clearance=max(dot(pre,self.normal),p['table_plane_offset_m']+p['transit_clearance_height_m'])
            above=add_scaled(pre,self.normal,clearance-dot(pre,self.normal))
            points,q=self.joint_transit(q,above,p['orientation_xyzw'],'PREPARE_TRANSITION',max(self.floor,clearance-.003))
            all_points.extend(points)
            add(pre,'SERVO_PREGRASP');prefix=list(all_points)
        if not remaining and not joint_mode:
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
        if joint_mode:
            floor=max(self.floor,min(dot(lift,self.normal),dot(transport,self.normal),
                                     p['table_plane_offset_m']+p['transit_clearance_height_m'])-.003)
            points,q=self.joint_transit(q,transport,transport_orientation,'MOVE_TO_TRANSPORT_POSE',floor)
            all_points.extend(points)
            return prefix,all_points
        clearance = max(dot(lift, self.normal), dot(transport, self.normal),
                        p['table_plane_offset_m']+p['transit_clearance_height_m'])
        add(add_scaled(lift, self.normal, clearance-dot(lift, self.normal)), 'RAISE_FOR_TRANSPORT')
        add(add_scaled(transport, self.normal, clearance-dot(transport, self.normal)),
            'MOVE_TO_TRANSPORT_POSE', max(self.floor, clearance-.003))
        add(transport, 'MOVE_TO_TRANSPORT_POSE')
        return prefix, all_points
