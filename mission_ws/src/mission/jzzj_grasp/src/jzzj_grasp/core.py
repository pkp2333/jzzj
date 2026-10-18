"""ROS-free, fail-closed stage C orchestration. Backends never own the base."""
import math
import threading
import time
from jzzj_visual_servo.servo import limited_target
from dataclasses import dataclass

CODES = dict(SUCCESS=0, NOT_READY=1, NO_TARGET=2, NEEDS_REPOSITION=3,
             SENSOR_INVALID=4, MOTION_FAILED=5, GRASP_UNCONFIRMED=6,
             TIMEOUT=7, CANCELED=8, STOP_FAILED=9)

class Failure(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code

@dataclass(frozen=True)
class Context:
    stamp: float
    session: str
    locked: bool
    granted: bool
    stop: bool = False
    received: float = 0.0

@dataclass(frozen=True)
class Observation:
    stamp: float
    session: str
    frame: str
    position: tuple
    color: str
    label: str = 'object'
    class_id: int = 0
    track_id: int = 0
    quality: float = 1.0
    valid: bool = True
    roi: tuple = (0,0,0,0)

@dataclass
class Outcome:
    code: str
    message: str
    grasp_confirmed: bool = False
    holding_known: bool = False
    holding_object: bool = False
    ready_for_transport: bool = False
    stop_confirmed: bool = False

def finite_vector(values, size):
    return len(values) == size and all(math.isfinite(v) for v in values)

def dot(a,b):
    return sum(x*y for x,y in zip(a,b))

def add_scaled(a,b,scale):
    return tuple(x+scale*y for x,y in zip(a,b))

def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def step_toward(current, target, limit):
    delta = distance(current, target)
    if not math.isfinite(limit) or limit <= 0 or not finite_vector(current,3) or not finite_vector(target,3):
        raise Failure('MOTION_FAILED', 'invalid Cartesian step')
    return limited_target(current,target,limit)


def validate_config(cfg):
    positive=('context_timeout','observation_timeout','joint_timeout','search_timeout','max_goal_seconds',
              'ik_timeout','stop_timeout','motion_seconds','arrival_timeout','max_step_m','position_tolerance',
              'association_radius','verify_radius','min_observed_lift','max_final_descent')
    if any(not math.isfinite(cfg[k]) or cfg[k]<=0 for k in positive):
        raise ValueError('positive finite timing/geometry parameters required')
    for k in ('stable_frames','verification_frames','max_motion_steps'):
        if type(cfg[k]) is not int or cfg[k]<1:raise ValueError('positive integer budgets required')
    if not 0<cfg['min_quality']<=1:raise ValueError('invalid quality threshold')
    if cfg.get('verification_mode','lift') not in ('lift','disappearance'):raise ValueError('invalid verification_mode')
    if 'gripper_closed_tolerance_rad' in cfg and (not math.isfinite(cfg['gripper_closed_tolerance_rad']) or cfg['gripper_closed_tolerance_rad']<=0):
        raise ValueError('invalid gripper tolerance')
    if not math.isfinite(cfg.get('orientation_tolerance_rad',.08)) or cfg.get('orientation_tolerance_rad',.08)<=0:
        raise ValueError('positive finite orientation_tolerance_rad required')
    if not finite_vector(cfg['workspace_min'],3) or not finite_vector(cfg['workspace_max'],3):raise ValueError('invalid workspace')
    if any(a>=b for a,b in zip(cfg['workspace_min'],cfg['workspace_max'])):raise ValueError('empty workspace')
    if not math.isfinite(cfg['safe_tcp_min_height_m']) or cfg['safe_tcp_min_height_m']<0:raise ValueError('invalid table clearance')
    if len(cfg['joint_names'])!=6 or len(set(cfg['joint_names']))!=6:raise ValueError('invalid joint mapping')
    for key,default,upper in (('planning_joint_step_rad',.35,.35),('planning_rotation_step_rad',.1,.1)):
        value=cfg.get(key,default)
        if not math.isfinite(value) or not 0<value<=upper:raise ValueError('invalid '+key)
    for p in cfg['profiles'].values():
        escapes=p.get('transition_joint_waypoints_rad',[])
        if not isinstance(escapes,list) or len(escapes)>cfg['max_motion_steps'] or any(not finite_vector(q,5) for q in escapes):
            raise ValueError('transition requires a bounded list of five-joint waypoints')
        if escapes and p.get('transition_calibrated') is not True:
            raise ValueError('joint transition requires explicit calibration')
        if not finite_vector(p['orientation_xyzw'],4) or abs(sum(v*v for v in p['orientation_xyzw'])-1)>1e-4:
            raise ValueError('unit quaternion required')
        for k in ('surface_to_grasp','transport_position'):
            if not finite_vector(p[k],3):raise ValueError('invalid profile position')
        for k in ('approach_height','lift_height'):
            if not math.isfinite(p[k]) or p[k]<=0:raise ValueError('positive profile distances required')
        if p['approach_height']>cfg['max_final_descent']:raise ValueError('unbounded final descent')
        if not all(math.isfinite(p[k]) for k in ('open_rad','closed_rad','transit_clearance_height_m')):raise ValueError('nonfinite profile')
        if p['open_rad']==p['closed_rad']:raise ValueError('distinct gripper setpoints required')
        if not finite_vector(p['vertical_axis'],3) or abs(dot(p['vertical_axis'],p['vertical_axis'])-1)>1e-5:raise ValueError('unit upward table normal required')
        if not all(math.isfinite(p[k]) for k in ('table_plane_offset_m','min_object_height_m','max_object_height_m')) or not 0<=p['min_object_height_m']<p['max_object_height_m']:
        # min=0 is legal: plane-mode observations sit exactly on the calibrated plane.
            raise ValueError('explicit table plane and object-height band required')

class Store:
    def __init__(self):
        self.lock = threading.RLock()
        self.context = None
        self.observations = []
        self.diagnostic = 'no observations'

    def get(self):
        with self.lock:
            return self.context, list(self.observations), self.diagnostic

class Engine:
    """Serial observe/move loop; ROS callbacks only update Store/cancel Event."""
    def __init__(self, backend, store, config, now=time.time, monotonic=time.monotonic,
                 sleep=time.sleep, feedback=lambda *args: None):
        validate_config(config)
        self.backend, self.store, self.cfg = backend, store, config
        self.verification_mode = config.get('verification_mode', 'lift')
        self.now, self.clock, self.sleep, self.feedback = now, monotonic, sleep, feedback
        self.cancel = threading.Event()
        self.session = None
        self.closed = False
        self.verified = False
        self.last_stamp = 0.0

    def guard(self):
        if self.cancel.is_set():
            raise Failure('CANCELED', 'cancel requested')
        if self.clock() >= self.deadline:
            raise Failure('TIMEOUT', 'stage deadline exceeded')
        c, _, _ = self.store.get()
        if (c is None or c.session != self.session or not c.locked or not c.granted
                or c.stop or not math.isfinite(c.stamp) or c.stamp <= 0
                or not -0.05 <= self.now()-c.stamp <= self.cfg['context_timeout']
                or not 0 <= self.clock()-c.received <= self.cfg['context_timeout']):
            raise Failure('NOT_READY', 'base/arm handoff absent, stale, stopped or wrong session')

    def report(self, step, target=None, error=0.0, diagnostic=''):
        self.feedback(step, target, error, diagnostic)

    def valid(self, o):
        return (o.valid and o.session == self.session and o.color == self.color
                and o.frame == self.cfg['arm_frame'] and finite_vector(o.position,3)
                and math.isfinite(o.quality) and o.quality >= self.cfg['min_quality']
                and math.isfinite(o.stamp) and o.stamp > self.last_stamp
                and -0.05 <= self.now()-o.stamp <= self.cfg['observation_timeout'])

    def reachable(self, point):
        return all(lo <= v <= hi for v,lo,hi in zip(point,self.cfg['workspace_min'],self.cfg['workspace_max']))

    def observe(self, anchor=None, identity=None, verification=False, after=0.0):
        end = min(self.deadline, self.clock()+self.cfg['search_timeout'])
        saw_unreachable = False
        while self.clock() < end:
            self.guard()
            _, candidates, diagnostic = self.store.get()
            choices = [o for o in candidates if self.valid(o) and o.stamp > after]
            if identity:
                choices = [o for o in choices if (o.class_id,o.label,o.color)==identity]
            if anchor is not None:
                choices = [o for o in choices if distance(o.position,anchor) <= self.cfg['verify_radius' if verification else 'association_radius']]
                # Ambiguous same-colour/class candidates are not positive evidence.
                if len(choices) > 1:
                    choices = []
            feasible = [o for o in choices if self.reachable(o.position)]
            saw_unreachable |= bool(choices and not feasible)
            if feasible:
                tcp = self.backend.tcp(self.guard)
                feasible.sort(key=lambda o:(distance(o.position,anchor or tcp),-o.quality))
                chosen = feasible[0]
                self.last_stamp = chosen.stamp
                self.backend.selected(chosen)
                return chosen
            self.report('VERIFY_GRASP' if verification else 'SEARCH_TARGET', diagnostic=diagnostic)
            self.sleep(0.02)
        self.guard()
        if verification:
            raise Failure('GRASP_UNCONFIRMED', 'no fresh unambiguous target at predicted held position')
        if saw_unreachable:
            raise Failure('NEEDS_REPOSITION','detected target outside calibrated arm workspace')
        if diagnostic and diagnostic != 'ok':
            raise Failure('SENSOR_INVALID', diagnostic)
        raise Failure('NO_TARGET','no fresh matching target')

    def move(self, point, step):
        self.guard()
        if not self.reachable(point) or dot(point,self.profile['vertical_axis'])-self.profile['table_plane_offset_m'] < self.cfg['safe_tcp_min_height_m']:
            raise Failure('NEEDS_REPOSITION','TCP path outside safe workspace')
        self.report(step,point,distance(self.backend.tcp(self.guard),point))
        self.backend.move(point,self.profile['orientation_xyzw'],self.guard)
        self.guard()

    def path(self,target,step):
        for _ in range(self.cfg['max_motion_steps']):
            current=self.backend.tcp(self.guard)
            if distance(current,target) <= self.cfg['position_tolerance']:
                return
            self.move(step_toward(current,target,self.cfg['max_step_m']),step)
        raise Failure('MOTION_FAILED','Cartesian path step budget exceeded')

    def verify(self, identity, contact_anchor, lift_position):
        if dot(tuple(a-b for a,b in zip(lift_position,contact_anchor)),self.profile['vertical_axis']) < self.cfg['min_observed_lift']:
            raise Failure('GRASP_UNCONFIRMED','insufficient physical lift to verify')
        # The object surface offset relative to TCP was measured at closing.
        expected = tuple(v+d for v,d in zip(lift_position,self.surface_offset))
        after=self.now()  # Never accept pre-motion/replayed detections as held-object evidence.
        for _ in range(self.cfg['verification_frames']):
            o=self.observe(expected,identity,True,after)
            if dot(tuple(a-b for a,b in zip(o.position,self.original_surface)),self.profile['vertical_axis']) < self.cfg['min_observed_lift']:
                raise Failure('GRASP_UNCONFIRMED','object did not rise from original surface')
        self.verified=True

    def verify_disappearance(self,identity,site):
        """Depth-free grasp evidence: a successful close plus the target leaving
        the grasp site for verification_frames fresh, healthy observation frames.
        Anything ambiguous stays GRASP_UNCONFIRMED/SENSOR_INVALID - never success."""
        if not self.closed:raise Failure('GRASP_UNCONFIRMED','close never completed')
        checked=self.now()
        frames=0
        while frames<self.cfg['verification_frames']:
            self.guard()
            _,candidates,diagnostic=self.store.get()
            if diagnostic and diagnostic!='ok':raise Failure('SENSOR_INVALID',diagnostic)
            fresh=[o for o in candidates if math.isfinite(o.stamp) and o.stamp>checked and self.valid(o)]
            if not fresh:
                self.report('VERIFY_GRASP',site,0.,'awaiting fresh post-close observations')
                self.sleep(0.02);continue
            checked=max(o.stamp for o in candidates if math.isfinite(o.stamp) and o.stamp>checked)
            if any((o.class_id,o.label,o.color)==identity and distance(o.position,site)<=self.cfg['association_radius'] for o in fresh):
                raise Failure('GRASP_UNCONFIRMED','target still at grasp site')
            frames+=1

    def execute(self, session, color, profile, timeout):
        self.session,self.color=session,color
        self.closed=self.verified=False
        self.last_stamp=0.0
        self.deadline=self.clock()+min(timeout,self.cfg['max_goal_seconds'])
        try:
            if not session or color not in self.cfg['supported_colors'] or not math.isfinite(timeout) or timeout <= 0:
                raise Failure('NOT_READY','session, colour and finite positive timeout required')
            if profile not in self.cfg['profiles']:
                raise Failure('NOT_READY','unknown grasp profile')
            self.profile=self.cfg['profiles'][profile]
            self.guard()
            self.backend.ready(self.guard)
            self.report('CHECK_READY')
            # Require a calibrated, fresh target before issuing even an opening motion.
            o=self.observe()
            if not self.profile['min_object_height_m']<=dot(o.position,self.profile['vertical_axis'])-self.profile['table_plane_offset_m']<=self.profile['max_object_height_m']:
                raise Failure('NEEDS_REPOSITION','target surface outside calibrated tabletop height band')
            self.report('PREFLIGHT_PATH',o.position)
            self.backend.preflight(o.position,self.profile,self.guard)
            self.report('PREPARE')
            self.backend.grip(self.profile['open_rad'],self.guard)
            o=self.observe(o.position,(o.class_id,o.label,o.color),after=self.now())
            identity=(o.class_id,o.label,o.color)
            height=dot(o.position,self.profile['vertical_axis'])-self.profile['table_plane_offset_m']
            if not self.profile['min_object_height_m']<=height<=self.profile['max_object_height_m']:
                raise Failure('NEEDS_REPOSITION','target surface outside calibrated tabletop height band')
            self.backend.prepare(o.position,self.profile,self.guard,
                                 lambda step,point:self.report(step,point))
            self.original_surface=o.position
            stable=0
            self.report('SERVO_PREGRASP',o.position)
            for _ in range(self.cfg['max_motion_steps']):
                grasp=tuple(a+b for a,b in zip(o.position,self.profile['surface_to_grasp']))
                normal=self.profile['vertical_axis']
                pre=add_scaled(grasp,normal,self.profile['approach_height'])
                tcp=self.backend.tcp(self.guard)
                if distance(tcp,pre) <= self.cfg['position_tolerance']:
                    stable+=1
                    if stable >= self.cfg['stable_frames']:
                        break
                else:
                    stable=0
                    # Raise before lateral motion; no diagonal sweep across table/objects.
                    tcp_height=dot(tcp,normal);pre_height=dot(pre,normal)
                    clearance=max(tcp_height,pre_height,self.profile['transit_clearance_height_m']+self.profile['table_plane_offset_m'])
                    difference=tuple(a-b for a,b in zip(pre,tcp))
                    lateral=distance(difference,tuple(dot(difference,normal)*v for v in normal))
                    if lateral > self.cfg['position_tolerance']:
                        target=add_scaled(tcp,normal,clearance-tcp_height) if tcp_height < clearance-self.cfg['position_tolerance'] else add_scaled(pre,normal,clearance-pre_height)
                    else:
                        target=pre
                    self.move(step_toward(tcp,target,self.cfg['max_step_m']),'SERVO_PREGRASP')
                # Post-arrival sample, not a cached frame captured during motion.
                o=self.observe(o.position,identity,after=self.now())
            else:
                raise Failure('MOTION_FAILED','visual convergence step budget exceeded')
            self.original_surface=o.position
            grasp=tuple(a+b for a,b in zip(o.position,self.profile['surface_to_grasp']))
            self.guard()
            # The selected fruit may have moved during visual alignment. Recheck
            # descent/lift/transport before entering the final occluded segment.
            self.backend.preflight(o.position,self.profile,self.guard,remaining=True)
            # Validated final short descent tolerates expected gripper occlusion.
            if self.profile['approach_height'] > self.cfg['max_final_descent']:
                raise Failure('NOT_READY','final descent exceeds calibrated occlusion segment')
            self.path(grasp,'DESCEND')
            contact=self.backend.tcp(self.guard)
            self.surface_offset=tuple(a-b for a,b in zip(o.position,contact))
            self.report('CLOSE_GRIPPER')
            self.closed=True  # From this point an interrupted close means unknown holding state.
            self.backend.grip(self.profile['closed_rad'],self.guard)
            lift=add_scaled(contact,self.profile['vertical_axis'],self.profile['lift_height'])
            self.path(lift,'LIFT')
            self.report('VERIFY_GRASP')
            if self.verification_mode=='disappearance':
                self.verify_disappearance(identity,self.original_surface)
            else:
                lift=self.backend.tcp(self.guard)
                self.verify(identity,contact,lift)
            transport=self.profile['transport_position']
            normal=self.profile['vertical_axis']
            clearance=max(dot(lift,normal),dot(transport,normal),
                          self.profile['table_plane_offset_m']+self.profile['transit_clearance_height_m'])
            self.path(add_scaled(lift,normal,clearance-dot(lift,normal)),'MOVE_TO_TRANSPORT_POSE')
            self.path(add_scaled(transport,normal,clearance-dot(transport,normal)),'MOVE_TO_TRANSPORT_POSE')
            self.path(transport,'MOVE_TO_TRANSPORT_POSE')
            if self.verification_mode=='disappearance':
                self.guard()
                self.report('SUCCESS')
                return Outcome('SUCCESS','grasp site clear; object at transport pose',True,True,True,True,True)
            self.verified=False
            self.verify(identity,contact,self.backend.tcp(self.guard))
            self.guard()
            self.report('SUCCESS')
            return Outcome('SUCCESS','object verified at transport pose',True,True,True,True,True)
        except Failure as exc:
            try:
                stopped=self.backend.hold(self.cfg['stop_timeout'])
            except Exception:
                stopped=False
            message=str(exc)
            if not stopped and getattr(self.backend,'stop_diagnostic',''):
                message+='; stop: '+self.backend.stop_diagnostic
            return Outcome(exc.code if stopped else 'STOP_FAILED',message,False,
                           False,False,False,stopped)
        except Exception as exc:
            try:
                stopped=self.backend.hold(self.cfg['stop_timeout'])
            except Exception:
                stopped=False
            message=str(exc)
            if not stopped and getattr(self.backend,'stop_diagnostic',''):
                message+='; stop: '+self.backend.stop_diagnostic
            return Outcome('MOTION_FAILED' if stopped else 'STOP_FAILED',message,
                           stop_confirmed=stopped)
