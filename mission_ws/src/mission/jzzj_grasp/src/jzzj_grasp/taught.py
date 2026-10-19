"""Single taught point + stop/observe planar alignment. No inverse kinematics."""
import copy
import hashlib
import json
import math
import numpy as np
from .core import Failure, finite_vector, distance

PHASES=('open','approach','descend','close','lift','transport')
DEFAULTS=dict(step_m=.01,speed_mps=.02,total_m=.1,max_steps=15,timeout=60.,
              tolerance_m=.005,stable_seconds=1.,observation_timeout=.5,search_timeout=5.)


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def template_hash(template):
    return digest({k:v for k,v in template.items() if k not in ('empty_validation','notes')})


def plane_basis(normal):
    n=np.asarray(normal,dtype=float)
    if n.shape!=(3,) or not np.isfinite(n).all() or abs(np.linalg.norm(n)-1)>1e-5:
        raise ValueError('unit table normal required')
    axis=np.array([1.,0.,0.]) if abs(n[0])<.9 else np.array([0.,1.,0.])
    u=axis-n*np.dot(axis,n);u/=np.linalg.norm(u)
    return np.column_stack((u,np.cross(n,u)))


def fit_base_matrix(samples,normal):
    """Samples use independently measured base translations in metres."""
    if len(samples)<4:raise ValueError('at least +/- samples for both base axes required')
    basis=plane_basis(normal);commands=[];changes=[]
    for s in samples:
        d=s['base_delta_m'];before=s['before_m'];after=s['after_m']
        if not finite_vector(d,2) or not finite_vector(before,3) or not finite_vector(after,3):
            raise ValueError('invalid calibration sample')
        if not .005<=np.linalg.norm(d)<=.03:raise ValueError('calibration translation must be 5..30mm')
        commands.append(d);changes.append(basis.T@(np.array(after)-before))
    b=np.array(commands);e=np.array(changes)
    if np.linalg.matrix_rank(b)!=2:raise ValueError('both translation axes must be measured')
    for axis in (0,1):
        if not (np.any(b[:,axis]>.003) and np.any(b[:,axis]<-.003)):
            raise ValueError('positive and negative calibration samples required per axis')
    matrix=np.linalg.lstsq(b,e,rcond=None)[0].T
    validate_matrix(matrix)
    residual=float(np.max(np.linalg.norm(e-b@matrix.T,axis=1)))
    if residual>.003:raise ValueError('base calibration residual exceeds 3mm')
    return matrix.tolist(),residual


def validate_matrix(matrix):
    a=np.asarray(matrix,dtype=float)
    if a.shape!=(2,2) or not np.isfinite(a).all():raise ValueError('measured 2x2 base matrix required')
    singular=np.linalg.svd(a,compute_uv=False)
    if min(singular)<.8 or max(singular)>1.2:raise ValueError('base matrix scale/direction calibration invalid')
    return a


def validate_template(t,bindings=None,require_empty=True,arm_only=False):
    """Validate complete data before issuing even a base acquisition command."""
    if t.get('version')!=1 or t.get('units')!='rad':raise ValueError('template version=1 and radians required')
    expected='arm_only' if arm_only else 'base_alignment'
    if t.get('execution_mode','base_alignment')!=expected:raise ValueError('template execution mode mismatch')
    gates=('real_motion_enabled','workspace_clear_confirmed','table_height_confirmed')
    for gate in gates+(() if arm_only else ('base_calibrated',)):
        if t.get(gate) is not True:raise ValueError('template gate not confirmed: '+gate)
    if not t.get('bindings') or (bindings is not None and t['bindings']!=bindings):
        raise ValueError('camera/table/TCP/servo bindings missing or changed; reteach')
    if not t.get('arm_frame'):raise ValueError('arm frame required')
    plane_basis(t['table_normal'])
    if not arm_only:
        if not finite_vector(t.get('reference_position_m',[]),3):raise ValueError('reference fruit coordinate required')
        if not isinstance(t.get('identity'),dict) or not all(k in t['identity'] for k in ('class_id','label','color')):
            raise ValueError('reference target identity required')
        validate_matrix(t.get('base_matrix',[]))
    if not finite_vector(t.get('start_rad',[]),6):raise ValueError('six measured starting joints required')
    steps=t.get('steps',[]);phases=[];previous=t['start_rad']
    if not steps or len(steps)>150:raise ValueError('record a complete bounded taught sequence')
    for step in steps:
        phase=step['phase'];q=step['positions_rad'];seconds=step['seconds']
        if phase not in PHASES or not finite_vector(q,6):raise ValueError('invalid taught phase/joints')
        if isinstance(seconds,bool) or not isinstance(seconds,(int,float)) or not math.isfinite(seconds) or not .2<=seconds<=5:
            raise ValueError('recorded duration must be 0.2..5 seconds')
        if phase in ('open','close'):
            if max(abs(a-b) for a,b in zip(q[:5],previous[:5]))>.01:
                raise ValueError('gripper phase must hold the five arm joints')
        elif abs(q[5]-previous[5])>.01:raise ValueError('arm phase must preserve gripper')
        if not phases or phase!=phases[-1]:phases.append(phase)
        previous=q
    if tuple(phases)!=PHASES:raise ValueError('required phase order: '+','.join(PHASES))
    if sum(s['phase']=='open' for s in steps)!=1 or sum(s['phase']=='close' for s in steps)!=1:
        raise ValueError('exactly one open and one close phase required')
    opened=next(s['positions_rad'][5] for s in steps if s['phase']=='open')
    closed=next(s['positions_rad'][5] for s in steps if s['phase']=='close')
    if abs(opened-closed)<=.035:raise ValueError('distinct open and closed positions required')
    if require_empty and t.get('empty_validation',{}).get('template_hash')!=template_hash(t):
        raise ValueError('successful empty execution of this exact template required')


def split_sequence(template):
    q=tuple(template['start_rad']);result=[]
    for step in template['steps']:
        target=tuple(step['positions_rad']);phase=step['phase']
        if phase in ('open','close'):
            # Measurement jitter is not an instruction to move the arm during grip.
            target=q[:5]+(target[5],);n=1
        else:
            target=target[:5]+(q[5],)
            n=max(1,math.ceil(max(abs(a-b) for a,b in zip(q[:5],target[:5]))/.1))
        start=q
        for i in range(1,n+1):
            q=tuple(a+(b-a)*i/n for a,b in zip(start,target))
            result.append(dict(phase=phase,positions_rad=q,seconds=max(.8,step['seconds']/n)))
        if len(result)>150:raise ValueError('segmented taught sequence exceeds 150 points')
    return result


class Snapshot:
    """A full-frame, timestamped stable window; empty frames invalidate old data."""
    def __init__(self,session,frame,identity,after,anchor=None,stable_seconds=1.,max_age=.5):
        self.session,self.frame,self.identity=session,frame,identity
        self.after,self.anchor,self.span,self.max_age=after,anchor,stable_seconds,max_age
        self.samples=[];self.last=after

    def add(self,stamp,session,diagnostic,targets,now):
        if session!=self.session or stamp<=self.last or not -.05<=now-stamp<=self.max_age:return None
        self.last=stamp
        if diagnostic!='ok':raise Failure('SENSOR_INVALID','observation: '+diagnostic)
        choices=[o for o in targets if o.valid and o.session==session and o.frame==self.frame and
                 o.stamp==stamp and finite_vector(o.position,3) and .25<=o.quality<=1 and
                 (o.class_id,o.label,o.color)==self.identity]
        if len(choices)>1:raise Failure('NO_TARGET','ambiguous same-class/colour targets')
        if not choices:self.samples=[];return None
        point=tuple(choices[0].position)
        if self.anchor is not None and distance(point,self.anchor)>.03:
            raise Failure('NO_TARGET','selected fruit jumped outside association window')
        if self.samples and any(distance(point,p)>.005 for _,p in self.samples):self.samples=[]
        self.samples.append((stamp,point))
        # Retain a bounded one-second window, with a frame spanning its start.
        while len(self.samples)>3 and self.samples[1][0]<stamp-self.span:self.samples.pop(0)
        if len(self.samples)>=3 and stamp-self.samples[0][0]>=self.span:
            return tuple(sum(p[i] for _,p in self.samples)/len(self.samples) for i in range(3))
        return None


class Coordinator:
    """Adapter owns transport and watchdogs. This state machine never calls IK."""
    def __init__(self,adapter,template,now,config=None,arm_only=False):
        self.io,self.template,self.now=adapter,copy.deepcopy(template),now
        self.arm_only=arm_only
        self.cfg=dict(DEFAULTS,**(config or {}));self.trace=[]
        for k,limit in DEFAULTS.items():
            v=self.cfg[k]
            if not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
                raise ValueError('invalid alignment parameter '+k)
            if k in ('step_m','speed_mps','total_m','max_steps','timeout','tolerance_m','observation_timeout') and v>limit:
                raise ValueError('alignment limit may not be increased: '+k)
        if type(self.cfg['max_steps']) is not int:raise ValueError('integer step budget required')
        if self.cfg['stable_seconds']<1.:raise ValueError('stable window must be at least one second')

    def run(self,empty=False):
        if self.arm_only:return self.run_arm_only(empty)
        result={'status':'NOT_READY','motion_complete':False,'human_verdict':'pending','trace':self.trace}
        acquired=False;arm_started=False
        try:
            validate_template(self.template,self.io.bindings(),require_empty=not empty)
            steps=split_sequence(self.template)
            self.io.ready();self.io.wait_arm_still();self.io.check_start(self.template['start_rad'])
            self.io.preflight(steps)  # Full fixed sequence checked before moving the base.
            self.io.acquire_base();acquired=True
            basis=plane_basis(self.template['table_normal']);matrix=validate_matrix(self.template['base_matrix'])
            ref=np.array(self.template['reference_position_m']);anchor=None;total=0.;measured_total=0.
            deadline=self.now()+self.cfg['timeout'];p=None
            if not empty:
                for attempt in range(self.cfg['max_steps']+1):
                    self.io.guard()
                    if self.now()>deadline:raise Failure('TIMEOUT','base alignment deadline')
                    self.io.hold_base()
                    p=np.array(self.io.observe(anchor,deadline));error=basis.T@(p-ref)
                    self.trace.append(dict(step='BASE_REMEASURE',position_m=p.tolist(),error_m=float(np.linalg.norm(error))))
                    if np.linalg.norm(error)<=self.cfg['tolerance_m']:break
                    if attempt>=self.cfg['max_steps']:raise Failure('NEEDS_REPOSITION','base step budget exhausted')
                    correction=np.linalg.solve(matrix,-error);axis=int(np.argmax(np.abs(correction)))
                    delta=float(np.clip(correction[axis],-self.cfg['step_m'],self.cfg['step_m']))
                    if total+abs(delta)>self.cfg['total_m']+1e-12:raise Failure('NEEDS_REPOSITION','base travel budget exhausted')
                    vector=np.zeros(2);vector[axis]=delta
                    predicted=p+basis@(matrix@vector)
                    self.io.pulse(axis,math.copysign(self.cfg['speed_mps'],delta),abs(delta)/self.cfg['speed_mps'])
                    total+=abs(delta);anchor=tuple(predicted)
                    self.io.hold_base();new=np.array(self.io.observe(anchor,deadline))
                    travelled=float(np.linalg.norm(np.linalg.solve(matrix,basis.T@(new-p))))
                    measured_total+=travelled
                    if travelled>.015 or measured_total>self.cfg['total_m']:
                        raise Failure('NEEDS_REPOSITION','measured base displacement exceeds calibrated limits')
                else:raise Failure('NEEDS_REPOSITION','alignment did not converge')
            self.io.lock_base()
            if not empty:
                p=np.array(self.io.observe(tuple(p),deadline))
                if np.linalg.norm(basis.T@(p-ref))>self.cfg['tolerance_m']:
                    raise Failure('NEEDS_REPOSITION','fruit moved after base lock')
            self.io.wait_arm_still();self.io.check_start(self.template['start_rad'])
            for step in steps:
                self.io.guard();arm_started=True
                self.trace.append(dict(step=step['phase'],target_rad=list(step['positions_rad'])))
                self.trace[-1]['arrival']=self.io.execute(step)
            if not self.io.hold_arm():raise Failure('STOP_FAILED','final measured arm hold failed')
            self.io.hold_base(locked=True)
            result.update(status='MOTION_COMPLETE_AWAITING_OPERATOR',motion_complete=True,
                          stop_confirmed=True,base_stop_evidence='serial_zero_and_visual' if not empty else 'serial_zero_operator_empty_trial')
        except Exception as exc:
            try:arm_ok=not arm_started or self.io.hold_arm()
            except Exception:arm_ok=False
            try:base_ok=not acquired or self.io.stop_base()
            except Exception:base_ok=False
            result.update(status=getattr(exc,'code','NOT_READY') if arm_ok and base_ok else 'STOP_FAILED',
                          error=str(exc),stop_confirmed=bool(arm_ok and base_ok))
        try:result['template_hash']=template_hash(self.template)
        except (ValueError,TypeError):result['template_hash']=None
        return result

    def run_arm_only(self,empty=False):
        """Fixed joints only; never acquire, command, or invent a base lock."""
        result=dict(status='NOT_READY',motion_complete=False,human_verdict='pending',
                    execution_mode='arm_only',trace=self.trace,
                    base_stop_evidence='operator_confirmed_physical_immobilization')
        arm_started=False
        try:
            validate_template(self.template,self.io.bindings(),require_empty=not empty,arm_only=True)
            steps=split_sequence(self.template)
            self.io.ready();self.io.wait_arm_still();self.io.check_start(self.template['start_rad'])
            self.io.preflight(steps)
            for step in steps:
                self.io.guard();arm_started=True
                self.trace.append(dict(step=step['phase'],target_rad=list(step['positions_rad'])))
                self.trace[-1]['arrival']=self.io.execute(step)
            if not self.io.hold_arm():raise Failure('STOP_FAILED','final measured arm hold failed')
            result.update(status='MOTION_COMPLETE_AWAITING_OPERATOR',motion_complete=True,stop_confirmed=True)
        except Exception as exc:
            try:arm_ok=not arm_started or self.io.hold_arm()
            except Exception:arm_ok=False
            result.update(status=getattr(exc,'code','NOT_READY') if arm_ok else 'STOP_FAILED',
                          error=str(exc),stop_confirmed=bool(arm_ok))
        try:result['template_hash']=template_hash(self.template)
        except (ValueError,TypeError):result['template_hash']=None
        return result
