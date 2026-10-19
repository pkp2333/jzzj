"""One fixed legacy-angle grasp, gated by a captured visual reference. No IK."""
import hashlib
import json
import math
from .core import Failure, finite_vector, distance

REAR_DEG=(-90,0,90,90,0,60)
OPEN_DEG=(-1,56,58,50,72,60)
CLOSE_DEG=(-1,56,58,50,72,20)
SIGNS=(1,-1,1,1,1,1)
OFFSETS_DEG=(0,3,0,0,0,0)
DWELL_SECONDS=3.


def calibration_digest(snapshot):
    return hashlib.sha256(json.dumps(snapshot,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_reference(cfg):
    if cfg.get('version')!=1 or cfg.get('angle_convention')!='legacy_arm_test_terminal_degrees':
        raise ValueError('captured legacy-terminal angle reference required')
    for key,expected in (('rear_ready_angles_deg',REAR_DEG),('grasp_open_angles_deg',OPEN_DEG),
                         ('grasp_close_angles_deg',CLOSE_DEG)):
        if tuple(cfg.get(key,()))!=expected:raise ValueError('fixed sequence differs from approved '+key)
    if cfg.get('wait_before_close_seconds')!=DWELL_SECONDS:raise ValueError('fixed dwell must be 3 seconds')
    if not finite_vector(cfg.get('reference_position_m',()),3):raise ValueError('capture a real fruit reference first')
    if cfg.get('arm_frame')!='arm_left/arm_base_link' or cfg.get('reference_frame')!=cfg['arm_frame']:
        raise ValueError('reference must use arm_left/arm_base_link')
    identity=cfg.get('identity',{})
    if not all(k in identity for k in ('class_id','label','color')) or identity['color']!=cfg.get('target_color'):
        raise ValueError('captured class/label/colour identity required')
    if (cfg.get('trigger_radius_m') not in (.005,.020,.050) or cfg.get('stable_seconds')!=1. or
            cfg.get('min_frames')!=3 or cfg.get('max_observation_age_seconds')!=.5):
        raise ValueError('fixed trigger requires 5mm, 20mm or 50mm radius, 1 second, 3 frames and 0.5s freshness')
    if not cfg.get('calibration_snapshot') or calibration_digest(cfg['calibration_snapshot'])!=cfg.get('calibration_sha256'):
        raise ValueError('capture calibration snapshot/hash missing or inconsistent')
    return cfg


class ReferenceGate:
    def __init__(self,cfg,session,after):
        validate_reference(cfg)
        self.cfg,self.session,self.last=cfg,session,after
        self.samples=[];self.track=None

    def add(self,stamp,session,diagnostic,targets,now):
        if session!=self.session or stamp<=self.last:return None
        self.last=stamp
        if not -.05<=now-stamp<=.5:self.samples=[];return None
        if diagnostic!='ok':raise Failure('SENSOR_INVALID','observation: '+diagnostic)
        key=self.cfg['identity'];matches=[]
        for o in targets:
            if (o.valid and o.session==session and o.frame==self.cfg['arm_frame'] and o.stamp==stamp and
                    finite_vector(o.position,3) and .25<=o.quality<=1 and
                    (o.class_id,o.label,o.color)==(key['class_id'],key['label'],key['color'])):
                matches.append(o)
        if len(matches)>1:raise Failure('NO_TARGET','multiple matching fruits; no target switching')
        if not matches:self.samples=[];return None
        o=matches[0]
        if self.track is None:self.track=o.track_id
        elif o.track_id!=self.track:raise Failure('NO_TARGET','target track changed; retry explicitly')
        if distance(o.position,self.cfg['reference_position_m'])>self.cfg['trigger_radius_m']:
            self.samples=[];return None
        if self.samples and (stamp-self.samples[-1][0]>.5 or any(distance(o.position,p)>.005 for _,p in self.samples)):
            self.samples=[]
        self.samples.append((stamp,tuple(o.position)))
        while len(self.samples)>3 and self.samples[1][0]<stamp-1.:self.samples.pop(0)
        if len(self.samples)>=3 and stamp-self.samples[0][0]>=1.:
            return tuple(sum(p[i] for _,p in self.samples)/len(self.samples) for i in range(3))
        return None


class FixedSequence:
    def __init__(self,io,cfg):self.io,self.cfg,self.trace=io,cfg,[]

    def run(self):
        result=dict(status='NOT_READY',motion_complete=False,success=False,trace=self.trace)
        started=False
        try:
            validate_reference(self.cfg)
            self.io.ready()
            self.io.validate_all((REAR_DEG,OPEN_DEG,CLOSE_DEG))
            self.io.guard();started=True
            self.trace.append(dict(step='REAR_READY',angles_deg=list(REAR_DEG)))
            self.io.move(REAR_DEG)
            point=self.io.wait_reference()
            self.trace.append(dict(step='VISUAL_ALIGNED',position_m=list(point)))
            self.io.guard()
            if hasattr(self.io,'prepare_grasp'):self.io.prepare_grasp()
            self.trace.append(dict(step='APPROACH',angles_deg=list(OPEN_DEG)))
            self.io.move(OPEN_DEG)
            self.trace.append(dict(step='DWELL',seconds=DWELL_SECONDS))
            self.io.dwell(DWELL_SECONDS)
            self.io.guard()
            self.trace.append(dict(step='CLOSE',angles_deg=list(CLOSE_DEG)))
            self.io.move(CLOSE_DEG)
            if not self.io.hold():raise Failure('STOP_FAILED','final measured hold failed')
            result.update(status='SUCCESS',success=True,motion_complete=True,stop_confirmed=True)
        except Exception as exc:
            try:stopped=not started or self.io.hold()
            except Exception:stopped=False
            result.update(status=getattr(exc,'code','NOT_READY') if stopped else 'STOP_FAILED',
                          stop_confirmed=stopped,error=str(exc))
        return result
