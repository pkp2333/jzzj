"""Tag-free fruit-relative translation feedback. No map pose or fake odometry."""
import math
import numpy as np
from .core import Failure,finite_vector
from .taught import validate_matrix,fit_base_matrix


def validate_base(cfg,reference=None):
    if cfg.get('version')!=2:
        raise ValueError('version 2 fruit-relative base calibration required; old map calibration is not reusable')
    if (cfg.get('body_axes')!='x_forward_y_left' or
            cfg.get('drive_interface')!='chassis_calib_d_forward_cyy_right_v1'):
        raise ValueError('ruler samples must use PDF body X-forward/Y-left and the d-forward/cyy-right adapter; refit existing snapshots with actual signed body displacements')
    for key in ('calibration_confirmed','stationary_fruit_confirmed','rigid_camera_confirmed','translation_only_confirmed'):
        if cfg.get(key) is not True:raise ValueError('unconfirmed visual-base condition: '+key)
    matrix=validate_matrix(cfg.get('base_matrix'))
    samples=cfg.get('base_calibration_samples',[])
    if any(s.get('measurement_source')!='manual_ruler' for s in samples):
        raise ValueError('independently measured ruler translation samples required')
    fitted,_=fit_base_matrix(samples,[0.,0.,1.])
    if not np.allclose(matrix,fitted,rtol=0,atol=1e-6):raise ValueError('matrix differs from measured samples')
    for key,upper in (('alignment_tolerance_m',.020),('step_m',.050),('total_m',math.inf),
                      ('max_steps',math.inf),('timeout_seconds',math.inf)):
        value=cfg.get(key)
        if key in ('total_m','max_steps','timeout_seconds') and value is None:continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=upper:
            raise ValueError('invalid base limit: '+key)
    if cfg['max_steps'] is not None and type(cfg['max_steps']) is not int:raise ValueError('max_steps must be integer/null')
    if cfg.get('feedback_topic')!='/stage_c/base/visual_feedback' or cfg.get('target_frame')!='stage_c_body':
        raise ValueError('explicit stage-C visual feedback and relative body frame required')
    if reference is not None and cfg.get('reference_calibration_sha256')!=reference['calibration_sha256']:
        raise ValueError('camera/arm calibration changed since ruler calibration')
    return cfg


def correction(cfg,point,reference):
    if not finite_vector(point,3) or not finite_vector(reference,3):raise Failure('SENSOR_INVALID','invalid fruit position')
    if abs(point[2]-reference[2])>.005:raise Failure('NEEDS_REPOSITION','translation cannot correct target height')
    if math.dist(point,reference)<=cfg['alignment_tolerance_m']:return None
    delta=np.linalg.solve(validate_matrix(cfg['base_matrix']),np.asarray(reference[:2])-point[:2])
    delta*=min(1.,cfg['step_m']/np.linalg.norm(delta))
    return tuple(float(v) for v in delta)


class FruitTranslation:
    """Observable XY translation under confirmed fixed-fruit/no-turn assumptions."""
    def __init__(self,cfg,reference,session,after):
        self.cfg,self.reference,self.session=cfg,reference,session
        self.last=after;self.anchor=None;self.track=None;self.position=(0.,0.)

    def add(self,stamp,session,diagnostic,targets,now):
        if session!=self.session or stamp<=self.last:return None
        previous=self.last;self.last=stamp
        if stamp<=0 or not -.05<=now-stamp<=.5:raise Failure('SENSOR_INVALID','fruit feedback stale')
        if diagnostic!='ok':raise Failure('SENSOR_INVALID','fruit observation: '+diagnostic)
        identity=self.reference['identity']
        choices=[o for o in targets if o.valid and o.session==session and o.stamp==stamp and
                 o.frame==self.reference['arm_frame'] and finite_vector(o.position,3) and .25<=o.quality<=1. and
                 (o.class_id,o.label,o.color)==(identity['class_id'],identity['label'],identity['color'])]
        if not choices and self.anchor is None:return None
        if len(choices)!=1:raise Failure('NO_TARGET','one unambiguous stationary fruit required')
        o=choices[0]
        if self.track is None:self.track=o.track_id
        if o.track_id!=self.track:raise Failure('NO_TARGET','fruit track changed')
        if self.anchor is None:self.anchor=tuple(o.position)
        if abs(o.position[2]-self.anchor[2])>.005:raise Failure('SENSOR_INVALID','fruit/camera height changed')
        position=tuple(np.linalg.solve(validate_matrix(self.cfg['base_matrix']),np.asarray(o.position[:2])-self.anchor[:2]))
        if previous>0 and math.dist(position,self.position)>.02*(stamp-previous)+.006:
            raise Failure('SENSOR_INVALID','fruit motion incompatible with bounded base translation')
        self.position=tuple(float(v) for v in position)
        return dict(stamp=stamp,session_id=session,track_id=self.track,x=self.position[0],y=self.position[1],
                    fruit_position_m=tuple(o.position),valid=True,diagnostic='fruit_relative_translation; yaw unobserved')


class MicroNavigator:
    """Bounded relative-body goal verified using measured fruit-relative XY change."""
    def __init__(self):
        self.goal=None;self.origin=None;self.last_seq=-1;self.heartbeat=-math.inf
        self.until=0.;self.stable=0;self.last_stamp=0.;self.reason='idle'

    def command(self,navigation,now):
        if navigation:self.heartbeat=now
        else:self.stop('HOLD received')

    def stop(self,reason):
        self.goal=None;self.reason=reason;self.stable=0;self.heartbeat=-math.inf

    def accept(self,delta,position,seq,now):
        if (self.goal is not None or now-self.heartbeat>.3 or seq<=self.last_seq or
                not finite_vector(delta,2) or not finite_vector(position,2) or math.hypot(*delta)>.050000001):
            self.stop('invalid, replayed, unarmed or excessive relative goal');raise ValueError(self.reason)
        self.origin=tuple(position);self.goal=tuple(a+b for a,b in zip(position,delta));self.last_seq=seq
        self.until=now+10.;self.stable=0;self.last_stamp=0.;self.reason='moving'

    def tick(self,position,stamp,age,now):
        if self.goal is None:return (0.,0.),False
        if (not finite_vector(position,2) or not math.isfinite(stamp) or stamp<=0 or
                not -.05<=age<=.5 or now-self.heartbeat>.3 or now>self.until or math.dist(position,self.origin)>.055):
            self.stop('visual/heartbeat lost, step deadline or measured XY travel exceeded');return (0.,0.),False
        error=np.asarray(self.goal)-position;norm=float(np.linalg.norm(error))
        if norm<=.002:
            if stamp>self.last_stamp:self.stable+=1;self.last_stamp=stamp
            if self.stable>=4:self.stop('arrived with four distinct fruit feedback frames');return (0.,0.),True
            return (0.,0.),False
        self.stable=0;velocity=error/norm*min(.02,max(.008,norm*2.))
        return tuple(float(v) for v in velocity),False
