"""Bounded map-goal micro navigation and measured body/arm alignment math."""
import math
import numpy as np
from .core import Failure,finite_vector
from .taught import validate_matrix,fit_base_matrix


def validate_base(cfg,reference=None):
    if cfg.get('version')!=1 or cfg.get('calibration_confirmed') is not True:
        raise ValueError('base calibration is not confirmed; collect +/- body X/Y samples first')
    matrix=validate_matrix(cfg.get('base_matrix'))
    fitted,_=fit_base_matrix(cfg.get('base_calibration_samples',[]),[0.,0.,1.])
    if not np.allclose(matrix,fitted,rtol=0,atol=1e-6):
        raise ValueError('base matrix differs from measured calibration samples')
    for key,upper in (('alignment_tolerance_m',.020),('step_m',.05),('total_m',math.inf),
                      ('max_steps',math.inf),('timeout_seconds',math.inf)):
        value=cfg.get(key)
        if key in ('total_m','max_steps','timeout_seconds') and value is None:continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=upper:
            raise ValueError('invalid/excessive base limit: '+key)
    if cfg['max_steps'] is not None and type(cfg['max_steps']) is not int:raise ValueError('max_steps must be integer or null')
    if cfg.get('pose_topic')!='/front_car_pose_orig_map' or cfg.get('pose_frame')!='map':
        raise ValueError('micro navigation requires front_car_pose_orig_map in map')
    if reference is not None and cfg.get('reference_calibration_sha256')!=reference['calibration_sha256']:
        raise ValueError('base calibration is not bound to this camera/arm reference calibration')
    return cfg


def correction(cfg,point,reference,pose):
    if not finite_vector(point,3) or not finite_vector(reference,3) or not finite_vector(pose,3):
        raise Failure('SENSOR_INVALID','non-finite alignment coordinates')
    if abs(point[2]-reference[2])>.005:
        raise Failure('NEEDS_REPOSITION','base translation cannot correct target height')
    error=np.array(point[:2])-reference[:2]
    if np.linalg.norm(np.asarray(point)-reference)<=cfg['alignment_tolerance_m']:return None
    delta=np.linalg.solve(validate_matrix(cfg['base_matrix']),-error)
    delta*=min(1.,cfg['step_m']/np.linalg.norm(delta))
    c,s=math.cos(pose[2]),math.sin(pose[2])
    world=np.array([[c,-s],[s,c]])@delta
    return (pose[0]+float(world[0]),pose[1]+float(world[1]),pose[2]),tuple(delta)


def angular(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))


def map_pose(msg,now):
    p,q=msg.pose.position,msg.pose.orientation;stamp=msg.header.stamp.to_sec()
    values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,stamp,now)
    if (msg.header.frame_id!='map' or not all(math.isfinite(v) for v in values) or
            stamp<=0 or not -.05<=now-stamp<=.3 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
        raise Failure('SENSOR_INVALID','fresh normalized map localization required')
    return (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),stamp


class MicroNavigator:
    """Navigation-layer controller. Scheduler publishes only car_target/car_cmd."""
    def __init__(self):
        self.goal=None;self.origin=None;self.last_seq=-1;self.heartbeat=-math.inf
        self.until=0.;self.stable=0;self.last_stamp=0.;self.reason='idle'

    def command(self,navigation,now):
        if navigation:self.heartbeat=now
        else:self.stop('HOLD received')

    def stop(self,reason):
        self.goal=None;self.reason=reason;self.stable=0;self.heartbeat=-math.inf

    def accept(self,goal,pose,seq,now):
        if (self.goal is not None or now-self.heartbeat>.3 or seq<=self.last_seq or
                not finite_vector(goal,3) or not finite_vector(pose,3) or
                math.dist(goal[:2],pose[:2])>.050000001 or abs(angular(goal[2],pose[2]))>math.radians(.5)):
            self.stop('invalid, replayed, unarmed or excessive micro goal')
            raise ValueError(self.reason)
        self.goal=tuple(goal);self.origin=tuple(pose);self.last_seq=seq
        self.until=now+10.;self.stable=0;self.last_stamp=0.;self.reason='moving'

    def tick(self,pose,stamp,age,now):
        if self.goal is None:return (0.,0.),False
        if (not finite_vector(pose,3) or not math.isfinite(stamp) or stamp<=0 or
                not -.05<=age<=.3 or now-self.heartbeat>.3 or now>self.until or
                math.dist(pose[:2],self.origin[:2])>.055 or
                abs(angular(pose[2],self.origin[2]))>math.radians(2.)):
            self.stop('pose/heartbeat lost, deadline or measured travel/yaw limit')
            return (0.,0.),False
        error=np.array(self.goal[:2])-pose[:2];norm=float(np.linalg.norm(error))
        if norm<=.002:
            if stamp>self.last_stamp:self.stable+=1;self.last_stamp=stamp
            if self.stable>=4:
                self.stop('arrived with four distinct measured poses');return (0.,0.),True
            return (0.,0.),False
        self.stable=0
        world=error/norm*min(.02,max(.008,norm*2.))
        c,s=math.cos(pose[2]),math.sin(pose[2])
        return (float(c*world[0]+s*world[1]),float(-s*world[0]+c*world[1])),False
