"""YOLO-box-only colour/depth observation, with no ROS or hardware dependency."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Candidate:
    track_id: int
    class_id: int
    label: str
    score: float
    box: tuple

def bgr_to_hsv(image):
    """HSV matching OpenCV units: hue [0,180), saturation/value [0,255]."""
    x=np.asarray(image,dtype=np.float64)/255.0
    b,g,r=x[...,0],x[...,1],x[...,2]
    hi=x.max(axis=-1);lo=x.min(axis=-1);d=hi-lo
    h=np.zeros_like(hi);nonzero=d>1e-12
    mask=nonzero & (hi==r);h[mask]=((g[mask]-b[mask])/d[mask])%6
    mask=nonzero & (hi==g) & (hi!=r);h[mask]=(b[mask]-r[mask])/d[mask]+2
    mask=nonzero & (hi==b) & (hi!=r) & (hi!=g);h[mask]=(r[mask]-g[mask])/d[mask]+4
    s=np.zeros_like(hi);np.divide(d,hi,out=s,where=hi>0)
    return np.stack((h*30,s*255,hi*255),axis=-1)

def colour_mask(bgr, intervals):
    hsv=bgr_to_hsv(bgr);mask=np.zeros(hsv.shape[:2],dtype=bool)
    for low,high in intervals:
        mask |= np.all((hsv>=np.array(low)) & (hsv<=np.array(high)),axis=-1)
    # One-pixel erosion excludes box/background boundaries without expanding YOLO ROI.
    if min(mask.shape)>=3:
        eroded=mask.copy()
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                eroded[1:-1,1:-1] &= mask[1+dy:mask.shape[0]-1+dy,1+dx:mask.shape[1]-1+dx]
        eroded[[0,-1],:]=False;eroded[:,[0,-1]]=False
        mask=eroded
    return mask

def validate_transform(value):
    t=np.asarray(value,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or
            not np.allclose(t[3],[0,0,0,1],atol=1e-7) or
            not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-4) or
            not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-4)):
        raise ValueError('invalid rigid camera transform')
    return t

def project_depth_to_rgb(depth, depth_k, rgb_k, depth_to_rgb, shape, scale):
    """Z-buffer registration; inputs must be rectified pinhole images (ROS gate checks D)."""
    t=validate_transform(depth_to_rgb)
    if not np.isfinite(scale) or scale<=0:
        raise ValueError('invalid depth scale')
    kd=np.asarray(depth_k,dtype=float).reshape(3,3);kr=np.asarray(rgb_k,dtype=float).reshape(3,3)
    if not np.isfinite(kd).all() or not np.isfinite(kr).all() or any(abs(v)<=1e-12 for v in (kd[0,0],kd[1,1],kr[0,0],kr[1,1])):
        raise ValueError('invalid intrinsics')
    yy,xx=np.indices(depth.shape);z=depth.astype(float)*scale
    valid=np.isfinite(z)&(z>0)
    points=np.stack(((xx[valid]-kd[0,2])*z[valid]/kd[0,0],
                     (yy[valid]-kd[1,2])*z[valid]/kd[1,1],z[valid]),axis=0)
    points=t[:3,:3]@points+t[:3,3:4]
    front=points[2]>0;points=points[:,front]
    u=np.rint(kr[0,0]*points[0]/points[2]+kr[0,2]).astype(int)
    v=np.rint(kr[1,1]*points[1]/points[2]+kr[1,2]).astype(int)
    h,w=shape;ok=(u>=0)&(u<w)&(v>=0)&(v<h)
    out=np.full(h*w,np.inf);np.minimum.at(out,v[ok]*w+u[ok],points[2,ok])
    out[~np.isfinite(out)]=0
    return out.reshape(h,w)

class DetectorAdapter:
    def __init__(self,cfg):
        self.cfg=cfg
        self.transform=validate_transform(cfg['camera_to_arm'])
        for key in ('min_confidence','min_color_fraction','min_depth_fraction'):
            if not math.isfinite(cfg[key]) or not 0<cfg[key]<=1:raise ValueError('invalid fraction/confidence threshold')
        for key in ('min_color_pixels','min_depth_pixels'):
            if type(cfg[key]) is not int or cfg[key]<1:raise ValueError('positive pixel thresholds required')
        if not all(math.isfinite(cfg[k]) and cfg[k]>0 for k in ('min_depth_m','max_depth_m','depth_outlier_m')) or cfg['min_depth_m']>=cfg['max_depth_m']:
            raise ValueError('invalid depth limits')
        for intervals in cfg['colors'].values():
            for low,high in intervals:
                if len(low)!=3 or len(high)!=3 or any(not math.isfinite(a) or not math.isfinite(b) or not 0<=a<=b<=m for a,b,m in zip(low,high,(180,255,255))):
                    raise ValueError('invalid HSV interval')

    def extract(self,bgr,depth_m,k,candidates,color):
        if color not in self.cfg['colors']:
            raise ValueError('unsupported target colour')
        if bgr.ndim!=3 or bgr.shape[2]!=3 or depth_m.shape!=bgr.shape[:2]:
            raise ValueError('RGB/depth shape mismatch')
        k=np.asarray(k,dtype=float).reshape(3,3)
        if not np.isfinite(k).all() or any(abs(v)<=1e-12 for v in (k[0,0],k[1,1])):
            raise ValueError('invalid RGB intrinsics')
        if not self.cfg['allowed_labels']:
            raise ValueError('explicit model label allowlist is required')
        h,w=depth_m.shape;results=[]
        for c in candidates:
            if (not math.isfinite(c.score) or c.score<self.cfg['min_confidence'] or
                    c.label not in self.cfg['allowed_labels'] or
                    (self.cfg.get('allowed_class_ids') and c.class_id not in self.cfg['allowed_class_ids']) or
                    len(c.box)!=4 or not np.isfinite(c.box).all()):
                continue
            x1,y1,x2,y2=c.box
            # Reject inverted/out-of-view boxes; clip legitimate partial boxes.
            if x2<=x1 or y2<=y1:continue
            x1,y1=max(0,int(math.floor(x1))),max(0,int(math.floor(y1)))
            x2,y2=min(w,int(math.ceil(x2))),min(h,int(math.ceil(y2)))
            if x2-x1<3 or y2-y1<3:continue
            mask=colour_mask(bgr[y1:y2,x1:x2],self.cfg['colors'][color])
            count=int(mask.sum());area=mask.size
            if count<self.cfg['min_color_pixels'] or count/area<self.cfg['min_color_fraction']:continue
            d=depth_m[y1:y2,x1:x2]
            valid=mask & np.isfinite(d) & (d>=self.cfg['min_depth_m']) & (d<=self.cfg['max_depth_m'])
            if valid.sum()<self.cfg['min_depth_pixels'] or valid.sum()/max(count,1)<self.cfg['min_depth_fraction']:continue
            median=float(np.median(d[valid]));mad=float(np.median(np.abs(d[valid]-median)))
            valid &= np.abs(d-median)<=max(self.cfg['depth_outlier_m'],3*mad)
            if valid.sum()<self.cfg['min_depth_pixels']:continue
            y,x=np.nonzero(valid);z=d[valid]
            points=np.vstack(((x+x1-k[0,2])*z/k[0,0],(y+y1-k[1,2])*z/k[1,1],z))
            arm=self.transform[:3,:3]@points+self.transform[:3,3:4]
            position=tuple(float(v) for v in np.median(arm,axis=1))
            quality=float(c.score*min(1.,count/area)*valid.sum()/max(count,1))
            results.append((c,position,quality,(x1,y1,x2-x1,y2-y1)))
        return results
