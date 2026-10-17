"""Read-only model preflight and saved-output reference decoder. No RKNN/ROS import.

The decoder handles already sigmoid-activated, dequantized outputs; it must not
be mistaken for a runtime, model conversion, or new inference result.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def load_manifest(path):
    m=json.loads(Path(path).read_text(encoding='utf-8'))
    if m.get('schema_version')!=1 or m.get('profile')!='fruit6_rknn_152':
        raise ValueError('unsupported model manifest')
    if m['labels']['class_num']!=6 or m['labels']['names']!=['fruit-'+str(i) for i in range(1,7)]:
        raise ValueError('fruit6 neutral label contract mismatch')
    if m['model']['input']['shape']!=[1,3,640,640] or [o['shape'] for o in m['model']['outputs']]!=[[1,33,80,80],[1,33,40,40],[1,33,20,20]]:
        raise ValueError('fruit6 tensor contract mismatch')
    if len(m['anchors'])!=18 or any(not math.isfinite(v) or v<=0 for v in m['anchors']):
        raise ValueError('invalid model anchors')
    if m['preprocessing']['resize']!='stretch' or m['preprocessing']['application_divide_by_255'] is not False:
        raise ValueError('unsupported preprocessing contract')
    if m['model']['file']!=Path(m['model']['file']).name:
        raise ValueError('model file must be a basename')
    return m


def validate_config(config,manifest):
    expected={'class_num':6,'anchors':manifest['anchors'],'expected_input_width':640,
              'expected_input_height':640,'conf_thresh':manifest['conf_thresh'],'iou_thresh':manifest['iou_thresh']}
    for key,value in expected.items():
        if config.get(key)!=value:
            raise ValueError('detector config mismatch: '+key)
    if not math.isfinite(config['max_frame_age']) or config['max_frame_age']<=0:
        raise ValueError('invalid detector frame age')


def check_resources(model_dir,labels_path,manifest_path,all_assets=False):
    """Pin external model bytes and package label bytes. Nothing is copied or written."""
    m=load_manifest(manifest_path);base=Path(model_dir).expanduser().resolve()
    model=base/m['model']['file'];labels=Path(labels_path)
    if not model.is_file() or model.stat().st_size==0:raise ValueError('model missing/empty: '+str(model))
    if sha256(model)!=m['model']['sha256']:raise ValueError('model SHA256 mismatch')
    if not labels.is_file():raise ValueError('package labels missing')
    lines=labels.read_text(encoding='utf-8-sig').splitlines()
    if lines!=['6']+m['labels']['names']:
        raise ValueError('label count/order mismatch; external 14-class names are NOT valid here')
    if sha256(labels)!=m['labels']['sha256']:raise ValueError('label SHA256 mismatch')
    checked=[m['model']['file']]
    if all_assets:
        for name,record in m['provenance']['files'].items():
            path=base/name
            if path.name!=name or not path.is_file():raise ValueError('reference asset missing: '+name)
            if sha256(path)!=record['sha256']:raise ValueError('reference SHA256 mismatch: '+name)
            checked.append(name)
    return {'profile':m['profile'],'model':str(model),'labels':str(labels.resolve()),
            'model_sha256':m['model']['sha256'],'class_names':m['labels']['names'],
            'checked_assets':sorted(set(checked)),
            'runtime_compatibility':'NOT_VALIDATED_ON_DEVICE','purpose':'integration_validation_only'}


def nms_xyxy(boxes,scores,threshold):
    # Same IoU convention as the supplied NumPy example (no inclusive pixel +1).
    if not 0<threshold<1:raise ValueError('invalid NMS threshold')
    boxes=np.asarray(boxes,dtype=float);scores=np.asarray(scores,dtype=float)
    order=np.argsort(scores)[::-1];keep=[]
    area=(boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:,1])
    while len(order):
        index=order[0];keep.append(int(index));rest=order[1:]
        if not len(rest):break
        intersection=np.maximum(0,np.minimum(boxes[index,2:],boxes[rest,2:])-np.maximum(boxes[index,:2],boxes[rest,:2])+1e-5)
        overlap=intersection[:,0]*intersection[:,1]
        iou=overlap/np.maximum(area[index]+area[rest]-overlap,1e-12)
        order=rest[iou<=threshold]
    return keep


def decode_saved_outputs(outputs,source_size=(640,480),conf_thresh=.25,iou_thresh=.45,anchors=None):
    """Return source-pixel boxes/class IDs/scores. No network/model is executed."""
    anchors=[10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326] if anchors is None else anchors
    if len(outputs)!=3 or len(anchors)!=18:raise ValueError('three outputs / eighteen anchors required')
    if not 0<conf_thresh<1 or not 0<iou_thresh<1:raise ValueError('invalid confidence/NMS threshold')
    if len(source_size)!=2 or any(not math.isfinite(v) or v<=0 for v in source_size):raise ValueError('invalid source image size')
    if any(not math.isfinite(a) or a<=0 for a in anchors):raise ValueError('invalid anchors')
    boxes=[];classes=[];scores=[]
    for head,(tensor,stride) in enumerate(zip(outputs,(8,16,32))):
        a=np.asarray(tensor)
        grid=640//stride
        if a.shape!=(1,33,grid,grid) or a.dtype.kind!='f' or not np.isfinite(a).all():
            raise ValueError('invalid saved output shape/dtype/value at head '+str(head))
        if np.any(a<0) or np.any(a>1):raise ValueError('expected sigmoid-activated dequantized outputs in [0,1]')
        a=a.reshape(3,11,grid,grid).transpose(2,3,0,1)
        yy,xx=np.indices((grid,grid))
        offset=np.stack((xx,yy),axis=-1)[:,:,None,:]
        xy=(a[...,:2]*2-.5+offset)*stride
        wh=(a[...,2:4]*2)**2*np.asarray(anchors[head*6:head*6+6]).reshape(1,1,3,2)
        xyxy=np.concatenate((xy-wh/2,xy+wh/2),axis=-1).reshape(-1,4)
        confidence=a[...,4].reshape(-1);probs=a[...,5:].reshape(-1,6)
        ids=probs.argmax(axis=-1);best=probs.max(axis=-1)
        valid=(confidence>=conf_thresh)&(best>=conf_thresh)
        boxes.append(xyxy[valid]);classes.append(ids[valid]);scores.append((confidence*best)[valid])
    boxes=np.concatenate(boxes);classes=np.concatenate(classes);scores=np.concatenate(scores)
    out_boxes=[];out_classes=[];out_scores=[]
    for category in sorted(set(classes.tolist())):
        indices=np.flatnonzero(classes==category)
        keep=nms_xyxy(boxes[indices],scores[indices],iou_thresh)
        selected=indices[keep]
        out_boxes.extend(boxes[selected]);out_classes.extend(classes[selected]);out_scores.extend(scores[selected])
    boxes=np.asarray(out_boxes,dtype=float).reshape(-1,4)
    # Direct stretch preprocessing: X and Y have independent inverse scales, no letterbox offset.
    boxes*=np.array([source_size[0]/640,source_size[1]/640]*2)
    boxes[:,[0,2]]=np.clip(boxes[:,[0,2]],0,source_size[0])
    boxes[:,[1,3]]=np.clip(boxes[:,[1,3]],0,source_size[1])
    valid=(boxes[:,2]>boxes[:,0])&(boxes[:,3]>boxes[:,1])
    return boxes[valid],np.asarray(out_classes,dtype=int)[valid],np.asarray(out_scores,dtype=float)[valid]


class FrameGate:
    """Monotonic source timestamps only. Empty results remain real fresh frames."""
    def __init__(self,max_age=.5):
        if not math.isfinite(max_age) or max_age<=0:raise ValueError('invalid frame age')
        self.max_age=max_age;self.last=0.

    def accept(self,stamp,now):
        if not math.isfinite(stamp) or not math.isfinite(now) or stamp<=self.last or stamp<=0 or not -.05<=now-stamp<=self.max_age:
            return False
        self.last=stamp
        return True
