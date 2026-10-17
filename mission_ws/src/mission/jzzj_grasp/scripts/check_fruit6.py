#!/usr/bin/env python3
"""Read-only manifest verification / saved tensor replay. Never imports rknn or torch."""
import argparse
import json
from pathlib import Path
import sys
from jzzj_grasp.fruit6 import check_resources,load_manifest,decode_saved_outputs


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir',required=True)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--labels',required=True)
    parser.add_argument('--all-assets',action='store_true')
    parser.add_argument('--replay',action='store_true',help='decode saved tensors; NOT fresh inference')
    args=parser.parse_args(argv)
    try:
        report=check_resources(args.model_dir,args.labels,args.manifest,args.all_assets or args.replay)
        if args.replay:
            import numpy as np
            m=load_manifest(args.manifest)
            outputs=[np.load(Path(args.model_dir)/('onnx_yolov5_'+str(i)+'.npy'),allow_pickle=False) for i in range(3)]
            ref=m['offline_reference']
            boxes,classes,scores=decode_saved_outputs(outputs,ref['source_image_size_wh'],m['conf_thresh'],m['iou_thresh'],m['anchors'])
            wh=ref['source_image_size_wh'];expected=np.asarray(ref['boxes_model_xyxy'])*np.array([wh[0]/640,wh[1]/640]*2)
            if classes.tolist()!=ref['classes'] or not np.allclose(scores,ref['scores'],atol=.00015,rtol=0) or not np.allclose(boxes,expected,atol=.12,rtol=0):
                raise ValueError('saved tensor regression differs from manifest baseline')
            report['saved_tensor_replay']={'source':'PREVIOUSLY_SAVED_OUTPUTS_ONLY','count':len(classes),
                'class_ids':classes.tolist(),'labels':[m['labels']['names'][int(i)] for i in classes],
                'scores':scores.tolist(),'source_size_wh':wh,'boxes_source_xyxy':boxes.tolist()}
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 0
    except (OSError,ValueError,KeyError) as exc:
        print('fruit6 preflight failed: '+str(exc),file=sys.stderr)
        return 2

if __name__=='__main__':
    sys.exit(main())
