#!/usr/bin/env python3
"""Validate external assets, then exec the EXISTING C++ detector, not another inference implementation."""
import argparse
import json
import os
import sys
from pathlib import Path
from jzzj_grasp.fruit6 import check_resources,load_manifest,validate_config


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir',required=True)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--labels',required=True)
    parser.add_argument('--config',required=True)
    args,remaps=parser.parse_known_args(argv)
    # Preserve ROS __name/__ns/__log remaps, reject unrelated unknown command arguments.
    if any(':=' not in arg for arg in remaps):parser.error('only ROS remap arguments may follow preflight arguments')
    try:
        if not Path(args.model_dir).is_absolute():raise ValueError('model_dir must be an absolute directory (expand ~ in the shell)')
        manifest=load_manifest(args.manifest)
        validate_config(json.loads(Path(args.config).read_text(encoding='utf-8')),manifest)
        report=check_resources(args.model_dir,args.labels,args.manifest)
        print('Fruit6 verified assets (NOT inference validation): '+json.dumps(report,ensure_ascii=False),flush=True)
        from roslib.packages import find_node
        executables=find_node('yolov5_rknn','yolov5_rknn_stream')
        if not executables:raise ValueError('build/source yolov5_rknn first; executable not found')
        # roslaunch loads flat config/model/label parameters under the detector namespace.
        # No rospy node is initialized here: the existing executable receives all ROS remaps.
        os.execv(executables[0],[executables[0]]+remaps)
    except (OSError,ValueError,KeyError,ImportError) as exc:
        print('Refusing fruit6 detector launch: '+str(exc),file=sys.stderr)
        return 2

if __name__=='__main__':
    sys.exit(main())
