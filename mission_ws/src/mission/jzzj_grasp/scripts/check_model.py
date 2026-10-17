#!/usr/bin/env python3
"""Offline metadata/sha256 check; tensor ABI validation occurs in the RKNN node."""
import argparse
import hashlib
import json
from pathlib import Path

def check(model,labels,class_num):
    model,labels=Path(model),Path(labels)
    if not model.is_file() or model.stat().st_size==0:raise ValueError('model missing/empty')
    lines=labels.read_text(encoding='utf-8-sig').splitlines()
    if not lines or int(lines[0])!=class_num or not 1<=class_num<=256:raise ValueError('class count mismatch')
    names=[x.strip() for x in lines[1:]]
    if len(names)!=class_num or not all(names) or len(set(names))!=class_num:raise ValueError('invalid/duplicate labels')
    def digest(p):
        h=hashlib.sha256()
        with p.open('rb') as stream:
            for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
        return h.hexdigest()
    return {'model_sha256':digest(model),'labels_sha256':digest(labels),'labels':names,
            'tensor_compatibility':'NOT_CHECKED: run RKNN target validation'}
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--model',required=True);parser.add_argument('--labels',required=True)
    parser.add_argument('--class-num',type=int,required=True);args=parser.parse_args()
    print(json.dumps(check(args.model,args.labels,args.class_num),ensure_ascii=False,indent=2))
