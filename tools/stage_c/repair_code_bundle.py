#!/usr/bin/env python3
"""Verify or restore an explicit Stage-C code bundle, retaining source/cache backups."""
import argparse
import ast
import datetime
import hashlib
import io
import json
import os
from pathlib import Path,PurePosixPath
import shutil
import stat
import tarfile
import uuid
import xml.etree.ElementTree as ET

ALLOWED={
    'catkin_ws/src/actuator/chassis/package.xml',
    'catkin_ws/src/actuator/chassis/scripts/chassis_control_node.py',
    'deploy/stage_c_fixed_base.example.yaml',
    'mission_ws/src/mission/jzzj_grasp/CMakeLists.txt',
    'mission_ws/src/mission/jzzj_grasp/launch/stage_c_fixed_base.launch',
    'mission_ws/src/mission/jzzj_grasp/launch/stage_c_fixed_full.launch',
    'mission_ws/src/mission/jzzj_grasp/package.xml',
    'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_grasp.py',
    'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_navigation.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_base.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_base_ros.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_grasp.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fixed_grasp_ros.py',
    'mission_ws/src/mission/jzzj_grasp/tests/test_fixed_base.py',
    'mission_ws/src/mission/jzzj_grasp/tests/test_fixed_grasp.py',
    'mission_ws/src/mission/jzzj_grasp/tests/test_fixed_startup.py',
    'mission_ws/src/mission/jzzj_grasp_interfaces/CMakeLists.txt',
    'tools/stage_c/build.sh','tools/stage_c/calibrate_fixed_base.py','tools/stage_c/start_fixed_grasp.py',
    'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fruit_stage.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/fruit_stage_ros.py',
    'mission_ws/src/mission/jzzj_grasp/src/jzzj_grasp/handoff.py',
    'mission_ws/src/mission/jzzj_grasp/tests/test_handoff_ros.py',
    'mission_ws/src/mission/jzzj_grasp_interfaces/msg/VisualBaseFeedback.msg',
    'mission_ws/src/mission/jzzj_grasp_interfaces/srv/StageCControl.srv',
    'tools/stage_c/stage_c_control.py',
}


def digest(data):return hashlib.sha256(data).hexdigest()


def load_bundle(path):
    payload={}
    with tarfile.open(path,'r:gz') as tar:
        members=tar.getmembers();names=[m.name for m in members]
        if len(names)!=len(set(names)) or set(names)!=ALLOWED|{'MANIFEST.json'}:
            raise ValueError('archive must contain exactly the explicit Stage-C code allowlist and manifest')
        for m in members:
            if not m.isfile() or m.size>2_000_000 or PurePosixPath(m.name).is_absolute() or '..' in PurePosixPath(m.name).parts:
                raise ValueError('unsafe archive member: '+m.name)
        manifest=json.loads(tar.extractfile('MANIFEST.json').read())
        if set(manifest)!=ALLOWED:raise ValueError('manifest path mismatch')
        for m in members:
            if m.name=='MANIFEST.json':continue
            data=tar.extractfile(m).read()
            if b'\0' in data or digest(data)!=manifest[m.name]:raise ValueError('corrupt payload: '+m.name)
            text=data.decode('utf-8')
            if m.name.endswith('.py'):ast.parse(text,filename=m.name)
            elif m.name.endswith(('.xml','.launch')):ET.fromstring(text)
            elif m.name.endswith('.yaml'):json.loads(text)
            payload[m.name]=(data,0o755 if m.name.endswith('.sh') or '/scripts/' in m.name and m.name.endswith('.py') else 0o644)
    return payload


def target(root,name):
    path=root/name
    if path.is_symlink() or os.path.commonpath((str(path.resolve()),str(root)))!=str(root):
        raise ValueError('path escapes project or is a symlink: '+name)
    return path


def verify(root,payload):
    bad=[]
    for name,(data,_) in sorted(payload.items()):
        path=target(root,name)
        if not path.is_file():bad.append(name);print('MISSING',name);continue
        actual=path.read_bytes()
        if actual!=data:
            bad.append(name);print('MISMATCH NUL='+str(actual.count(b'\0')),name)
    return bad


def write_atomic(path,data,mode):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name('.'+path.name+'.repair-'+uuid.uuid4().hex)
    try:
        with temp.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.chmod(temp,mode);os.replace(temp,path)
    finally:
        if temp.exists():temp.unlink()


def restore(root,payload,backup):
    # Back up ALL affected files before the first source mutation.
    originals={}
    with tarfile.open(backup/'sources-before.tar','w') as tar:
        for name in sorted(payload):
            path=target(root,name)
            if path.exists():
                if not path.is_file():raise ValueError('not a regular source: '+name)
                data=path.read_bytes();mode=stat.S_IMODE(path.stat().st_mode)
                originals[name]=(data,mode)
                info=tarfile.TarInfo(name);info.size=len(data);info.mode=mode
                tar.addfile(info,io.BytesIO(data))
            else:originals[name]=None
    with (backup/'original-state.json').open('x') as f:
        json.dump({n:None if v is None else digest(v[0]) for n,v in originals.items()},f,indent=2)
        f.flush();os.fsync(f.fileno())
    with (backup/'sources-before.tar').open('rb') as f:os.fsync(f.fileno())
    applied=[]
    try:
        for name,(data,mode) in sorted(payload.items()):
            applied.append(name);write_atomic(target(root,name),data,mode)
        if hasattr(os,'sync'):os.sync()
        if verify(root,payload):raise IOError('post-write source verification failed')
    except Exception:
        for name in reversed(applied):
            path=target(root,name);original=originals[name]
            if original is None:
                if path.exists():path.unlink()
            else:write_atomic(path,*original)
        raise


def active_controllers():
    names=('arm_trajectory_node.py','arm_control_node.py','arm_test_node.py','chassis_control_node.py',
           'chassis_calib.py','wp_nav_pid_node.py','stage_c_fixed_grasp.py','stage_c_fruit_stage.py','stage_c_fixed_navigation.py')
    result=[]
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name)==os.getpid():continue
        try:args=(proc/'cmdline').read_bytes().split(b'\0')
        except (OSError,PermissionError):continue
        found=[n for n in names if any(Path(a.decode(errors='replace')).name==n for a in args if a)]
        if found:result.append((proc.name,found))
    return result


def quarantine_builds(root,backup):
    for workspace in ('catkin_ws','mission_ws'):
        path=target(root,workspace+'/build')
        if path.exists():
            if not path.is_dir():raise ValueError('build cache is not a directory')
            shutil.move(str(path),str(backup/('build-cache-'+workspace)))


def other_nul_sources(root):
    bad=[]
    for base in ('catkin_ws/src','mission_ws/src','tools/stage_c','deploy'):
        for path in (root/base).rglob('*'):
            if path.is_file() and not path.is_symlink() and path.suffix in ('.py','.sh','.xml','.launch','.yaml','.msg','.srv','.txt'):
                if b'\0' in path.read_bytes():bad.append(str(path.relative_to(root)))
    return bad


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',required=True);p.add_argument('--bundle',required=True)
    p.add_argument('--apply',action='store_true');p.add_argument('--quarantine-build',action='store_true')
    args=p.parse_args();root=Path(args.root).resolve();payload=load_bundle(args.bundle)
    if not (root/'catkin_ws/src').is_dir() or not (root/'mission_ws/src').is_dir():p.error('not the expected project')
    if args.quarantine_build and not args.apply:p.error('--quarantine-build requires --apply')
    if args.apply:
        running=active_controllers()
        if running:p.error('stop existing actuator controllers first; no processes killed: '+str(running))
        recovery=target(root,'.local/code-recovery')
        recovery.mkdir(parents=True,exist_ok=True)
        backup=recovery/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'-'+uuid.uuid4().hex[:8]);backup.mkdir()
        print('BACKUP:',backup,flush=True)
        restore(root,payload,backup)
        if args.quarantine_build:quarantine_builds(root,backup)
        print('Sources restored; private samples, sessions, recovery latches and hardware locks untouched.')
    bad=verify(root,payload)
    remaining=other_nul_sources(root)
    for name in remaining:print('OTHER_NUL_SOURCE',name)
    if bad or remaining:raise SystemExit(2)
    print('CODE_BUNDLE_VERIFIED: 27 files match SHA256; no NUL in scanned source types. No actuator command sent.')


if __name__=='__main__':main()
