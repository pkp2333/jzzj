"""Persistent session state machine; no ROS or actuator access."""
import json
import math
import os
from pathlib import Path
import re

STATES=('WAIT_TAKEOVER','TAKEOVER_CHECK','OBSERVE','BASE_ADJUST','STOP_CONFIRM',
        'ARM_GRASP','DONE_WAIT_HANDOFF','FAULT','RELEASED')
ALLOWED={
    'TAKEOVER_CHECK':{'OBSERVE','STOP_CONFIRM','FAULT'},
    'OBSERVE':{'BASE_ADJUST','STOP_CONFIRM','FAULT'},
    'BASE_ADJUST':{'STOP_CONFIRM','FAULT'},
    'STOP_CONFIRM':{'OBSERVE','BASE_ADJUST','ARM_GRASP','DONE_WAIT_HANDOFF','FAULT'},
    'ARM_GRASP':{'DONE_WAIT_HANDOFF','FAULT'},
}


def validate_envelope(session,request_id,issued,expires,now,last=0):
    if (not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',session or '') or type(request_id) is not int or request_id<=last or
            not all(math.isfinite(t) for t in (issued,expires,now)) or issued<=0 or issued>now+.05 or
            expires<now-.05 or not 0<expires-issued<=5.):
        raise ValueError('invalid, expired or replayed session request')


class Handoff:
    def __init__(self,path=None):
        self.path=Path(path) if path is not None else None
        self.state='WAIT_TAKEOVER';self.session='';self.requests={};self.used=[]
        self.code='';self.diagnostic='';self.cancel_requested=False;self.next_owner=''
        self.result={};self.owner='';self.recovery_required=False;self.recovery_authorized=False
        if self.path is not None and self.path.exists():
            saved=json.loads(self.path.read_text(encoding='utf-8'))
            self.requests=saved['requests'];self.used=saved['used'];self.session=saved['session_id']
            self.result=saved.get('result',{});self.next_owner=saved.get('next_owner','')
            self.owner=saved.get('owner','')
            self.state=saved['state']
            if self.state not in ('WAIT_TAKEOVER','RELEASED'):
                self.state='FAULT';self.code='RECOVERY_REQUIRED';self.recovery_required=True
                self.diagnostic='previous session did not complete explicit handoff; no automatic restart'
            self.save()

    def snapshot(self):
        return dict(state=self.state,session_id=self.session,requests=self.requests,used=self.used,
                    code=self.code,diagnostic=self.diagnostic,owner=self.owner,next_owner=self.next_owner,
                    cancel_requested=self.cancel_requested,recovery_required=self.recovery_required,
                    result=self.result)

    def save(self):
        if self.path is None:return
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.path.with_name(self.path.name+'.tmp')
        with temporary.open('w',encoding='utf-8') as f:
            json.dump(self.snapshot(),f,ensure_ascii=False,allow_nan=False);f.flush();os.fsync(f.fileno())
        os.replace(temporary,self.path)

    def request(self,operation,session,request_id,issued,expires,now,**flags):
        if (not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',session or '') or type(request_id) is not int or request_id<=0 or
                not all(math.isfinite(t) for t in (issued,expires,now)) or
                not now-.05<=expires or issued>now+.05 or issued<=0 or not 0<expires-issued<=5. or
                request_id<=self.requests.get(session,0)):
            raise ValueError('invalid, expired or replayed session request')
        if operation=='TAKEOVER':
            if self.state not in ('WAIT_TAKEOVER','RELEASED') or session in self.used or self.recovery_required:
                raise ValueError('busy, recovered or previously used session')
            if flags.get('confirmed_stopped') is not True:raise ValueError('upstream stop acknowledgement required')
            self.session=session;self.state='TAKEOVER_CHECK';self.used.append(session)
            self.cancel_requested=False;self.code='';self.diagnostic='checking exclusive graph and measured visual stop'
            self.result={};self.owner='stage_c_pending';self.next_owner='';self.recovery_authorized=False
        else:
            if session!=self.session or self.state in ('WAIT_TAKEOVER','RELEASED'):
                raise ValueError('no matching active session')
            if operation=='CANCEL':
                self.cancel_requested=True;self.code='CANCELED';self.diagnostic='stop requested; awaiting stop processing'
            elif operation=='HANDOFF_ACK':
                if self.state!='DONE_WAIT_HANDOFF' or flags.get('confirmed_stopped') is not True:
                    raise ValueError('completed stage and external stop confirmation required')
                if not flags.get('next_owner'):raise ValueError('explicit next owner required')
                self.next_owner=flags['next_owner']
                self.diagnostic='external stop acknowledged; release must be measured/processed before transfer'
            elif operation=='RECOVER':
                if self.state!='FAULT' or not flags.get('confirmed_stopped') or not flags.get('confirmed_empty_and_clear'):
                    raise ValueError('fault recovery requires explicit stopped, empty and clear confirmation')
                self.diagnostic='operator recovery requested; actual arm/base hold and idle checks still required'
                self.recovery_authorized=True
            else:raise ValueError('unknown operation')
        self.requests[session]=request_id;self.save()

    def transition(self,state,diagnostic=''):
        if state not in ALLOWED.get(self.state,set()):raise ValueError('invalid state transition '+self.state+' -> '+state)
        self.state=state;self.diagnostic=diagnostic
        if state=='OBSERVE':self.owner='stage_c'
        self.save()

    def finish(self,result):
        self.result=dict(result)
        if result.get('motion_complete') is True:
            self.state='DONE_WAIT_HANDOFF';self.code='';self.diagnostic='holding; waiting for explicit upstream takeover acknowledgement'
        else:
            self.state='FAULT';self.code=result.get('status','NOT_READY');self.diagnostic=result.get('error','stage failed')
            self.recovery_required=True
        self.save()

    def released(self,recovered=False):
        if self.state not in ('DONE_WAIT_HANDOFF','FAULT'):raise ValueError('release requires completed or recovered session')
        if recovered and not self.recovery_authorized:raise ValueError('explicit stopped/empty recovery request missing')
        if not recovered and not self.next_owner:raise ValueError('handoff acknowledgement missing')
        self.owner=self.next_owner if not recovered else '';self.state='RELEASED';self.recovery_required=False
        self.diagnostic='control released; no navigation goal or NAVIGATION command issued';self.save()
