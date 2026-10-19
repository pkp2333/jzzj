"""Stage-C handoff service and execution owner. Tag navigation is an external peer."""
import json
import threading
import time
from pathlib import Path
from .handoff import Handoff
from .fixed_grasp import FixedSequence
from .fixed_grasp_ros import FixedRos,wait_startup


class FruitStage:
    def __init__(self,r,reference,base_cfg,reference_path,cancel):
        from std_msgs.msg import String
        from jzzj_grasp_interfaces.srv import StageCControl,StageCControlResponse
        self.r,self.reference,self.base_cfg,self.path,self.cancel=r,reference,base_cfg,Path(reference_path),cancel
        self.lock=threading.RLock();self.machine=Handoff(self.path.parent/'handoff_state.json')
        self.io=None;self.pending=None;self.running=False;self.external_empty=False
        self.response=StageCControlResponse;self.string=String
        self.state_pub=r.Publisher('/stage_c/handoff/status',String,queue_size=1,latch=True)
        self.result_pub=r.Publisher('/stage_c/handoff/result',String,queue_size=1,latch=True)
        self.service=r.Service('/stage_c/handoff/control',StageCControl,self.control)
        self.timer=r.Timer(r.Duration(.2),lambda event:self.publish())
        self.publish()

    def publish(self):
        with self.lock:
            state=self.machine.snapshot()
            for key in ('requests','used'):state.pop(key,None)
            state.update(version=1,stamp=self.r.Time.now().to_sec())
            self.state_pub.publish(self.string(data=json.dumps(state,ensure_ascii=False,allow_nan=False)))

    def transition(self,state,reason=''):
        with self.lock:
            if self.machine.state!=state:self.machine.transition(state,reason)
            else:self.machine.diagnostic=reason
            self.publish()

    def check_upstream_released(self,allow_current=False):
        import rosgraph
        pubs,subs,_=rosgraph.Master(self.r.get_name()).getSystemState();pubs,subs=dict(pubs),dict(subs)
        own=self.r.get_name();nav='/stage_c_fixed_navigation'
        velocity={nav} if allow_current else set()
        targets={own} if allow_current else set()
        modes={own,nav} if allow_current else set()
        if (set(pubs.get('/cmd_vel',[]))-velocity or set(pubs.get('/car_target',[]))-targets or
                set(pubs.get('/car_cmd',[]))-modes):
            raise ValueError('upstream must unregister its command publishers before TAKEOVER')
        if set(subs.get('/car_target',[]))-{'/stage_c_fixed_navigation'}:
            raise ValueError('upstream target subscriber must release shared topic before TAKEOVER')
        allowed_modes={'/chassis',nav}|({own} if allow_current else set())
        if set(subs.get('/cmd_vel',[]))-{'/chassis'} or set(subs.get('/car_cmd',[]))-allowed_modes:
            raise ValueError('another base consumer/controller remains connected')

    def control(self,req):
        with self.lock:
            try:
                names={0:'TAKEOVER',1:'CANCEL',2:'HANDOFF_ACK',3:'RECOVER'}
                operation=names.get(req.operation)
                if operation is None:raise ValueError('unsupported stage operation')
                if self.pending is not None and operation!='CANCEL':raise ValueError('another control request is pending')
                if operation=='TAKEOVER':self.check_upstream_released()
                if operation=='CANCEL' and self.machine.state=='DONE_WAIT_HANDOFF':
                    raise ValueError('completed stage is already holding; use HANDOFF_ACK')
                self.machine.request(operation,req.session_id,int(req.request_id),req.issued_at.to_sec(),
                    req.expires_at.to_sec(),self.r.Time.now().to_sec(),next_owner=req.next_owner,
                    confirmed_stopped=req.confirmed_stopped,confirmed_empty_and_clear=req.confirmed_empty_and_clear)
                if operation=='TAKEOVER':self.pending='RUN';self.cancel.clear()
                elif operation=='CANCEL':self.cancel.set()
                elif operation=='HANDOFF_ACK':self.pending='RELEASE'
                elif operation=='RECOVER':self.pending='RECOVER';self.external_empty=req.confirmed_empty_and_clear
                self.publish()
                return self.response(True,self.machine.state,'ACCEPTED','accepted; completion is published on status/result')
            except Exception as exc:return self.response(False,self.machine.state,'REJECTED',str(exc))

    def run(self):
        session=self.machine.session;result=None
        try:
            wait_startup(self.r,40.,self.cancel,base_adjust=True)
            self.io=FixedRos(self.r,self.reference,session,self.cancel,base_cfg=self.base_cfg)
            self.io.stage_state=self.transition
            result=FixedSequence(self.io,self.reference).run()
            result.update(session_id=session,base_trace=self.io.base.trace,
                          base_stop_evidence=self.io.base.stop_evidence,
                          pregrasp_visual_stop_confirmed=self.io.base.pregrasp_visual_stop,
                          independent_postgrasp_stop_confirmed=False)
            result['actuator_hold_confirmed']=result.pop('stop_confirmed',False)
            # SUCCESS means action completion, as requested; handoff still needs an explicit external stop ACK.
            if result.get('motion_complete'):result.update(status='SUCCESS',success=True)
        except Exception as exc:
            try:stopped=self.io is not None and self.io.hold()
            except Exception:stopped=False
            result=dict(status='STOP_FAILED' if not stopped else getattr(exc,'code','NOT_READY'),
                        success=False,motion_complete=False,stop_confirmed=stopped,error=str(exc),session_id=session)
        finally:
            if self.io is not None:
                try:self.io.close()
                except Exception:pass
        result.update(reference_file=str(self.path.resolve()),reference_position_m=self.reference['reference_position_m'])
        out=self.path.parent/'runs'/(session+'.json');out.parent.mkdir(parents=True,exist_ok=True)
        # Never overwrite a previous session result.
        with out.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False)
        with self.lock:
            self.machine.finish(result)
            self.result_pub.publish(self.string(data=json.dumps(result,ensure_ascii=False,allow_nan=False)))
            self.publish()

    def release(self,recovery=False):
        session=self.machine.session
        self.check_upstream_released(allow_current=True)
        if self.io is None:
            self.io=FixedRos(self.r,self.reference,session,self.cancel,base_cfg=self.base_cfg)
            self.io.base.phase='ARM'
        # This branch is reached only after an explicit session-scoped stop acknowledgement.
        self.cancel.clear();self.io.deadline=time.monotonic()+180.
        if recovery:
            held=self.r.get_param('/stage_c/fixed_grasp/session','')
            if held not in ('',session):raise ValueError('another fixed-grasp session owns recovery')
            for key in ('stop_failed_latched','holding_latched'):
                if self.r.get_param('/stage_c_server/'+key,False):raise ValueError('recover the original C server latch first')
            self.io.base.profile(True)
            if not self.io.arm.hold(12.):raise ValueError('arm measured recovery hold failed')
            self.io.base.stop(visual=False)
            base_owner=self.r.get_param('/stage_c_fixed_navigation/active_session','')
            if base_owner==session:self.io.base.release(empty=True)
            elif base_owner=='':
                self.io.base.profile(False)
                self.io.base.goal.unregister();self.io.base.mode.unregister()
            else:raise ValueError('another session owns the base adapter')
            self.r.set_param('/stage_c/fixed_grasp/session','');self.r.set_param('/stage_c/taught/recovery_required',False)
        else:
            self.io.release(session)
            self.io.base.release(empty=False)
        self.dispose()
        with self.lock:self.machine.released(recovered=recovery);self.publish()

    def dispose(self):
        if self.io is None:return
        for obj in [self.io.pub,self.io.sub,self.io.stop_sub,self.io.arm.sub,self.io.arm.observation_pub,*self.io.base.subs]:
            try:obj.unregister()
            except Exception:pass
        self.io=None

    def loop(self):
        while not self.r.is_shutdown():
            with self.lock:pending=self.pending;self.pending=None;self.running=pending=='RUN'
            try:
                if pending=='RUN':self.run()
                elif pending in ('RELEASE','RECOVER'):self.release(recovery=pending=='RECOVER')
                elif self.io is not None and self.io.base.acquired:
                    # Completion/fault does not silently release ownership or resume navigation.
                    self.io.base.mode.publish(self.io.base.cmd(cmd=self.io.base.cmd.HOLD))
            except Exception as exc:
                if self.io is not None:
                    self.r.set_param('/stage_c/taught/recovery_required',True)
                    self.r.set_param('/stage_c/fixed_grasp/session',self.machine.session)
                with self.lock:
                    self.machine.finish(dict(status='STOP_FAILED',success=False,motion_complete=False,error=str(exc)))
                    self.publish()
            finally:self.running=False
            if self.cancel.is_set() and self.machine.state=='TAKEOVER_CHECK':
                self.machine.finish(dict(status='CANCELED',success=False,motion_complete=False,error='canceled before acquisition'))
                self.publish()
            time.sleep(.05)

    def shutdown(self):
        self.cancel.set()
        if self.io is not None:
            self.io.base.mode.publish(self.io.base.cmd(cmd=self.io.base.cmd.HOLD))
            try:self.io.hold()
            except Exception:pass
