"""Optional stage-C client; no navigation, context synthesis or base commands."""
import threading
import actionlib
import rospy
from jzzj_grasp_interfaces.msg import VisualGraspAction, VisualGraspGoal

class StageCAdapter:
    def __init__(self):
        self.client=actionlib.SimpleActionClient('/stage_c/grasp',VisualGraspAction)
        self.lock=threading.RLock();self.active=False;self.done=None;self.latest=None
        self.started=0.;self.cancel_at=None;self.serial=0;self.stop_uncertain=False

    def reset(self):
        with self.lock:
            if self.active or self.stop_uncertain:
                raise RuntimeError('stage C busy or physical stop uncertain; operator intervention required')
            self.serial+=1;self.done=self.latest=None;self.cancel_at=None

    def start(self,color):
        with self.lock:
            if self.active:raise RuntimeError('stage C still running or canceling')
            if not self.client.wait_for_server(rospy.Duration(.5)):raise RuntimeError('stage C server unavailable')
            session=rospy.get_param('~stage_c_session_id','')
            if not session:raise RuntimeError('explicit stage_c_session_id / upstream context required')
            self.serial+=1;serial=self.serial
            g=VisualGraspGoal(session_id=session,target_color=color,
                grasp_profile=rospy.get_param('~stage_c_profile','tabletop'),
                timeout=rospy.Duration(rospy.get_param('~stage_c_timeout',90.)))
            self.active=True;self.done=self.latest=None;self.cancel_at=None;self.started=rospy.get_time()
            self.client.send_goal(g,done_cb=lambda state,result:self.completed(serial,state,result),
                                  feedback_cb=lambda f:self.feedback(serial,f))

    def feedback(self,serial,f):
        with self.lock:
            if serial==self.serial:self.latest=f.step

    def completed(self,serial,state,result):
        with self.lock:
            if serial==self.serial:
                self.active=False;self.done=(state,result)
                if result is None or result.code==9 or (self.cancel_at is not None and not result.stop_confirmed):
                    self.stop_uncertain=True

    def cancel(self):
        import time
        with self.lock:
            if self.active:
                self.client.cancel_goal()
                if self.cancel_at is None:self.cancel_at=time.monotonic()

    def tick(self,node):
        import time
        with self.lock:
            done=self.done;latest=self.latest;cancel_at=self.cancel_at;active=self.active
        if done:
            state,result=done
            with self.lock:self.done=None
            if node.state!='RUNNING':
                if not result or not result.stop_confirmed:node._fault('stage C stop not confirmed')
                return
            if state==3 and result and result.code==0 and result.grasp_confirmed and result.ready_for_transport and result.holding_known and result.holding_object:
                node.grasp_confirmed=True
                if node.MOVE_TO_DROP not in node.stages:node._fault('MOVE_TO_DROP missing from configured stages');return
                node.stage_index=node.stages.index(node.MOVE_TO_DROP)
                node.stage_started_at=rospy.Time.now();node._publish_state();node._publish_stage_command()
            else:node._fault('stage C failed: '+(result.message if result else 'missing result'))
            return
        if cancel_at is not None:
            stop_wait=max(rospy.get_param('~stage_c_cancel_timeout',6.),
                          rospy.get_param('/stage_c_server/execution/stop_timeout',12.)+1.)
            if active and time.monotonic()-cancel_at>stop_wait:
                node._fault('stage C cancel timeout; physical stop not confirmed')
            return
        if not active:return
        if rospy.get_time()-self.started>=rospy.get_param('~stage_c_timeout',90.):
            self.cancel();node._fault('stage C timeout: cancel requested');return
        mapping={'CHECK_READY':node.PICK_ALIGN,'PREPARE':node.PICK_ALIGN,'SEARCH_TARGET':node.SEARCH_OBJECT,
                 'SERVO_PREGRASP':node.VISUAL_APPROACH,'DESCEND':node.GRASP,'CLOSE_GRIPPER':node.GRASP,
                 'LIFT':node.VERIFY_GRASP,'VERIFY_GRASP':node.VERIFY_GRASP,'MOVE_TO_TRANSPORT_POSE':node.VERIFY_GRASP}
        if latest in mapping and mapping[latest] in node.stages:
            node.stage_index=node.stages.index(mapping[latest]);node._publish_state()
