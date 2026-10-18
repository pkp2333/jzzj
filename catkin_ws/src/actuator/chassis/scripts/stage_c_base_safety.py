"""Pure exclusive planar-base lease. No serial/ROS access; monotonic watchdog."""
import math


class BaseLease:
    def __init__(self, enabled=False, calibrated=False, watchdog_verified=False,
                 timeout=.3, speed=.02):
        if not 0 < timeout <= .3 or not 0 < speed <= .02:
            raise ValueError('micro limits may not exceed 0.3s / 0.02m/s')
        self.enabled=all(v is True for v in (enabled,calibrated,watchdog_verified))
        self.timeout,self.speed=timeout,speed
        self.owner='';self.phase='IDLE';self.velocity=(0.,0.)
        self.command_id=0;self.received=None;self.diagnostic='';self.used=set()

    def fault(self, reason):
        if self.owner:
            self.phase='FAULT';self.velocity=(0.,0.);self.diagnostic=reason

    def accept(self, session, command_id, operation, vx, vy, now, legacy_stopped,
               operator_confirmed=False):
        if (not session or not isinstance(command_id,int) or command_id<=0 or
                not all(math.isfinite(v) for v in (vx,vy,now))):
            raise ValueError('invalid base command')
        if operation not in (0,1,2,3,4):raise ValueError('unknown base operation')
        if abs(vx)>self.speed+1e-12 or abs(vy)>self.speed+1e-12 or vx*vy!=0:
            raise ValueError('one bounded translation axis per pulse; no turn')
        if operation!=1 and (vx!=0 or vy!=0):raise ValueError('non-move command must be zero')
        if not self.owner:
            if operation!=0 or not self.enabled or not legacy_stopped or session in self.used:
                raise ValueError('base acquisition disabled, busy or replayed')
            self.owner=session;self.phase='ALIGN';self.used.add(session);self.command_id=0;self.received=now
        if session!=self.owner:raise ValueError('different base owner')
        if command_id<=self.command_id:raise ValueError('replayed base command')
        # Expiry must latch even if a command callback runs before the timer.
        self.tick(now)
        if operation==4:
            if not operator_confirmed:raise ValueError('operator stop/hand clearance confirmation required')
            self.owner='';self.phase='IDLE';self.velocity=(0.,0.);self.diagnostic='released by operator'
        elif self.phase=='FAULT':
            if operation not in (2,3):raise ValueError('base fault latched; explicit operator release required')
            self.velocity=(0.,0.)  # Acknowledge stop requests without clearing the latch.
        elif operation==1:
            if self.phase=='LOCKED':raise ValueError('base locked for arm execution')
            self.velocity=(vx,vy)
        elif operation in (0,2,3):
            self.velocity=(0.,0.)
            if operation==3:self.phase='LOCKED'
        self.command_id=command_id;self.received=now

    def tick(self, now):
        if self.owner and self.received is not None and now-self.received>self.timeout:
            self.fault('micro command watchdog expired')
        return self.velocity if self.owner and self.phase=='ALIGN' else (0.,0.)
