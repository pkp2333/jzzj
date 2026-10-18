"""Project-local I2C boundary: reject bad positions and propagate bus errors."""
import time
import logging
from dataclasses import dataclass
from servo_protocol import (POSITION_MIN, POSITION_MAX,
    FEEDBACK_POSITION_MIN, FEEDBACK_POSITION_MAX, validate_position_range,
    validate_servo_id, angle_to_position, position_to_angle, duration_bytes)

LOGGER=logging.getLogger(__name__)


@dataclass(frozen=True)
class CommandResult:
    sent_ids: tuple
    skipped_ids: tuple

    @property
    def complete(self):return not self.skipped_ids


class CheckedBus:
    def __init__(self, bus):
        self.bus = bus
        self.error = None

    def __getattr__(self, name):
        value = getattr(self.bus, name)
        if not callable(value):
            return value

        def call(*args, **kwargs):
            try:
                return value(*args, **kwargs)
            except Exception as exc:
                self.error = exc
                raise
        return call


class CheckedArmDevice:
    def __init__(self, lr='left', factory=None, position_min=POSITION_MIN, position_max=POSITION_MAX):
        if lr not in ('l','L','left','Left','LEFT','r','R','right','Right','RIGHT'):
            raise ValueError('explicit left/right arm required')
        validate_position_range(position_min,position_max)
        self.left=lr in ('l','L','left','Left','LEFT')
        self.addr=0x10 if self.left else 0x11
        self.position_min,self.position_max=position_min,position_max
        if factory is None:
            from Arm_Lib import Arm_Device
            factory = Arm_Device
        self.device = factory(lr=lr)
        if getattr(self.device,'addr',self.addr)!=self.addr:
            raise ValueError('SDK arm address mismatch')
        self.bus = CheckedBus(self.device.bus)
        self.device.bus = self.bus
        self.last_command_result=None

    def _call(self, name, *args):
        # The trajectory driver serializes each complete SDK operation with its I/O lock.
        self.bus.error = None
        result = getattr(self.device, name)(*args)
        if self.bus.error is not None:
            raise OSError('{}: I2C operation failed'.format(name)) from self.bus.error
        return result

    def Arm_serial_servo_read(self, servo_id):
        validate_servo_id(servo_id)
        self.bus.error=None
        self.bus.write_i2c_block_data(self.addr,0x37,[servo_id]);time.sleep(.003)
        packet=self.bus.read_i2c_block_data(self.addr,0x37);time.sleep(.1)
        if (packet is None or len(packet)<3 or
                any(type(v) is not int or not 0<=v<=255 for v in packet[:3])):
            raise OSError('servo {} malformed position feedback'.format(servo_id))
        position=(packet[1]<<8)|packet[2]
        if not FEEDBACK_POSITION_MIN<=position<=FEEDBACK_POSITION_MAX:
            raise OSError('servo {} invalid feedback position {}; expected {}..{}'.format(
                servo_id,position,FEEDBACK_POSITION_MIN,FEEDBACK_POSITION_MAX))
        return position_to_angle(servo_id,position,self.left)

    @property
    def servo_degree_limits(self):
        return [tuple(sorted(position_to_angle(i,p,self.left) for p in
                             (self.position_min,self.position_max))) for i in range(1,7)]

    def write_positions(self, positions, milliseconds):
        if len(positions)!=6:raise ValueError('six servo positions required')
        timing=duration_bytes(milliseconds)
        valid=[];skipped=[]
        for servo_id,position in enumerate(positions,1):
            if type(position) is int and self.position_min<=position<=self.position_max:
                valid.append((servo_id,position))
            else:skipped.append(servo_id)
        report=CommandResult(tuple(i for i,_ in valid),tuple(skipped))
        self.bus.error=None
        self.last_command_result=None
        if not skipped:
            data=[byte for _,p in valid for byte in ((p>>8)&255,p&255)]
            self.bus.write_i2c_block_data(self.addr,0x1e,timing)
            self.bus.write_i2c_block_data(self.addr,0x1d,data)
        else:
            # No command, including a replacement/hold target, goes to skipped
            # servo IDs. Individual writes preserve every valid target and time.
            for servo_id,position in valid:
                self.bus.write_i2c_block_data(self.addr,0x19,
                    [servo_id,(position>>8)&255,position&255]+timing)
        self.last_command_result=report
        if skipped:
            LOGGER.warning('servo command partial: sent %s; skipped %s (position %s..%s)',
                report.sent_ids,report.skipped_ids,self.position_min,self.position_max)
        return report

    def Arm_serial_servo_write6(self, *args):
        if len(args)!=7:raise ValueError('six angles and duration required')
        positions=[]
        for i,v in enumerate(args[:6],1):
            try:positions.append(angle_to_position(i,v,self.left))
            except (ValueError,TypeError,OverflowError):positions.append(None)
        return self.write_positions(positions,args[6])

    def Arm_serial_servo_write(self, servo_id, degrees, milliseconds=800):
        validate_servo_id(servo_id)
        timing=duration_bytes(milliseconds)
        try:position=angle_to_position(servo_id,degrees,self.left)
        except (ValueError,TypeError,OverflowError):position=None
        if type(position) is not int or not self.position_min<=position<=self.position_max:
            report=CommandResult((),(servo_id,));self.last_command_result=report
            LOGGER.warning('servo %s command skipped: position outside %s..%s',
                servo_id,self.position_min,self.position_max)
            return report
        self.bus.error=None
        self.last_command_result=None
        self.bus.write_i2c_block_data(self.addr,0x19,[servo_id,(position>>8)&255,position&255]+timing)
        time.sleep(.1)
        report=CommandResult((servo_id,),());self.last_command_result=report
        return report

    def Arm_serial_set_torque(self, enabled):
        return self._call('Arm_serial_set_torque', enabled)
