"""Project-local SDK boundary: vendor code must not swallow bus failures."""


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
    def __init__(self, lr='left', factory=None):
        if factory is None:
            from Arm_Lib import Arm_Device
            factory = Arm_Device
        self.device = factory(lr=lr)
        self.bus = CheckedBus(self.device.bus)
        self.device.bus = self.bus

    def _call(self, name, *args):
        # The trajectory driver serializes each complete SDK operation with its I/O lock.
        self.bus.error = None
        result = getattr(self.device, name)(*args)
        if self.bus.error is not None:
            raise OSError('{}: I2C operation failed'.format(name)) from self.bus.error
        return result

    def Arm_serial_servo_read(self, servo_id):
        value = self._call('Arm_serial_servo_read', servo_id)
        if value is None:
            raise OSError('servo {} returned no feedback'.format(servo_id))
        return value

    def Arm_serial_servo_write6(self, *args):
        return self._call('Arm_serial_servo_write6', *args)

    def Arm_serial_set_torque(self, enabled):
        return self._call('Arm_serial_set_torque', enabled)
