"""Servo position units (not degrees): reject outside 96..4000, never clip."""
import math

POSITION_MIN = 96
POSITION_MAX = 4000


def validate_position_range(lower, upper):
    if (type(lower) is not int or type(upper) is not int or
            not POSITION_MIN <= lower < upper <= POSITION_MAX):
        raise ValueError('servo position range must be inside 96..4000')


def validate_positions(values, lower=POSITION_MIN, upper=POSITION_MAX):
    validate_position_range(lower, upper)
    if not values or any(type(v) is not int or not lower <= v <= upper for v in values):
        raise ValueError('servo positions must be integers in {}..{}; command not sent'.format(lower, upper))
    return tuple(values)


def validate_servo_id(servo_id):
    if type(servo_id) is not int or not 1 <= servo_id <= 6:
        raise ValueError('servo id must be 1..6')


def angle_to_position(servo_id, degrees, left=True):
    validate_servo_id(servo_id)
    if not isinstance(degrees, (int, float)) or isinstance(degrees, bool) or not math.isfinite(degrees):
        raise ValueError('finite servo angle required')
    # Keep the installed SDK's calibration and left/right signs, without clip().
    if servo_id in (1, 5):
        value = 380.+(degrees+135.)*3320./270.
    elif servo_id in (2, 3, 4):
        value = 900.+(90.-degrees if left else degrees+90.)*2200./180.
    else:
        value = 900.+(180.-degrees)*2200./180.
    if not math.isfinite(value):
        raise ValueError('nonfinite encoded servo position')
    return int(math.floor(value+.5))  # Quantize to one position unit; never saturate.


def position_to_angle(servo_id, position, left=True):
    validate_servo_id(servo_id)
    if servo_id in (1, 5):
        return (position-380.)*270./3320.-135.
    value = (position-900.)*180./2200.
    if servo_id in (2, 3, 4):
        return 90.-value if left else value-90.
    return 180.-value


def duration_bytes(milliseconds):
    if type(milliseconds) is not int or not 1 <= milliseconds <= 65535:
        raise ValueError('duration must be integer milliseconds in 1..65535')
    return [(milliseconds >> 8) & 255, milliseconds & 255]


def joint_limits(signs, offsets, lower=POSITION_MIN, upper=POSITION_MAX, left=True):
    validate_position_range(lower, upper)
    if (len(signs) != 6 or len(offsets) != 6 or any(s not in (-1, 1) for s in signs) or
            not all(math.isfinite(v) for v in offsets)):
        raise ValueError('six calibrated signs and offsets required')
    low = []; high = []
    for i, (sign, offset) in enumerate(zip(signs, offsets), 1):
        values = [math.radians((position_to_angle(i, p, left)-offset)*sign) for p in (lower, upper)]
        low.append(min(values)); high.append(max(values))
    return low, high
