"""Pure validation/conversion shared by the trajectory node and offline tests."""
import math

def validate_mapping(signs,offsets,lower,upper):
    if any(len(v)!=6 for v in (signs,offsets,lower,upper)):
        raise ValueError('six explicitly calibrated joints required')
    if any(s not in (-1,1) for s in signs) or not all(math.isfinite(v) for vec in (offsets,lower,upper) for v in vec):
        raise ValueError('invalid signs, zero offsets or joint limits')
    if any(lo>=hi for lo,hi in zip(lower,upper)):raise ValueError('invalid joint range')

def encode(joints,signs,offsets):
    return [math.degrees(q)*s+o for q,s,o in zip(joints,signs,offsets)]

def decode(raw,signs,offsets):
    return [math.radians((v-o)*s) for v,s,o in zip(raw,signs,offsets)]

def validate_target(names,positions,expected,lower,upper,check_limits=True):
    if len(names)!=6 or len(set(names))!=6 or set(names)!=set(expected) or len(positions)!=6:
        raise ValueError('exactly the six configured unique joint names required')
    values=dict(zip(names,positions));ordered=[values[n] for n in expected]
    if not all(math.isfinite(v) and (not check_limits or lo<=v<=hi) for v,lo,hi in zip(ordered,lower,upper)):
        raise ValueError('nonfinite or out-of-range joint position')
    return ordered
