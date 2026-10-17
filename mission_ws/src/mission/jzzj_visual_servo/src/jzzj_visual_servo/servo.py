"""Position-only incremental servo primitives (no Twist / base command output)."""
import math

def limited_target(current,target,max_step):
    if max_step<=0 or not all(math.isfinite(v) for v in tuple(current)+tuple(target)):
        raise ValueError('invalid position servo input')
    delta=[b-a for a,b in zip(current,target)];norm=math.sqrt(sum(d*d for d in delta))
    scale=min(1.,max_step/max(norm,1e-12))
    return tuple(a+scale*d for a,d in zip(current,delta))
