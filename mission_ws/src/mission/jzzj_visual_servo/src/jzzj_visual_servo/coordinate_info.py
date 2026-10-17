"""Format read-only arm-frame coordinate diagnostics; never commands hardware."""
import math


def coordinate_info(frame, stamp, track_id, label, color, surface, profile_name, profile):
    """Distinguish observed object surface from a calibrated candidate TCP grasp point."""
    if len(surface) != 3 or not all(math.isfinite(v) for v in surface):
        raise ValueError("finite three-dimensional arm-frame position required")
    if not frame or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError("frame and valid source timestamp required")
    # Keep each update on one physical log line even if external labels contain whitespace.
    clean = lambda value: " ".join(str(value).split())
    prefix = ("[ARM_TARGET] frame=%s stamp=%.6f id=%s label=%s color=%s "
              "surface_m=(X=%+.4f,Y=%+.4f,Z=%+.4f)" %
              (clean(frame), stamp, clean(track_id), clean(label), clean(color), *surface))
    offset = profile.get("surface_to_grasp", [])
    if (not profile.get("calibrated", False) or len(offset) != 3 or
            not all(math.isfinite(v) for v in offset)):
        return prefix + " grasp_tcp_m=UNAVAILABLE(profile_not_calibrated) candidate_only=true"
    grasp = tuple(a+b for a,b in zip(surface, offset))
    if not all(math.isfinite(v) for v in grasp):
        raise ValueError("nonfinite candidate grasp point")
    return prefix + (" grasp_tcp_m=(X=%+.4f,Y=%+.4f,Z=%+.4f) profile=%s "
                     "candidate_only=true" % (*grasp, clean(profile_name)))
