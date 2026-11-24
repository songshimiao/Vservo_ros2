"""Independent reference accuracy and fixed-target repeatability are separate."""
import numpy as np
from .geometry import validate, mean_pose, error
from .calibration import stats


def report(poses, references=None):
    poses = [validate(p) for p in poses]
    if len(poses) < 5:
        raise ValueError('Need >=5 observations')
    if references is None:
        ref = mean_pose(poses)
        pairs = [error(ref, p) for p in poses]
        kind = 'fixed_target_repeatability_only'
    else:
        if len(references) != len(poses):
            raise ValueError('Reference count mismatch')
        pairs = [error(validate(r), p) for r, p in zip(references, poses)]
        kind = 'absolute_accuracy_against_independent_reference'
    return dict(kind=kind, samples=len(poses), translation_mm=stats([p[0]*1000 for p in pairs]),
                rotation_deg=stats([np.degrees(p[1]) for p in pairs]))
