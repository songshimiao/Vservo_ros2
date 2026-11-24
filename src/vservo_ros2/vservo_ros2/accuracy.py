# 评估输出明确区分：同一静止目标的重复性，以及与独立参考比较的绝对误差。
"""Independent reference accuracy and fixed-target repeatability are separate."""
import numpy as np
from .geometry import validate, mean_pose, error
from .calibration import stats


# 至少 5 个观测；无参考时以均值为中心，仅报告重复性；有逐帧参考时才报告绝对误差。
# 最终平移换算为 mm、旋转换算为度，统计指标由 calibration.stats 统一计算。
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
