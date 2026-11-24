import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import validate, inverse, transform, error


def validate_motion_limits(linear, angular, acceleration):
    if (not np.isfinite([linear, angular, acceleration]).all() or not 0 < linear <= 0.2
            or not 0 < angular <= np.deg2rad(60) or not 0 < acceleration <= 2.0):
        raise ValueError('线速度应为 0–200 mm/s，角速度为 0–60°/s，加速度为 0–2 rad/s²，均须大于 0')


def cartesian_velocity_scale(model, q, velocity, period, linear_speed, angular_speed):
    translation, rotation = error(model.fk(q), model.fk(q+velocity*period))
    return min(1.0, linear_speed*period/max(translation, 1e-12),
               angular_speed*period/max(rotation, 1e-12))


def step(current, desired, max_translation=0.001, max_rotation=0.01, gain=0.2):
    current, desired = validate(current), validate(desired)
    dp = (desired[:3, 3]-current[:3, 3]) * gain
    dr = Rotation.from_matrix(current[:3, :3].T @ desired[:3, :3]).as_rotvec() * gain
    dp *= min(1, max_translation/max(np.linalg.norm(dp), 1e-12))
    dr *= min(1, max_rotation/max(np.linalg.norm(dr), 1e-12))
    return transform(current[:3, :3] @ Rotation.from_rotvec(dr).as_matrix(), current[:3, 3]+dp)


def target_flange(base_T_target, flange_T_camera, distance):
    # OpenCV board normal +Z points through board; camera sees its front from -Z.
    target_T_camera = transform(t=[0, 0, -distance])
    return validate(base_T_target) @ target_T_camera @ inverse(flange_T_camera)


def in_workspace(T, minimum, maximum):
    p = validate(T)[:3, 3]
    lo, hi = np.asarray(minimum), np.asarray(maximum)
    return bool(lo.shape == (3,) and hi.shape == (3,) and np.all(lo < hi)
                and np.all(p >= lo) and np.all(p <= hi))


class JointLimiter:
    """Bound commanded joint velocity/acceleration on the SDK nominal period.

    Braking distance reduces speed near goal. Small discrete overshoot is allowed
    rather than breaking the acceleration bound by snapping to a new endpoint.
    """
    def __init__(self, q, max_speed, max_acceleration, period=0.01):
        if max_speed <= 0 or max_acceleration <= 0 or period <= 0:
            raise ValueError('Speed, acceleration and period must be positive')
        self.q = np.array(q, float).copy()
        self.v = np.zeros_like(self.q)
        self.max_speed, self.acceleration, self.period = max_speed, max_acceleration, period

    def desired_velocity(self, goal):
        distance = np.asarray(goal)-self.q
        return np.sign(distance)*np.minimum(self.max_speed, np.sqrt(2*self.acceleration*np.abs(distance)))

    def step(self, goal, velocity_scale=1.0):
        desired_v = self.desired_velocity(goal)*velocity_scale
        self.v += np.clip(desired_v-self.v, -self.acceleration*self.period, self.acceleration*self.period)
        self.q = self.q+self.v*self.period
        return self.q.copy()
