"""MDH parameters copied from fr_visual_servo; no RTB dependency required."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .vendor.fairino_dh_params import get_mdh_params, DEFAULT_QLIM_RAD
from .geometry import error, validate

_worker_model = None


def warmup_ik_worker():
    global _worker_model
    if _worker_model is None:
        _worker_model = Kinematics()
    return True


def solve_ik_worker(target, seed):
    warmup_ik_worker()
    return _worker_model.ik(target, seed)


def check_home_path_worker(start, home, minimum, maximum):
    from .control import in_workspace
    warmup_ik_worker()
    for q in np.linspace(start, home, 81):
        if (np.any(q <= _worker_model.limits[:, 0]) or np.any(q >= _worker_model.limits[:, 1])
                or not in_workspace(_worker_model.fk(q), minimum, maximum)):
            raise ValueError('Reset joint path leaves configured joint / workspace limits')
    return True


class Kinematics:
    def __init__(self):
        self.links = get_mdh_params('FR5')
        self.limits = np.array(DEFAULT_QLIM_RAD) + np.array([np.deg2rad(3), -np.deg2rad(3)])

    def fk(self, q):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Need six finite joint angles in radians')
        T = np.eye(4)
        for theta, p in zip(q, self.links):
            ct, st = np.cos(theta+p.theta_offset), np.sin(theta+p.theta_offset)
            ca, sa = np.cos(p.alpha), np.sin(p.alpha)
            T = T @ np.array([[ct, -st, 0, p.a], [st*ca, ct*ca, -sa, -p.d*sa],
                               [st*sa, ct*sa, ca, p.d*ca], [0, 0, 0, 1]])
        return T

    def ik(self, target, seed):
        target = validate(target)
        seed = np.asarray(seed, float)
        if np.any(seed <= self.limits[:, 0]) or np.any(seed >= self.limits[:, 1]):
            raise ValueError('Robot joints outside configured safety margins')
        def residual(q):
            T = self.fk(q)
            return np.r_[T[:3, 3]-target[:3, 3],
                         Rotation.from_matrix(target[:3, :3].T @ T[:3, :3]).as_rotvec() * 0.3]
        sol = least_squares(residual, seed, bounds=(self.limits[:, 0], self.limits[:, 1]),
                            max_nfev=60, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        t, a = error(self.fk(sol.x), target)
        if not sol.success or t > 0.001 or a > np.deg2rad(0.5):
            raise ValueError('IK failed reachability / residual gate')
        return sol.x
