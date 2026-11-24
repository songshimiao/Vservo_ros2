# FR5 数值运动学：FK 用改进 DH 连乘，IK 用带关节边界的最小二乘。
# 输入关节角为弧度、矩阵平移为米；规划 worker 与 100 Hz SDK 控制线程隔离。
"""MDH parameters copied from fr_visual_servo; no RTB dependency required."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .vendor.fairino_dh_params import get_mdh_params, DEFAULT_QLIM_RAD
from .geometry import error, validate

_worker_model = None


# 在规划进程内懒初始化一个运动学模型，重复任务复用它，减少控制开始后的初始化开销。
def warmup_ik_worker():
    global _worker_model
    if _worker_model is None:
        _worker_model = Kinematics()
    return True


# 进程池入口：以 seed 为初值求解完整法兰目标的六个关节角。
def solve_ik_worker(target, seed):
    warmup_ik_worker()
    return _worker_model.ik(target, seed)


# 在起始角与复位角之间检查 81 个插值点的关节和法兰工作空间边界。
# 这是模型采样预检查，不包含实体障碍物碰撞规划；执行时仍逐步检查边界。
def check_home_path_worker(start, home, minimum, maximum):
    from .control import in_workspace
    warmup_ik_worker()
    for q in np.linspace(start, home, 81):
        if (np.any(q <= _worker_model.limits[:, 0]) or np.any(q >= _worker_model.limits[:, 1])
                or not in_workspace(_worker_model.fk(q), minimum, maximum)):
            raise ValueError('Reset joint path leaves configured joint / workspace limits')
    return True


# 轻量 FR5 模型，不依赖旧工程的 Robotics Toolbox。
class Kinematics:
    # 装载 FR5 的 MDH 参数，并在厂家关节限位两端各留出 3° 裕度。
    def __init__(self):
        self.links = get_mdh_params('FR5')
        self.limits = np.array(DEFAULT_QLIM_RAD) + np.array([np.deg2rad(3), -np.deg2rad(3)])

    # 六轴弧度 → base_T_flange；按关节次序右乘各节 MDH 变换。
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

    # 用有界数值优化寻找邻近 seed 的解，最终还须满足 1 mm / 0.5° 的残差门槛。
    def ik(self, target, seed):
        target = validate(target)
        seed = np.asarray(seed, float)
        if np.any(seed <= self.limits[:, 0]) or np.any(seed >= self.limits[:, 1]):
            raise ValueError('Robot joints outside configured safety margins')
        # 残差拼接平移差与旋转向量差；0.3 是两类残差的数值权重，不是单位转换。
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
