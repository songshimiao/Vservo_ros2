# 纯控制数学：笛卡尔微步、目标法兰位姿、工作空间和关节速度/加速度限幅。
# 本模块只计算指令，不调用 SDK；实际下发和故障处理位于 robot_node.py。
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import validate, inverse, transform, error


# 核对线速度（m/s）、角速度（rad/s）、关节加速度（rad/s²）的有限性和范围。
def validate_motion_limits(linear, angular, acceleration):
    if (not np.isfinite([linear, angular, acceleration]).all() or not 0 < linear <= 0.2
            or not 0 < angular <= np.deg2rad(60) or not 0 < acceleration <= 2.0):
        raise ValueError('线速度应为 0–200 mm/s，角速度为 0–60°/s，加速度为 0–2 rad/s²，均须大于 0')


# 用 q 与 q+v·dt 的 FK 估计下一步平移和转角，返回满足两种笛卡尔速度上限的共同倍率。
def cartesian_velocity_scale(model, q, velocity, period, linear_speed, angular_speed):
    translation, rotation = error(model.fk(q), model.fk(q+velocity*period))
    return min(1.0, linear_speed*period/max(translation, 1e-12),
               angular_speed*period/max(rotation, 1e-12))


# 兼容预览用 PBVS 微步：位置差、旋转向量先乘增益，再各自限制单步大小。
# 当前桥接使用完整 desired 目标，此处的微步上限不能直接当作实体速度。
def step(current, desired, max_translation=0.001, max_rotation=0.01, gain=0.2):
    current, desired = validate(current), validate(desired)
    dp = (desired[:3, 3]-current[:3, 3]) * gain
    dr = Rotation.from_matrix(current[:3, :3].T @ desired[:3, :3]).as_rotvec() * gain
    dp *= min(1, max_translation/max(np.linalg.norm(dp), 1e-12))
    dr *= min(1, max_rotation/max(np.linalg.norm(dr), 1e-12))
    return transform(current[:3, :3] @ Rotation.from_rotvec(dr).as_matrix(), current[:3, 3]+dp)


# 目标系沿 -Z 后退 distance，先得到期望相机位置，再乘手眼逆变换得到法兰位置。
# 传单位手眼矩阵时，distance 直接表示法兰中心距目标的距离。
def target_flange(base_T_target, flange_T_camera, distance):
    # OpenCV board normal +Z points through board; camera sees its front from -Z.
    target_T_camera = transform(t=[0, 0, -distance])
    return validate(base_T_target) @ target_T_camera @ inverse(flange_T_camera)


# 检查法兰平移是否位于配置的三维轴对齐包围盒；不判断机械臂各连杆或障碍物。
def in_workspace(T, minimum, maximum):
    p = validate(T)[:3, 3]
    lo, hi = np.asarray(minimum), np.asarray(maximum)
    return bool(lo.shape == (3,) and hi.shape == (3,) and np.all(lo < hi)
                and np.all(p >= lo) and np.all(p <= hi))


# 保存上一拍指令角 q 和速度 v；按固定 SDK 周期连续推进，限制关节速度及加速度。
class JointLimiter:
    """Bound commanded joint velocity/acceleration on the SDK nominal period.

    Braking distance reduces speed near goal. Small discrete overshoot is allowed
    rather than breaking the acceleration bound by snapping to a new endpoint.
    """
    # 从当前关节角起步，初始速度为零；period 默认为 10 ms。
    def __init__(self, q, max_speed, max_acceleration, period=0.01):
        if max_speed <= 0 or max_acceleration <= 0 or period <= 0:
            raise ValueError('Speed, acceleration and period must be positive')
        self.q = np.array(q, float).copy()
        self.v = np.zeros_like(self.q)
        self.max_speed, self.acceleration, self.period = max_speed, max_acceleration, period

    # 按目标方向运动；靠近目标时以 sqrt(2·a·剩余距离) 降速，为制动留出距离。
    def desired_velocity(self, goal):
        distance = np.asarray(goal)-self.q
        return np.sign(distance)*np.minimum(self.max_speed, np.sqrt(2*self.acceleration*np.abs(distance)))

    # 先限制每拍速度变化为 a·dt，再积分位置；不直接跳到终点，因此可能有小量离散过冲。
    def step(self, goal, velocity_scale=1.0):
        desired_v = self.desired_velocity(goal)*velocity_scale
        self.v += np.clip(desired_v-self.v, -self.acceleration*self.period, self.acceleration*self.period)
        self.q = self.q+self.v*self.period
        return self.q.copy()
