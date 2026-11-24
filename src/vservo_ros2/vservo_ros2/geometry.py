# 阅读约定：A_T_B 把 B 系中的点转换到 A 系；连乘时中间坐标系必须相同。
# 本工程内部使用米和弧度；ROS 四元数顺序为 x、y、z、w。此模块不访问硬件。
"""All T = parent_T_child, translation metres, rotation radians internally."""
import json
from pathlib import Path
import os
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation


# 用旋转矩阵 R 和平移向量 t 构造 4×4 齐次变换，省略部分取单位变换，并检查 SE(3) 合法性。
def transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t).reshape(3)
    return validate(T)


# 检查形状、有限数、齐次末行、旋转正交性及 det(R)=+1；拒绝缩放、反射和坏数据。
def validate(T):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Invalid SE(3) matrix')
    return T


# 刚体逆变换为 [Rᵀ, -Rᵀt]，将 parent_T_child 转为 child_T_parent。
def inverse(T):
    T = validate(T)
    return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


# 返回 a 与 b 的平移距离（米）、相对旋转角（弧度）；这里只给误差大小，不给方向。
def error(a, b):
    d = inverse(a) @ validate(b)
    return float(np.linalg.norm(d[:3, 3])), float(Rotation.from_matrix(d[:3, :3]).magnitude())


# 平移取算术均值，旋转使用 SciPy 的旋转均值，避免直接平均矩阵破坏正交性。
def mean_pose(poses):
    return transform(Rotation.from_matrix(np.array(poses)[:, :3, :3]).mean().as_matrix(),
                     np.mean(np.array(poses)[:, :3, 3], axis=0))


# ROS Pose → 变换矩阵；先检查四元数已归一化，避免静默接受非法姿态。
def pose_matrix(p):
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('Quaternion must be normalized')
    return transform(Rotation.from_quat(q).as_matrix(), [p.position.x, p.position.y, p.position.z])


# 将合法变换写入传入的 ROS Pose，返回同一消息对象。
def fill_pose(p, T):
    T = validate(T)
    p.position.x, p.position.y, p.position.z = map(float, T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(T[:3, :3]).as_quat())
    return p


# 读取 TransformStamped 的 translation/rotation，转换成 parent_T_child。
def tf_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return transform(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])


# 把矩阵写入 TF 消息的变换字段；frame_id、child_frame_id 和时间戳由调用者设置。
def fill_tf(msg, T):
    T = validate(T)
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T[:3, 3])
    q = msg.transform.rotation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(T[:3, :3]).as_quat())
    return msg


# 先写同目录临时文件再原子替换；replace=False 通过硬链接避免覆盖已有标定。
# 禁止序列化 NaN/Inf；异常或中断后清理临时文件，尽量避免产生半份标定数据。
def write_json(path, data, *, replace=True):
    """Atomic replacement; do not leave a half-written calibration on interruption."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=str(path.parent), prefix='.' + path.name)
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
            f.write('\n')
        if replace:
            os.replace(temp, path)
        else:
            os.link(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


# 展开用户目录并读取 JSON；字段含义与标定兼容性由上层检查。
def read_json(path):
    return json.loads(Path(path).expanduser().read_text())
