# 内参与手眼的纯数学求解及验证。训练/留出拆分用于检查拟合，独立会话验证由 CLI 完成。
# EIH 求 flange_T_camera，板相对 base 固定；ETH 求 base_T_camera，板相对 flange 固定。
"""Calibration math shared by ROS service and offline regression tools."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import validate, inverse, transform, mean_pose, error


# 每第 4 个样本留出，其余训练；样本数不足时直接拒绝求解。
def split_indices(n, minimum=12):
    if n < minimum:
        raise ValueError(f'Need >= {minimum} samples; recommend 25-40 diverse samples')
    held = list(range(3, n, 4))
    train = [i for i in range(n) if i not in held]
    return train, held


# 汇总均值、RMS、95 分位和最大值；单位沿用输入，拒绝空集合及非有限数据。
def stats(values):
    v = np.asarray(values, float)
    if v.size == 0 or not np.isfinite(v).all():
        raise ValueError('Empty/nonfinite evaluation data')
    return dict(mean=float(v.mean()), rms=float(np.sqrt(np.mean(v**2))),
                p95=float(np.percentile(v, 95)), maximum=float(v.max()))


# 用待验证内参求每帧板位姿，再计算图像重投影 RMS（像素）。
def reprojection(obj, pix, K, D):
    ok, r, t = cv2.solvePnP(obj, pix, K, D)
    if not ok or not np.isfinite(t).all() or t[2, 0] <= 0:
        raise ValueError('Cannot estimate validation pose')
    q = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((q-pix.reshape(-1, 2))**2, axis=1))))


# 检查样本数和双轴图像覆盖，用训练帧求 K/D，再用留出帧检验最大重投影误差。
def intrinsic(observations, size, max_rms_px=1.0):
    train, held = split_indices(len(observations), 16)
    objects = [np.asarray(x[0], np.float32) for x in observations]
    pixels = [np.asarray(x[1], np.float32) for x in observations]
    centers = np.array([x.mean(axis=0) / size for x in pixels])
    if np.any(np.ptp(centers, axis=0) < 0.18):
        raise ValueError('Insufficient image coverage: move board across both axes')
    rms, K, D, _, _ = cv2.calibrateCamera(
        [objects[i] for i in train], [pixels[i] for i in train], tuple(size), None, None)
    if not np.isfinite(K).all() or not np.isfinite(D).all() or min(K[0, 0], K[1, 1]) <= 0:
        raise ValueError('Invalid intrinsic solution')
    validation = [reprojection(objects[i], pixels[i], K, D) for i in held]
    if not np.isfinite(rms) or rms > max_rms_px or max(validation) > max_rms_px:
        raise ValueError(f'Intrinsic RMS gate failed: train={rms:.3f}, held={validation}')
    return dict(source='charuco_calibrated', image_size=list(size), camera_matrix=K.tolist(),
                dist_coeffs=D.reshape(-1).tolist(), train_rms_px=float(rms),
                heldout_rms_px=stats(validation), train_indices=train, heldout_indices=held,
                validated=True)


# 对相对旋转向量做 SVD，要求至少两个独立旋转方向，避免退化手眼运动。
def diverse(poses):
    rotations = np.array([Rotation.from_matrix(inverse(poses[0])[:3, :3] @ p[:3, :3]).as_rotvec()
                          for p in poses[1:]])
    singular = np.linalg.svd(rotations, compute_uv=False)
    if len(singular) < 2 or singular[1] < 0.15 or np.max(np.linalg.norm(rotations, axis=1)) < 0.35:
        raise ValueError('Degenerate robot motion: rotate about >=2 independent axes by >=20 degrees')
    return singular.tolist()


# 按安装方式构造应保持不变的板位姿：EIH 的 base_T_board 或 ETH 的 flange_T_board。
def invariants(robot, camera, X, mode):
    if mode == 'eih':
        return [g @ X @ c for g, c in zip(robot, camera)]  # base_T_board constant
    return [inverse(g) @ X @ c for g, c in zip(robot, camera)]  # flange_T_board constant


# 将各帧不变量与固定参考比较，返回平移（米）和旋转（度）的统计。
def evaluate_handeye(robot, camera, X, mode, reference):
    pairs = [error(reference, p) for p in invariants(robot, camera, X, mode)]
    return dict(translation_m=stats([p[0] for p in pairs]),
                rotation_deg=stats([np.degrees(p[1]) for p in pairs]))


# PARK 法求手眼；ETH 先反转机器人位姿，复用 OpenCV 的 EIH 形式。
# 训练与留出集都满足一致性门槛后，才输出 validated=True 的结果与坐标系元数据。
def handeye(robot, camera, mode, max_translation_m=0.01, max_rotation_deg=2.0):
    if mode not in ('eih', 'eth') or len(robot) != len(camera):
        raise ValueError('Mode or paired samples invalid')
    robot, camera = [validate(x) for x in robot], [validate(x) for x in camera]
    train, held = split_indices(len(robot), 16)
    singular = diverse([robot[i] for i in train])
    gs = [robot[i] if mode == 'eih' else inverse(robot[i]) for i in train]
    cs = [camera[i] for i in train]
    R, t = cv2.calibrateHandEye([g[:3, :3] for g in gs], [g[:3, 3] for g in gs],
                               [c[:3, :3] for c in cs], [c[:3, 3] for c in cs],
                               method=cv2.CALIB_HAND_EYE_PARK)
    X = transform(R, t)
    ref = mean_pose(invariants([robot[i] for i in train], cs, X, mode))
    reports = {}
    for name, ids in [('train', train), ('heldout', held)]:
        reports[name] = evaluate_handeye([robot[i] for i in ids], [camera[i] for i in ids], X, mode, ref)
        if (reports[name]['translation_m']['maximum'] > max_translation_m
                or reports[name]['rotation_deg']['maximum'] > max_rotation_deg):
            raise ValueError(f'Hand-eye consistency gate failed: {name}: {reports[name]}')
    return dict(mode=mode, parent_frame='flange' if mode == 'eih' else 'base',
                child_frame=f'{mode}_optical_frame', matrix=X.tolist(), units='m',
                method='PARK', reference_matrix=ref.tolist(), validated=True,
                train_indices=train, heldout_indices=held, rotation_singular_values=singular,
                reports=reports)


# 检查序列号、安装角色、分辨率和 K/D 数值，避免交叉使用另一台相机的内参。
def validate_intrinsic(data, role, serial, size):
    meta = data['camera']
    if (meta['serial'] != serial or data['image_size'] != list(size)
            or meta['role'] != {'eih': 'eye_in_hand', 'eth': 'eye_to_hand'}[role]):
        raise ValueError('Intrinsic serial / role / resolution mismatch')
    K, D = np.array(data['camera_matrix'], float), np.array(data['dist_coeffs'], float)
    if K.shape != (3, 3) or D.size not in (4, 5, 8, 12, 14) or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError('Invalid camera parameters')
    if min(K[0, 0], K[1, 1]) <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError('Invalid camera matrix')
    return K, D
