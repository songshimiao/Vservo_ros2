# 视觉观测统一返回 camera_T_target、6×6 协方差、目标 ID 和像素重投影 RMS。
# 标定板尺寸使用米，K 为像素内参；平面 PnP 会检查正深度、歧义和重投影误差。
"""Metric ChArUco and single-marker pose observations."""
from dataclasses import dataclass
import cv2
import numpy as np
from .geometry import transform


# 检测插件共同的数据契约；协方差前三维对应平移，后三维对应转动。
@dataclass
class Observation:
    camera_T_target: np.ndarray
    covariance: np.ndarray
    target_id: str
    reprojection_px: float


# ChArUco 标定板模型：ArUco 用于识别角点编号，棋盘交点用于亚像素几何测量。
class Board:
    # 按实物格数、格长和标记边长生成板模型；这些参数必须与打印实物一致。
    def __init__(self, squares_x=7, squares_y=5, square_m=0.015,
                 marker_m=0.011, dictionary='DICT_4X4_50'):
        if squares_x < 3 or squares_y < 3 or not 0 < marker_m < square_m:
            raise ValueError('Invalid physical board dimensions')
        self.spec = dict(squares_x=squares_x, squares_y=squares_y, square_m=square_m,
                         marker_m=marker_m, dictionary=dictionary)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, self.dictionary)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

    # 转灰度、找 ArUco，再插值得到 ChArUco 角点与编号；可传内参帮助插值。
    def charuco(self, image, K=None, D=None):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            raise ValueError('No ArUco markers detected')
        _, c, ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, cameraMatrix=K, distCoeffs=D)
        return c, ids

    # 至少接受 8 个且不共线的棋盘角点，按 ID 取相应的米制三维板上坐标。
    def corners(self, image, K=None, D=None):
        c, ids = self.charuco(image, K, D)
        if c is None or len(c) < 8:
            raise ValueError('Need at least 8 ChArUco corners')
        obj = self.board.getChessboardCorners()[ids.reshape(-1)]
        if np.linalg.matrix_rank(obj - obj.mean(axis=0), tol=1e-7) < 2:
            raise ValueError('Collinear corners')
        return obj.astype(np.float32), c.reshape(-1, 2).astype(np.float32), ids

    # 用板上点与像素点求平面位姿，输出与单标记检测相同的 Observation。
    def detect(self, image, K, D):
        obj, pix, _ = self.corners(image, K, D)
        return planar_pose(obj, pix, K, D, cv2.SOLVEPNP_IPPE, 'charuco_board')


# 单个指定 ID 的米制位姿检测；不能用未知尺寸的标记恢复绝对距离。
class ArucoMarker:
    # 创建正方形四角的三维模型，角点顺序遵守 IPPE_SQUARE 要求，开启亚像素精修。
    def __init__(self, marker_id, marker_m, dictionary='DICT_4X4_50'):
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if not 0 <= marker_id < len(self.dictionary.bytesList) or not np.isfinite(marker_m) or marker_m <= 0:
            raise ValueError('Invalid ArUco ID or physical side length')
        self.marker_id = marker_id
        self.target_id = f'aruco_{marker_id}'
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        h = marker_m / 2
        self.points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    # 必须恰好找到一个指定 ID；求解后翻转 Y/Z，使目标轴统一为右、下、指入标记。
    def detect(self, image, K, D):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        matches = np.flatnonzero(ids.ravel() == self.marker_id) if ids is not None else []
        if len(matches) != 1:
            raise ValueError(f'Need exactly one ArUco ID {self.marker_id}')
        obs = planar_pose(self.points, corners[matches[0]].reshape(4, 2), K, D,
                          cv2.SOLVEPNP_IPPE_SQUARE, self.target_id)
        # Use the same right/down/into-the-target axes as the ChArUco servo frame.
        obs.camera_T_target[:3, :3] = obs.camera_T_target[:3, :3] @ np.diag([1, -1, -1])
        return obs


# 先枚举平面 PnP 候选解，再拒绝歧义，最后只接受误差不增大的 LM 精修。
def planar_pose(obj, pix, K, D, method, target_id):
    # 把板上三维点投回图像，计算每点二维欧氏误差的 RMS（像素）。
    def reprojection(r, t):
        projected = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((projected - pix) ** 2, axis=1))))

    # 枚举候选位姿，只保留所有点在相机前方、数值有限的解，并按重投影误差排序。
    def solutions(flags):
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, pix, K, D, flags=flags)
        candidates = []
        if ok:
            for r, t in zip(rvecs, tvecs):
                if np.isfinite(r).all() and np.isfinite(t).all() and np.min((cv2.Rodrigues(r)[0] @ obj.T + t)[2]) > 0:
                    rms = reprojection(r, t)
                    if np.isfinite(rms):
                        candidates.append((rms, r, t))
        return sorted(candidates, key=lambda item: item[0])

    candidates = solutions(method)
    # OpenCV 4.10 IPPE_SQUARE can return high-error rotations at exactly frontal
    # views. SQPnP uses the same measured points; depth/ambiguity gates still apply.
    if method == cv2.SOLVEPNP_IPPE_SQUARE and (not candidates or candidates[0][0] > 2.0):
        candidates = solutions(cv2.SOLVEPNP_SQPNP)
    if not candidates:
        raise ValueError('No positive-depth PnP solution')
    rms, r, t = candidates[0]
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.05:
        relative = cv2.Rodrigues(r)[0].T @ cv2.Rodrigues(candidates[1][1])[0]
        if np.linalg.norm(cv2.Rodrigues(relative)[0]) > np.deg2rad(5):
            raise ValueError('Ambiguous planar pose; tilt board / increase coverage')
    refined_r, refined_t = cv2.solvePnPRefineLM(obj, pix, K, D, r.copy(), t.copy())
    refined_rms = reprojection(refined_r, refined_t)
    # Near a front-facing marker, LM may leave a good IPPE solution. Accept
    # refinement only when it improves reprojection and keeps every point in front.
    if (np.isfinite(refined_r).all() and np.isfinite(refined_t).all() and np.isfinite(refined_rms)
            and refined_rms <= rms
            and np.min((cv2.Rodrigues(refined_r)[0] @ obj.T + refined_t)[2]) > 0):
        rms, r, t = refined_rms, refined_r, refined_t
    if not np.isfinite(rms) or rms > 2.0 or np.min((cv2.Rodrigues(r)[0] @ obj.T + t)[2]) <= 0:
        raise ValueError('Refined pose fails positive depth or 2 px reprojection gate')
    # 此协方差是固定工程假设（3 mm、2° 标准差），不是从当前图像拟合出的精度结论。
    covariance = np.diag([0.003**2]*3 + [np.deg2rad(2)**2]*3)
    return Observation(transform(cv2.Rodrigues(r)[0], t), covariance, target_id, rms)
