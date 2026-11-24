"""Metric ChArUco and single-marker pose observations."""
from dataclasses import dataclass
import cv2
import numpy as np
from .geometry import transform


@dataclass
class Observation:
    camera_T_target: np.ndarray
    covariance: np.ndarray
    target_id: str
    reprojection_px: float


class Board:
    def __init__(self, squares_x=7, squares_y=5, square_m=0.015,
                 marker_m=0.011, dictionary='DICT_4X4_50'):
        if squares_x < 3 or squares_y < 3 or not 0 < marker_m < square_m:
            raise ValueError('Invalid physical board dimensions')
        self.spec = dict(squares_x=squares_x, squares_y=squares_y, square_m=square_m,
                         marker_m=marker_m, dictionary=dictionary)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, self.dictionary)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

    def charuco(self, image, K=None, D=None):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            raise ValueError('No ArUco markers detected')
        _, c, ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, cameraMatrix=K, distCoeffs=D)
        return c, ids

    def corners(self, image, K=None, D=None):
        c, ids = self.charuco(image, K, D)
        if c is None or len(c) < 8:
            raise ValueError('Need at least 8 ChArUco corners')
        obj = self.board.getChessboardCorners()[ids.reshape(-1)]
        if np.linalg.matrix_rank(obj - obj.mean(axis=0), tol=1e-7) < 2:
            raise ValueError('Collinear corners')
        return obj.astype(np.float32), c.reshape(-1, 2).astype(np.float32), ids

    def detect(self, image, K, D):
        obj, pix, _ = self.corners(image, K, D)
        return planar_pose(obj, pix, K, D, cv2.SOLVEPNP_IPPE, 'charuco_board')


class ArucoMarker:
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


def planar_pose(obj, pix, K, D, method, target_id):
    def reprojection(r, t):
        projected = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((projected - pix) ** 2, axis=1))))

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
    covariance = np.diag([0.003**2]*3 + [np.deg2rad(2)**2]*3)
    return Observation(transform(cv2.Rodrigues(r)[0], t), covariance, target_id, rms)
