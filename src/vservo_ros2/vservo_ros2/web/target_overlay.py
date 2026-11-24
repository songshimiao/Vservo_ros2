"""Display validated ID42 poses on the exact source image, outside ROS callbacks."""
import cv2
import numpy as np

from ..geometry import pose_matrix
from ..ros_utils import stamp_s

TARGET_MAX_AGE = 0.3


def tracking_id42(status, now):
    return bool(status and 0 <= now-status[0] <= TARGET_MAX_AGE
                and isinstance(status[1], str) and status[1].startswith('TRACKING aruco_42 '))


def matched_target_frame(history, observation, info, status, now, ros_now):
    if not observation or info is None or not tracking_id42(status, now):
        return None
    received, pose = observation
    if (not 0 <= now-received <= TARGET_MAX_AGE
            or not 0 <= ros_now-stamp_s(pose.header.stamp) <= TARGET_MAX_AGE):
        return None
    stamp = pose.header.stamp
    for frame_received, frame in reversed(history):
        if (frame.header.stamp.sec == stamp.sec and frame.header.stamp.nanosec == stamp.nanosec
                and frame.header.frame_id == pose.header.frame_id
                and (not info.header.frame_id or info.header.frame_id == frame.header.frame_id)
                and (frame.width, frame.height) == (info.width, info.height)):
            return frame_received, frame, pose
    return None


def draw_target_axes(bgr, pose, info, length=0.04):
    """X red, Y green, Z blue; use the servo target's into-marker +Z frame."""
    T = pose_matrix(pose.pose.pose)
    K, D = np.asarray(info.k, float).reshape(3, 3), np.asarray(info.d, float)
    if (not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0
            or bgr.shape[:2] != (info.height, info.width)):
        raise ValueError('Invalid preview intrinsics / image size')
    points = np.array([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]], float)
    if np.min((T[:3, :3] @ points.T + T[:3, 3:4])[2]) <= 0:
        raise ValueError('Preview axes behind camera')
    rvec = cv2.Rodrigues(T[:3, :3])[0]
    pixels = cv2.projectPoints(points, rvec, T[:3, 3], K, D)[0].reshape(-1, 2)
    if not np.isfinite(pixels).all() or np.max(np.abs(pixels)) > 1_000_000:
        raise ValueError('Preview projection outside drawing range')
    canvas = bgr.copy()
    cv2.drawFrameAxes(canvas, K, D, rvec, T[:3, 3], length, 2)
    height, width = canvas.shape[:2]
    for label, pixel, color in zip(('ID42', 'X', 'Y', 'Z'), pixels,
                                   ((0, 220, 255), (0, 0, 255), (0, 220, 0), (255, 0, 0))):
        x, y = np.rint(pixel).astype(int)
        offset = (-38, -10) if label == 'ID42' else (5, 14)
        origin = (int(np.clip(x+offset[0], 0, max(0, width-40))),
                  int(np.clip(y+offset[1], 15, max(15, height-3))))
        cv2.putText(canvas, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
    return canvas
