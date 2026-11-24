"""All T = parent_T_child, translation metres, rotation radians internally."""
import json
from pathlib import Path
import os
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation


def transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t).reshape(3)
    return validate(T)


def validate(T):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Invalid SE(3) matrix')
    return T


def inverse(T):
    T = validate(T)
    return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def error(a, b):
    d = inverse(a) @ validate(b)
    return float(np.linalg.norm(d[:3, 3])), float(Rotation.from_matrix(d[:3, :3]).magnitude())


def mean_pose(poses):
    return transform(Rotation.from_matrix(np.array(poses)[:, :3, :3]).mean().as_matrix(),
                     np.mean(np.array(poses)[:, :3, 3], axis=0))


def pose_matrix(p):
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('Quaternion must be normalized')
    return transform(Rotation.from_quat(q).as_matrix(), [p.position.x, p.position.y, p.position.z])


def fill_pose(p, T):
    T = validate(T)
    p.position.x, p.position.y, p.position.z = map(float, T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(T[:3, :3]).as_quat())
    return p


def tf_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return transform(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])


def fill_tf(msg, T):
    T = validate(T)
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T[:3, 3])
    q = msg.transform.rotation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(T[:3, :3]).as_quat())
    return msg


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


def read_json(path):
    return json.loads(Path(path).expanduser().read_text())
