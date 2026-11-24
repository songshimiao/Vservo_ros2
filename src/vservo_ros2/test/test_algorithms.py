# 纯算法回归：合成几何与图像提供已知真值，验证坐标方向、标定、选择策略与限幅。
# 测试不打开相机、不连接机器人；需要 NumPy、SciPy 和带 aruco 的 OpenCV。
import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from vservo_ros2.geometry import transform, inverse, error, validate
from vservo_ros2.calibration import handeye, intrinsic, validate_intrinsic
from vservo_ros2.localization import Selector
from vservo_ros2.control import step, target_flange, in_workspace, JointLimiter
from vservo_ros2.kinematics import Kinematics
from vservo_ros2.accuracy import report
from vservo_ros2.board import ArucoMarker, Board


# 用固定随机种子的生成器构造合法米制 SE(3) 样本。
def random_pose(rng, scale=0.3):
    return transform(Rotation.from_rotvec(rng.normal(0, 0.6, 3)).as_matrix(), rng.normal(0, scale, 3))


# 验证单标记只接受 ID42，原点为中心且 +Z 指入标记。
def test_single_aruco_marker_pose_uses_center_and_inward_normal():
    marker = ArucoMarker(42, 0.08)
    K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    D = np.zeros(5)
    expected = transform(Rotation.from_euler('xyz', [0.2, -0.23, 0.13]).as_matrix(), [0.02, -0.03, 0.35])
    rotation = expected[:3, :3] @ np.diag([1, -1, -1])
    pixels = cv2.projectPoints(marker.points, cv2.Rodrigues(rotation)[0], expected[:3, 3], K, D)[0]
    marker_pixels = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], dtype=np.float32)
    warp = cv2.getPerspectiveTransform(marker_pixels, pixels.reshape(4, 2))
    for mid in (41, 42):
        bitmap = cv2.aruco.generateImageMarker(marker.dictionary, mid, 200)
        image = cv2.warpPerspective(bitmap, warp, (640, 480), borderValue=255)
        if mid != 42:
            with pytest.raises(ValueError, match='exactly one ArUco ID 42'):
                marker.detect(image, K, D)
            continue
        observation = marker.detect(image, K, D)
        assert observation.target_id == 'aruco_42'
        t, a = error(expected, observation.camera_T_target)
        assert t < 0.005 and a < np.deg2rad(3)


# 分别构造 EIH/ETH 精确配对，检查能恢复已知手眼及留出误差。
@pytest.mark.parametrize('mode', ['eih', 'eth'])
def test_handeye_known_ground_truth(mode):
    rng = np.random.default_rng(7)
    X, fixed = random_pose(rng), random_pose(rng)
    gs = [random_pose(rng) for _ in range(32)]
    cs = [inverse(X) @ inverse(g) @ fixed if mode == 'eih' else inverse(X) @ g @ fixed for g in gs]
    out = handeye(gs, cs, mode)
    t, a = error(X, out['matrix'])
    assert t < 1e-6 and a < 1e-6
    assert out['reports']['heldout']['translation_m']['maximum'] < 1e-6


# 退化单轴旋转和随机错配样本必须拒绝。
def test_handeye_rejects_single_axis_and_bad_pairing():
    gs = [transform(Rotation.from_rotvec([0, 0, i*0.08]).as_matrix(), [0.1, i*0.01, 0.2]) for i in range(20)]
    with pytest.raises(ValueError, match='Degenerate'):
        handeye(gs, gs, 'eih')
    rng = np.random.default_rng(11)
    gs, cs = [random_pose(rng) for _ in range(24)], [random_pose(rng) for _ in range(24)]
    with pytest.raises(ValueError):
        handeye(gs, cs, 'eth')


# 用已知 K 投影角点并加小噪声，检查内参恢复及留出误差。
def test_intrinsic_recovers_synthetic_camera_and_heldout():
    rng = np.random.default_rng(4)
    K = np.array([[700., 0, 320], [0, 710., 240], [0, 0, 1]])
    D = np.zeros(5)
    obj = np.mgrid[0:7, 0:5].T.reshape(-1, 2).astype(np.float32)*0.025
    obj = np.c_[obj, np.zeros(len(obj), dtype=np.float32)]
    obs = []
    for i in range(36):
        r = rng.uniform(-0.5, 0.5, 3)
        t = np.array([rng.uniform(-0.15, 0.05), rng.uniform(-0.12, 0.04), rng.uniform(0.4, 0.7)])
        pix = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
        obs.append((obj, pix + rng.normal(0, 0.03, pix.shape)))
    result = intrinsic(obs, [640, 480])
    assert np.allclose(np.array(result['camera_matrix'])[:2, :2], K[:2, :2], atol=3)
    assert result['heldout_rms_px']['maximum'] < 0.2


# 样本不足或始终覆盖同一图像区域时不能通过求解。
def test_intrinsic_rejects_low_count_and_coverage():
    with pytest.raises(ValueError, match='Need'):
        intrinsic([], [640, 480])
    obj = np.zeros((10, 3), np.float32)
    with pytest.raises(ValueError, match='coverage'):
        intrinsic([(obj, np.ones((10, 2)))]*20, [640, 480])


# 历史 handoff 确认必须来自不同帧，重复 tick 不凑数，过期后 HOLD。
def test_selector_confirmations_require_distinct_frames_and_loss_holds():
    s = Selector(selection_policy='handoff')
    T = transform(t=[0.1, 0.2, 0.3])
    s.update('eth', 1.0, T, 0.8)
    assert s.choose(1.01)[0] == 'COARSE'
    s.update('eih', 1.0, T, 0.3)
    for _ in range(10):
        assert s.choose(1.01)[0] == 'COARSE'
    for ts in [1.05, 1.1]:
        s.update('eth', ts, T, 0.8)
        s.update('eih', ts, T, 0.3)
        s.choose(ts+0.001)
    assert s.choose(1.11)[0] == 'FINE'
    assert s.choose(1.5)[0] == 'HOLD'


# 启用一致性阻断时拒绝超差，未来帧不可用，旧帧不能覆盖新样本。
def test_selector_rejects_disagreement_future_and_old_samples():
    s = Selector(require_pair_agreement=True, selection_policy='handoff')
    s.update('eth', 1, np.eye(4), 1)
    s.update('eih', 1, transform(t=[0.1, 0, 0]), 0.3)
    assert s.choose(1.01)[0] == 'INCONSISTENT'
    assert s.choose(0)[0] == 'HOLD'
    s.update('eth', 0.5, transform(t=[10, 0, 0]), 1)
    assert s.samples['eth'][0] == 1


# 验证历史交接窗中 ETH 丢失后 EIH 可接管，辅助偏差可显示但不强制阻断。
def test_selector_hands_off_after_eth_loss_and_uses_eth_to_check_eih():
    s = Selector(selection_policy='handoff')
    target = transform(t=[0.2, 0.0, 0.5])
    s.update('eth', 1.0, target, 0.8)
    assert s.choose(1.01)[0] == 'COARSE'
    assert s.choose(1.31) == ('HOLD', None)
    for stamp in (1.35, 1.40):
        s.update('eih', stamp, target, 0.35)
        assert s.choose(stamp+0.01) == ('HOLD', None)
        assert s.diagnostics['reason'] == 'eih_confirming'
    for _ in range(4):
        assert s.choose(1.41) == ('HOLD', None)
    s.update('eih', 1.45, target, 0.35)
    state, sample = s.choose(1.46)
    assert state == 'FINE' and sample[0] == 1.45
    assert s.diagnostics['source'] == 'eih' and s.diagnostics['pair_status'] == 'missing'
    s.update('eth', 1.48, target, 0.8)
    assert s.choose(1.49)[0] == 'FINE'
    assert s.diagnostics['pair_status'] == 'agreed'
    s.update('eth', 1.50, transform(t=[0.3, 0.0, 0.5]), 0.8)
    assert s.choose(1.51)[0] == 'FINE'
    assert s.diagnostics['pair_status'] == 'inconsistent' and not s.diagnostics['pair_enforced']


# 验证历史策略需近期粗定位，并在精定位丢失后按恢复延迟回到 ETH。
def test_selector_requires_recent_coarse_acquisition_and_reacquires_after_fine_loss():
    s = Selector(confirmations=1, selection_policy='handoff')
    for stamp in (1.0, 2.01):
        s.update('eih', stamp, np.eye(4), 0.35)
        assert s.choose(stamp+0.01) == ('HOLD', None)
        if stamp == 1.0:
            s.update('eth', 1.0, np.eye(4), 0.8)
            assert s.choose(1.02)[0] == 'COARSE'
    s.update('eth', 2.03, np.eye(4), 0.8)
    assert s.choose(2.04)[0] == 'COARSE'
    assert s.choose(2.05)[0] == 'FINE'
    s.update('eth', 2.34, np.eye(4), 0.8)
    assert s.choose(2.35) == ('HOLD', None)
    assert s.diagnostics['reason'] == 'eih_lost'
    assert s.choose(2.40)[0] == 'HOLD'
    s.update('eth', 2.90, np.eye(4), 0.8)
    assert s.choose(2.91)[0] == 'COARSE'


# 短时丢失允许 EIH 自恢复，长时失效清除历史交接资格。
def test_eih_reacquires_without_eth_then_releases_lock_after_long_loss():
    s = Selector(confirmations=2, selection_policy='handoff')
    s.update('eth', 1.0, np.eye(4), 0.8)
    assert s.choose(1.01)[0] == 'COARSE'
    for ts in (1.1, 1.2):
        s.update('eih', ts, np.eye(4), 0.2)
        s.choose(ts+0.01)
    assert s.fine
    assert s.choose(1.51)[0] == 'HOLD'
    for ts in (1.6, 1.7):
        s.update('eih', ts, np.eye(4), 0.2)
        s.choose(ts+0.01)
    assert s.fine
    assert s.choose(4.0)[0] == 'HOLD'
    s.update('eih', 4.1, np.eye(4), 0.2)
    assert s.choose(4.11)[0] == 'HOLD'


# 运行中降低速度上限，实际指令速度逐拍减小且不破坏加速度界限。
def test_joint_speed_override_decelerates_without_acceleration_jump():
    limiter = JointLimiter(np.zeros(6), 0.15, 0.3)
    for _ in range(60):
        limiter.step(np.ones(6))
    assert limiter.v[0] == pytest.approx(0.15)
    limiter.max_speed = 0.03
    for _ in range(60):
        previous = limiter.v.copy()
        limiter.step(np.ones(6))
        assert np.max(np.abs(limiter.v-previous)) <= 0.00300001
    assert limiter.v[0] == pytest.approx(0.03)


# eth_primary 始终优先 ETH，偏差只诊断；ETH 丢失才用 EIH。
def test_eth_has_priority_even_when_pair_disagrees_and_eih_takes_over_on_eth_loss():
    selector = Selector(require_pair_agreement=True, selection_policy='eth_primary')
    eth = transform(t=[0.2, 0.1, 0.4])
    eih = transform(t=[0.05, 0.1, 0.4])
    selector.update('eth', 1.0, eth, 0.8)
    mode, sample = selector.choose(1.01)
    assert mode == 'COARSE' and np.allclose(sample[1], eth)
    selector.update('eih', 1.02, eih, 0.2)
    mode, sample = selector.choose(1.03)
    assert mode == 'COARSE' and np.allclose(sample[1], eth)
    assert selector.diagnostics['pair_status'] == 'inconsistent'
    assert not selector.diagnostics['pair_enforced']
    selector.update('eih', 1.32, eih, 0.2)
    mode, sample = selector.choose(1.33)
    assert mode == 'FINE' and np.allclose(sample[1], eih)
    assert selector.choose(1.7) == ('HOLD', None)
    selector.update('eih', 1.71, eih, 0.2)
    assert selector.choose(1.72)[0] == 'FINE'
    selector.update('eth', 1.73, eth, 0.8)
    assert selector.choose(1.74)[0] == 'COARSE'


# 默认 first_seen 覆盖两种先后顺序、主导保持、丢失接管及恢复不抢占。
@pytest.mark.parametrize('first,second', [('eih', 'eth'), ('eth', 'eih')])
def test_first_seen_camera_keeps_control_until_it_loses_target(first, second):
    selector = Selector(require_pair_agreement=True)
    target = {'eih': transform(t=[0, 0, 0.5]), 'eth': transform(t=[0.15, 0, 0.5])}
    selector.update(first, 1.0, target[first], 0.5)
    assert selector.choose(1.01)[1][0] == 1.0
    assert selector.diagnostics['source'] == first
    selector.update(second, 1.02, target[second], 0.5)
    selector.update(first, 1.03, target[first], 0.5)
    selector.choose(1.04)
    assert selector.diagnostics['source'] == first
    assert selector.diagnostics['pair_status'] == 'inconsistent'
    assert not selector.diagnostics['pair_enforced']
    selector.update(second, 1.33, target[second], 0.5)
    selector.choose(1.34)
    assert selector.diagnostics['source'] == second
    selector.update(first, 1.35, target[first], 0.5)
    selector.choose(1.36)
    assert selector.diagnostics['source'] == second
    selector.update(first, 1.67, target[first], 0.5)
    selector.choose(1.68)
    assert selector.diagnostics['source'] == first
    assert selector.choose(2.1) == ('HOLD', None)
    selector.update(second, 2.2, target[second], 0.5)
    selector.choose(2.21)
    assert selector.diagnostics['source'] == second


# 首次主导依据图像时间而非回调到达顺序；未来观测不参与选择。
def test_first_acquisition_uses_image_time_and_ignores_future_observations():
    selector = Selector()
    selector.update('eth', 1.02, np.eye(4), 0.5)
    selector.update('eih', 1.0, np.eye(4), 0.5)
    selector.choose(1.03)
    assert selector.diagnostics['source'] == 'eih'
    assert selector.choose(0) == ('HOLD', None)
    selector.choose(1.03)
    assert selector.diagnostics['source'] == 'eih'


# 验证手眼连乘方向、悬停距离、预览单步限幅与坏矩阵拒绝。
def test_transform_direction_hover_and_servo_limits():
    X = transform(t=[0.03, 0.0, 0.04])
    target = transform(Rotation.from_euler('xyz', [0.2, 0.3, 0.1]).as_matrix(), [0.2, 0.1, 0.5])
    flange = target_flange(target, X, 0.18)
    assert np.allclose(inverse(flange @ X) @ target, transform(t=[0, 0, 0.18]))
    nxt = step(np.eye(4), target)
    t, a = error(np.eye(4), nxt)
    assert t <= 0.001000001 and a <= 0.010000001
    assert not in_workspace(target, [-0.1]*3, [0.1]*3)
    with pytest.raises(ValueError):
        validate(np.zeros((4, 4)))


# 检查 FR5 零位 FK 和邻近位姿的 FK→IK→FK 回环误差。
def test_fk_ik_local_roundtrip_and_nominal_zero():
    m = Kinematics()
    assert np.allclose(m.fk(np.zeros(6))[:3, 3], [-0.82, -0.202, 0.05])
    q = np.deg2rad([20, -90, 75, -60, -80, 15])
    recovered = m.ik(m.fk(q+0.001), q)
    t, a = error(m.fk(recovered), m.fk(q+0.001))
    assert t < 1e-5 and a < 1e-5


# 固定 10 mm 偏置可以有零散布，必须区分重复性与独立参考误差。
def test_accuracy_bias_separate_from_repeatability():
    poses = [transform(t=[0.01, 0, 0])]*10
    assert report(poses)['translation_mm']['rms'] == pytest.approx(0, abs=1e-12)
    absolute = report(poses, [np.eye(4)]*10)
    assert absolute['translation_mm']['rms'] == pytest.approx(10)
    assert 'independent' in absolute['kind']


# 错误角色、序列号或分辨率都不能复用内参。
def test_intrinsic_metadata_isolation():
    data = dict(camera={'serial': 'A', 'role': 'eye_in_hand'}, image_size=[640, 480],
                camera_matrix=[[300, 0, 320], [0, 300, 240], [0, 0, 1]], dist_coeffs=[0]*5)
    validate_intrinsic(data, 'eih', 'A', [640, 480])
    for role, serial, size in [('eth', 'A', [640, 480]), ('eih', 'B', [640, 480]), ('eih', 'A', [1280, 720])]:
        with pytest.raises(ValueError):
            validate_intrinsic(data, role, serial, size)


# 用合成 ChArUco 图检查角点数与正深度位姿。
def test_rendered_board_detection():
    b = Board()
    raw = b.board.generateImage((700, 500))
    canvas = np.full((700, 900), 255, np.uint8)
    canvas[100:600, 100:800] = raw
    obj, pix, ids = b.corners(canvas)
    assert len(pix) == 24
    obs = b.detect(canvas, np.array([[1000., 0, 450], [0, 1000., 350], [0, 0, 1]]), np.zeros(5))
    assert obs.camera_T_target[2, 3] > 0


# 目标多次反向时，按实际 q 差分检查指令速度/加速度仍受限。
def test_joint_commands_bound_velocity_and_acceleration_on_reversals():
    limiter = JointLimiter(np.zeros(6), 0.15, 0.3)
    previous_q, previous_v = limiter.q.copy(), limiter.v.copy()
    for i in range(300):
        goal = np.full(6, 0.03 if i < 100 else -0.02 if i < 200 else 0)
        q = limiter.step(goal)
        v = (q-previous_q)/0.01
        assert np.max(np.abs(v)) <= 0.15000001
        assert np.max(np.abs(v-previous_v))/0.01 <= 0.30000001
        previous_q, previous_v = q, v
