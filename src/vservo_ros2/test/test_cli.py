# 离线命令行回归：临时目录中生成数据，检查输出报告、独立会话要求与失败退出码。
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from vservo_ros2.cli import main
from vservo_ros2.geometry import write_json, read_json, transform, inverse


# 超出 5 mm 阈值仍需写出失败报告，并以退出码 2 表示未通过。
def test_accuracy_cli_writes_report_and_nonzero_on_failed_gate(tmp_path):
    input_path, output = tmp_path/'input.json', tmp_path/'report.json'
    write_json(input_path, dict(poses=[transform(t=[0.01, 0, 0]).tolist()]*5,
                               references=[np.eye(4).tolist()]*5))
    with pytest.raises(SystemExit) as exc:
        main(['accuracy', '--input', str(input_path), '--out', str(output), '--max-mm', '5'])
    assert exc.value.code == 2
    assert read_json(output)['passed'] is False
    assert read_json(output)['translation_mm']['rms'] == pytest.approx(10)


# 训练求解后用全新会话验证；再传训练清单必须返回数据错误退出码 1。
def test_handeye_cli_checks_new_session_against_fixed_training_reference(tmp_path):
    rng = np.random.default_rng(33)
    X, fixed = transform(t=[0.02, 0.01, 0.05]), transform(t=[0.3, 0.2, 0.1])
    gs = [transform(Rotation.from_rotvec(rng.normal(0, 0.6, 3)).as_matrix(), rng.normal(0, 0.3, 3)) for _ in range(24)]
    cs = [inverse(X) @ inverse(g) @ fixed for g in gs]
    common = dict(role='eih', mode='handeye', camera={'serial': 'TEST', 'role': 'eye_in_hand'},
                  board={'square_m': 0.015}, image_size=[640, 480])
    samples = [dict(base_T_flange=g.tolist(), camera_T_board=c.tolist()) for g, c in zip(gs, cs)]
    session = tmp_path/'train/samples.json'
    calibration = tmp_path/'calibration.json'
    write_json(session, dict(**common, samples=samples[:16]))
    main(['solve', '--session', str(session), '--out', str(calibration)])
    validation, report = tmp_path/'check/samples.json', tmp_path/'report.json'
    write_json(validation, dict(**common, samples=samples[16:]))
    main(['check-handeye', '--session', str(validation), '--calibration', str(calibration), '--out', str(report)])
    assert read_json(report)['passed'] and read_json(report)['independent_session']
    with pytest.raises(SystemExit) as exc:
        main(['check-handeye', '--session', str(session), '--calibration', str(calibration), '--out', str(report)])
    assert exc.value.code == 1
