# Web 业务回归：使用临时文件与 ROS 客户端替身，不启动实体相机或机器人。
import json
from pathlib import Path
import shlex
from shutil import copyfile
import threading
from types import SimpleNamespace

import numpy as np
import pytest
from rcl_interfaces.msg import Log

from vservo_ros2.geometry import write_json
from vservo_ros2.web.ros_link import RosLink, camera_status
from vservo_ros2.web.server import Console, authorize_action
from vservo_ros2.web.workspace import Workspace, assess


# 构造最小有效 EIH 内参字典，可切换序列号或验证标志。
def intrinsic(serial='EIH', validated=True):
    return dict(validated=validated, camera=dict(serial=serial, role='eye_in_hand'),
                image_size=[640, 480], camera_matrix=[[360, 0, 320], [0, 360, 240], [0, 0, 1]],
                dist_coeffs=[0]*5)


# 构造在线相机快照，测试文件与实际设备的身份/尺寸核对。
def live(serial='EIH', size=None):
    return dict(online=True, size=size or [640, 480],
                metadata=dict(camera=dict(serial=serial, role='eye_in_hand')))


# 区分文件缺失、未验证、身份不符、尺寸不符和相机离线待核对。
def test_file_missing_unvalidated_and_incompatible(tmp_path):
    path = tmp_path/'camera.json'
    assert not assess(path, 'eih', 'intrinsic', live())['exists']
    write_json(path, intrinsic(validated=False))
    assert not assess(path, 'eih', 'intrinsic', live())['valid']
    write_json(path, intrinsic())
    assert assess(path, 'eih', 'intrinsic', live())['compatible'] is True
    assert assess(path, 'eih', 'intrinsic', live('wrong'))['compatible'] is False
    assert assess(path, 'eih', 'intrinsic', live(size=[1280, 720]))['compatible'] is False
    assert assess(path, 'eih', 'intrinsic', dict(online=False))['compatible'] is None


# 错误手眼父坐标系或非法矩阵不能通过文件评估。
def test_handeye_frame_and_matrix_checks(tmp_path):
    path = tmp_path/'handeye.json'
    data = dict(validated=True, mode='eih', camera=dict(serial='EIH', role='eye_in_hand'),
                units='m', parent_frame='flange', child_frame='eih_optical_frame', matrix=np.eye(4).tolist())
    write_json(path, data)
    assert assess(path, 'eih', 'handeye', live())['compatible'] is True
    write_json(path, dict(data, parent_frame='base'))
    assert not assess(path, 'eih', 'handeye', live())['valid']
    write_json(path, dict(data, matrix=[[1, 0]]))
    assert not assess(path, 'eih', 'handeye', live())['valid']


# 接收年龄与消息年龄都必须新鲜，未来 ROS 时间也应拒绝。
def test_frame_receipt_and_ros_stamp_must_both_be_fresh():
    assert camera_status([], None, None, 4, 101)['online'] is False
    frames = [(1, 99.9), (1.1, 100)]
    assert camera_status(frames, [640, 480], {}, 1.2, 100.1)['hz'] == 10
    assert not camera_status(frames, None, None, 3, 100.1)['online']
    assert not camera_status(frames, None, None, 1.2, 103)['online']
    assert not camera_status(frames, None, None, 1.2, 99)['online']


# 普通相机日志不入告警缓存，WARN 正确保存且回调不崩溃。
def test_camera_startup_log_keeps_ros_subscription_alive():
    link = SimpleNamespace(lock=threading.Lock(), values={})
    RosLink.camera_log(link, Log(name='eih.camera', level=20, msg='camera ready'))
    assert link.values == {}
    RosLink.camera_log(link, Log(name='eih.camera', level=30, msg='camera warning'))
    assert link.values['eih_camera_log'][1] == 'camera warning'


# 含空格/单引号路径仍正确引用，备用命令保持只读并保护已有结果。
def test_commands_quote_workspace_and_never_enable_motion(tmp_path):
    ws = Workspace(tmp_path/"site O'Brien")
    one = ws.new_session('eih', 'intrinsic', 'train')
    two = ws.new_session('eih', 'intrinsic', 'train')
    assert one['name'] != two['name']
    command = next(iter(one['commands'].values()))
    args = shlex.split(command)
    assert f"session_dir:={ws.sessions/one['name']}" in args
    assert 'protect_output:=true' in args
    assert str(ws.output(one['plan'])) == next(a.split(':=', 1)[1] for a in args if a.startswith('output:='))
    selections = {role: {kind: ws.select(role, kind) for kind in ('intrinsic', 'handeye')}
                  for role in ('eih', 'eth')}
    commands = ws.commands(selections, {r: dict(online=False) for r in selections})
    assert 'execute:=false' in commands['启动完整只读伺服（先停止独立相机及桥接）']
    assert all('execute:=true' not in cmd and '/robot_bridge/arm' not in cmd for cmd in commands.values())


# 采样/求解需匹配阶段、服务、检查项和只读状态，已有输出不可覆盖。
def test_action_rejects_wrong_phase_unavailable_and_existing_output(tmp_path):
    ws = Workspace(tmp_path)
    session = ws.new_session('eih', 'handeye', 'train')
    name = session['name']
    preview = dict(role='eih', mode='handeye', session_dir=str(ws.session_dir(name)),
                   output=str(ws.output(session['plan'])), protect_output=True)
    status = dict(values=dict(eih_preview=preview, robot='READ_ONLY'),
                  calibration_nodes=dict(eih=1), services=dict(eih=dict(capture=True, solve=True)))
    with pytest.raises(ValueError, match='检查清单'):
        authorize_action(ws, status, name, 'capture', False)
    assert authorize_action(ws, status, name, 'capture', True) == session['plan']
    status['values']['robot'] = 'ARMED'
    with pytest.raises(ValueError, match='READ_ONLY'):
        authorize_action(ws, status, name, 'capture', True)
    preview['mode'] = 'intrinsic'
    with pytest.raises(ValueError, match='不匹配'):
        authorize_action(ws, status, name, 'solve', True)
    preview['mode'] = 'handeye'
    status['services']['eih']['solve'] = False
    with pytest.raises(ValueError, match='不可用'):
        authorize_action(ws, status, name, 'solve', True)
    status['services']['eih']['solve'] = True
    write_json(ws.output(session['plan']), {'existing': 1})
    with pytest.raises(ValueError, match='已存在'):
        authorize_action(ws, status, name, 'solve', True)
    with pytest.raises(FileExistsError):
        write_json(ws.output(session['plan']), {'existing': 2}, replace=False)
    assert json.loads(ws.output(session['plan']).read_text()) == {'existing': 1}


# ROS Trigger 失败原因原样返回网页，不把失败误写成成功。
def test_trigger_failure_response_preserves_reason():
    response = SimpleNamespace(success=False, message='Need at least 8 ChArUco corners')
    future = SimpleNamespace(done=lambda: True, result=lambda: response,
                             add_done_callback=lambda callback: callback(None))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)
    link = SimpleNamespace(lock=threading.Lock(), inflight={}, trigger_clients={('eih', 'capture'): client})
    assert RosLink.trigger(link, 'eih', 'capture') == dict(success=False, message=response.message)
    with pytest.raises(KeyError):
        RosLink.trigger(link, 'robot_bridge', 'arm')


# 验证会话不提供求解动作；必须有成功退出码，且标定内容变化后旧报告失效。
def test_independent_session_has_no_solve_and_report_is_bound(tmp_path):
    ws = Workspace(tmp_path)
    source = ws.select('eih', 'intrinsic')
    write_json(source, intrinsic())
    check = ws.new_session('eih', 'intrinsic', 'check')
    name = check['name']
    assert 'test_intrinsics.sh' in '\n'.join(check['commands'].values())
    assert '/calibration/solve' not in '\n'.join(check['commands'].values())
    with pytest.raises(ValueError, match='独立验证'):
        authorize_action(ws, dict(values={}), name, 'solve', True)
    write_json(ws.session_dir(name)/'samples.json', {'samples': [{}]*5})
    report = ws.reports/f'{name}.json'
    write_json(report, {'passed': True, 'independent_session': True})
    assert ws.verification('eih', 'intrinsic', source) is None
    report.with_suffix('.exit').write_text('0\n')
    assert ws.verification('eih', 'intrinsic', source) == report.name
    write_json(source, dict(intrinsic(), revised=True))
    assert ws.verification('eih', 'intrinsic', source) is None


# 按角色选择板规格，并固定到会话；验证阶段沿用来源标定的板尺寸。
def test_web_sessions_bind_different_board_sizes_by_role_and_phase(tmp_path):
    ws = Workspace(tmp_path)
    sources = Path(__file__).resolve().parents[1]/'config'
    for profile in ('a4', 'compact'):
        config = ws.board_config(profile)
        config.parent.mkdir(parents=True, exist_ok=True)
        copyfile(sources/config.name, config)
    console = Console(ws, SimpleNamespace())
    eih = console.mutate('/api/session', dict(role='eih', kind='handeye', purpose='train'))
    eth = console.mutate('/api/session', dict(role='eth', kind='handeye', purpose='train'))
    assert (eih['board']['square_m'], eth['board']['square_m']) == (0.038, 0.025)
    path = ws.select('eth', 'handeye')
    write_json(path, dict(validated=True, mode='eth', camera=dict(serial='ETH', role='eye_to_hand'),
                          units='m', parent_frame='base', child_frame='eth_optical_frame',
                          matrix=np.eye(4).tolist(), board=eth['board']))
    check = console.mutate('/api/session', dict(role='eth', kind='handeye', purpose='check', board_profile='a4'))
    assert check['board'] == eth['board']
    assert f'board_config:={ws.session_dir(check["name"]) / "board.yaml"}' in ws.calibration_launch(check['plan'])
    plan = check['plan']
    preview = dict(role='eth', mode='handeye', session_dir=str(ws.session_dir(plan['name'])),
                   output=str(ws.output(plan)), board=eih['board'])
    snapshot = dict(values=dict(eth_preview=preview, robot='READ_ONLY'),
                    calibration_nodes=dict(eth=1), services=dict(eth=dict(capture=True)))
    with pytest.raises(ValueError, match='标定板规格'):
        authorize_action(ws, snapshot, plan['name'], 'capture', True)
    preview['board'] = eth['board']
    assert authorize_action(ws, snapshot, plan['name'], 'capture', True) == plan


# 未知运动路由和 ../ 路径越界不能访问。
def test_unknown_motion_route_and_file_escape_are_rejected(tmp_path):
    ws = Workspace(tmp_path)
    console = Console(ws, SimpleNamespace())
    with pytest.raises(FileNotFoundError):
        console.mutate('/api/arm', {'data': True})
    with pytest.raises(ValueError):
        console.report('../secrets.json')
    with pytest.raises(ValueError):
        ws.session_dir('../elsewhere')


# 允许运动时可先开启持续请求等目标，之后人工停止；只读模式拒绝。
def test_web_follow_starts_without_target_and_can_be_stopped(tmp_path):
    calls = []
    snapshot = dict(values=dict(robot='DISARMED'), tf=dict(ok=True))

    # 记录 enable/disable 请求的服务替身，不发送硬件命令。
    def arm(value):
        calls.append(value)
        return dict(success=True, message='test bridge')

    link = SimpleNamespace(snapshot=lambda: snapshot,
                           get_node_names_and_namespaces=lambda: [('robot_bridge', '/')], arm=arm)
    console = Console(Workspace(tmp_path), link)
    console.processes.running = lambda name: name == 'servo'
    assert not console.mutate('/api/start-motion', {'confirmed': True})['success']
    console.allow_motion = True
    assert calls == []
    assert console.mutate('/api/start-motion', {})['success']
    assert console.mutate('/api/stop-motion', {})['success']
    assert calls == [True, False]


# 只有 ROS 调速成功才保存新倍率，非法值或服务失败保留上次值。
def test_web_speed_preserves_last_value_when_ros_rejects_request(tmp_path):
    link = SimpleNamespace(snapshot=lambda: {}, get_node_names_and_namespaces=lambda: [('robot_bridge', '/')],
                           set_speed=lambda value: dict(success=True, message='applied'))
    console = Console(Workspace(tmp_path), link)
    console.processes.running = lambda name: name == 'servo'
    assert console.mutate('/api/speed', dict(percent=35))['success']
    assert console.speed_percent == 35
    link.set_speed = lambda value: dict(success=False, message='service unavailable')
    assert not console.mutate('/api/speed', dict(percent=75))['success']
    assert console.speed_percent == 35
    for value in (True, '50', 0, 101, float('nan')):
        assert not console.mutate('/api/speed', dict(percent=value))['success']
    assert console.speed_percent == 35


# 允许运动时启动监控传 auto_start/execute/所选速度，只读模式保留 execute=false。
def test_monitor_start_requests_auto_follow_with_selected_speed(tmp_path):
    ws = Workspace(tmp_path)
    for role, mount in [('eih', 'eye_in_hand'), ('eth', 'eye_to_hand')]:
        data = dict(intrinsic(role.upper()), camera=dict(serial=role.upper(), role=mount))
        write_json(ws.select(role, 'intrinsic'), data)
        write_json(ws.select(role, 'handeye'), dict(
            validated=True, mode=role, units='m', matrix=np.eye(4).tolist(),
            parent_frame='flange' if role == 'eih' else 'base', child_frame=f'{role}_optical_frame',
            camera=dict(serial=role.upper(), role=mount)))
    started = []
    console = Console(ws, SimpleNamespace(), allow_motion=True)
    console.speed_percent = 50
    console.processes = SimpleNamespace(stop=lambda name: None,
                                        start=lambda name, argv, conflicts: started.append(argv))
    snapshot = dict(values={}, cameras={r: dict(online=False) for r in ('eih', 'eth')})
    console.start_servo({}, snapshot)
    assert 'auto_start:=true' in started[-1]
    assert 'execute:=true' in started[-1]
    assert 'speed_scale:=0.5' in started[-1]
    assert 'linear_speed_m_s:=0.1' in started[-1]
    assert 'max_joint_accel_rad_s2:=0.5' in started[-1]
    console.allow_motion = False
    console.start_servo({}, snapshot)
    assert 'execute:=false' in started[-1]
    assert 'auto_start:=true' not in started[-1]


# 一组参数一起提交；任一非法或 ROS 拒绝时整组不更新 Web 已选配置。
def test_web_motion_profile_updates_atomically_and_keeps_draft_on_failure(tmp_path):
    config = tmp_path/'src/vservo_ros2/config/control.yaml'
    config.parent.mkdir(parents=True)
    config.write_text('/robot_bridge:\n  ros__parameters:\n    max_joint_accel_rad_s2: 0.8\n')
    applied = []
    link = SimpleNamespace(snapshot=lambda: {}, get_node_names_and_namespaces=lambda: [('robot_bridge', '/')],
                           set_speed=lambda percent, profile: applied.append((percent, profile)) or
                           dict(success=True, message='applied'))
    console = Console(Workspace(tmp_path), link)
    assert console.motion_profile['joint_accel_rad_s2'] == 0.8
    console.processes.running = lambda name: name == 'servo'
    body = dict(percent=80, linear_mm_s=150, angular_deg_s=45, joint_accel_rad_s2=1.0)
    result = console.mutate('/api/speed', body)
    assert result['success'] and applied[-1] == (80, result['motion_profile'])
    for patch in (dict(linear_mm_s=201), dict(angular_deg_s=61), dict(joint_accel_rad_s2=0),
                  dict(linear_mm_s=True), dict(joint_accel_rad_s2=float('nan'))):
        assert not console.mutate('/api/speed', dict(body, **patch))['success']
    assert len(applied) == 1
    link.set_speed = lambda *args: dict(success=False, message='rejected')
    assert not console.mutate('/api/speed', dict(body, percent=50, linear_mm_s=50))['success']
    assert console.speed_percent == 80 and console.motion_profile == result['motion_profile']
