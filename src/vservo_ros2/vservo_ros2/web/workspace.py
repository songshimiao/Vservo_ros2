# Web 文件层：限定路径、查找标定候选、固定会话板规格、绑定独立验证报告、生成备用命令。
# 训练结果与独立验收分开保存；验证通过标记绑定标定文件的 SHA-256，文件改变后不能沿用旧结论。
"""Filesystem views and shell-safe commands for the operator console."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shlex
from uuid import uuid4

import yaml

from ..calibration import validate_intrinsic
from ..geometry import validate, write_json

ROLES = ('eih', 'eth')
KINDS = ('intrinsic', 'handeye')
MOUNT = {'eih': 'eye_in_hand', 'eth': 'eye_to_hand'}
FILES = {'intrinsic': 'intrinsics', 'handeye': 'handeye'}
BOARD_PROFILES = {'legacy': 'board.yaml', 'compact': 'board_compact.yaml', 'a4': 'board_a4.yaml'}


# 最多读取 8 MiB 的 JSON 字典，并拒绝 NaN/Infinity 等非标准数值。
def read_object(path):
    if path.stat().st_size > 8 * 1024 * 1024:
        raise ValueError('JSON 超过 8 MiB')
    # 作为 JSON 回调，遇到非法数值立即抛错。
    def reject_constant(token):
        raise ValueError(f'JSON 包含无效数值 {token}')
    value = json.loads(path.read_text(), parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError('预期 JSON 对象')
    return value


# 先判断文件内容有效性，再核对在线相机身份/尺寸；compatible=None 表示尚无在线证据。
def assess(path, role, kind, camera):
    """Separate validated file contents from compatibility with a live camera."""
    result = dict(exists=path.is_file(), valid=False, compatible=None)
    try:
        data = read_object(path)
        if data.get('validated') is not True:
            raise ValueError('validated 未通过')
        identity = data['camera']
        serial = identity['serial']
        if not isinstance(serial, str) or not serial or identity.get('role') != MOUNT[role]:
            raise ValueError('相机身份或安装角色错误')
        if kind == 'intrinsic':
            size = data['image_size']
            if len(size) != 2 or any(type(n) is not int or n <= 0 for n in size):
                raise ValueError('图像分辨率无效')
            validate_intrinsic(data, role, serial, size)
        else:
            frames = ('flange' if role == 'eih' else 'base', f'{role}_optical_frame')
            if (data.get('mode') != role or data.get('units') != 'm'
                    or (data.get('parent_frame'), data.get('child_frame')) != frames):
                raise ValueError('手眼模式或坐标系错误')
            validate(data['matrix'])
        result.update(valid=True, reason='已通过文件校验，等待在线相机核对')
        if camera and camera.get('online') and camera.get('metadata'):
            live = camera['metadata'].get('camera', {})
            match = live.get('serial') == serial and live.get('role') == MOUNT[role]
            if kind == 'intrinsic':
                match = match and camera.get('size') == size
            result.update(compatible=match, reason='匹配在线相机' if match else '相机身份或分辨率不匹配，需重标')
        keys = ('train_rms_px', 'heldout_rms_px', 'reports', 'camera_matrix', 'dist_coeffs',
                'matrix', 'parent_frame', 'child_frame', 'sample_count', 'source', 'image_size')
        result.update(serial=serial, modified=path.stat().st_mtime, board=data.get('board'),
                      metrics={k: data[k] for k in keys if k in data})
    except FileNotFoundError:
        result['reason'] = '文件缺失，请先标定'
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
        result['reason'] = str(exc)
    return result


# 把 ROS Trigger/SetBool 的 success/message 转成 HTTP 可序列化字典。
def service_result(response):
    return dict(success=bool(response.success), message=response.message)


# 统一约束 data、sessions、reports 路径和会话文件格式。
class Workspace:
    # 解析工程根目录并派生现场数据目录，不在构造时生成采样数据。
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.data = self.root / 'data'
        self.sessions = self.data / 'sessions'
        self.reports = self.data / 'reports'

    # 解析真实路径后限制在给定根目录内，拒绝绝对路径、根本身和经符号链接越界。
    def bounded(self, root, relative):
        relative = Path(relative)
        target = (root / relative).resolve()
        if relative.is_absolute() or target == root or not target.is_relative_to(root.resolve()):
            raise ValueError('路径超出工作区')
        return target

    # 会话名只允许单个非隐藏目录名，再做路径边界校验。
    def session_dir(self, session):
        if not isinstance(session, str) or Path(session).name != session or session.startswith('.'):
            raise ValueError('会话名称无效')
        return self.bounded(self.sessions, session)

    # 先给标准标定路径，再列出近期同角色/同类型训练会话中的结果。
    def candidate_paths(self, role, kind):
        name = f'{role}_{FILES[kind]}.json'
        yield self.data / 'calibration' / name
        if self.sessions.is_dir():
            for directory in sorted(self.sessions.glob(f'{role}_{kind}_train_*'), reverse=True)[:60]:
                candidate = self.session_dir(directory.name) / name
                if candidate.is_file() and candidate.resolve().is_relative_to(self.sessions.resolve()):
                    yield candidate

    # 无显式选择则用首个候选；显式路径必须确实属于该角色与标定类型。
    def select(self, role, kind, relative=''):
        paths = list(self.candidate_paths(role, kind))
        if not relative:
            return paths[0]
        path = self.bounded(self.data, relative)
        if path not in paths:
            raise ValueError('标定文件不属于所选相机和类型')
        return path

    # 把预定义板规格名称映射到工程 config 中的固定文件。
    def board_config(self, profile):
        return self.root/'src/vservo_ros2/config'/BOARD_PROFILES[profile]

    # 创建唯一会话及固定板规格副本；独立验证同时锁定来源标定路径和内容哈希。
    def new_session(self, role, kind, purpose, selected='', board_profile=None):
        if role not in ROLES or kind not in KINDS or purpose not in ('train', 'check'):
            raise ValueError('相机、标定类型或会话用途无效')
        source = self.select(role, kind, selected)
        name = f'{role}_{kind}_{purpose}_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}'
        directory = self.session_dir(name)
        plan = dict(name=name, role=role, kind=kind, purpose=purpose)
        spec = None
        if purpose == 'check':
            if not assess(source, role, kind, None)['valid']:
                raise ValueError('请先选择有效的标定结果')
            plan.update(calibration=str(source.relative_to(self.data)),
                        calibration_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            spec = read_object(source).get('board')
            if spec:
                plan['board_profile'] = 'calibration'
        elif board_profile is not None:
            if board_profile not in BOARD_PROFILES:
                raise ValueError('未知标定板规格')
            spec = yaml.safe_load(self.board_config(board_profile).read_text())['/**']['ros__parameters']
            plan['board_profile'] = board_profile
        directory.mkdir(parents=True, exist_ok=False)
        if spec:
            # Keep the dimensions pinned even if the global board profile is later edited.
            plan['board'] = spec
            write_json(directory/'board.yaml', {'/**': {'ros__parameters': spec}}, replace=False)
        write_json(directory/'web_session.json', plan, replace=False)
        return self.session(name)

    # 读取会话计划并检查名字、角色、类型、用途，供所有后续动作共同使用。
    def plan(self, name):
        directory = self.session_dir(name)
        plan = read_object(self.bounded(directory, 'web_session.json'))
        if (plan.get('name') != name or plan.get('role') not in ROLES or
                plan.get('kind') not in KINDS or plan.get('purpose') not in ('train', 'check')):
            raise ValueError('会话计划无效')
        return plan

    # 根据角色/类型生成本会话的结果文件名；原标准标定文件不会在此直接覆盖。
    def output(self, plan):
        return self.session_dir(plan['name']) / f"{plan['role']}_{FILES[plan['kind']]}.json"

    # 只有独立样本≥5、报告比样本新、真实退出码为 0、passed=True 且哈希匹配才认可。
    def verification(self, role, kind, selected):
        """Count only passed Web verification sessions bound to these exact calibration bytes."""
        if not selected.is_file() or not self.sessions.is_dir():
            return None
        digest = hashlib.sha256(selected.read_bytes()).hexdigest()
        for directory in sorted(self.sessions.glob(f'{role}_{kind}_check_*'), reverse=True)[:60]:
            try:
                plan = self.plan(directory.name)
                if (plan['calibration'] != str(selected.relative_to(self.data))
                        or plan['calibration_sha256'] != digest):
                    continue
                report = self.bounded(self.reports, directory.name+'.json')
                exitfile = self.bounded(self.reports, directory.name+'.exit')
                manifest = read_object(self.bounded(directory, 'samples.json'))
                metrics = read_object(report)
                if (len(manifest['samples']) >= 5 and report.stat().st_mtime >= (directory/'samples.json').stat().st_mtime
                        and exitfile.stat().st_mtime >= report.stat().st_mtime
                        and exitfile.read_text().strip() == '0' and metrics.get('passed') is True
                        and metrics.get('independent_session') is True):
                    return report.name
            except (OSError, ValueError, KeyError, TypeError):
                continue
        return None

    # 组合计划、样本清单、重采标记、板规格与备用命令，供会话页面显示。
    def session(self, name):
        directory = self.session_dir(name)
        manifest, plan = {}, None
        if (directory/'web_session.json').exists():
            plan = self.plan(name)
        if (directory/'samples.json').exists():
            manifest = read_object(self.bounded(directory, 'samples.json'))
        markers = read_object(self.bounded(directory, 'web_resample.json')) if (directory/'web_resample.json').exists() else {}
        samples = [dict(filename=s['filename'], corners=len(s.get('corner_ids', [])),
                        reprojection_px=s.get('reprojection_px'), resample=markers.get(s['filename']))
                   for s in manifest.get('samples', [])]
        board = manifest.get('board') or (plan or {}).get('board')
        if board is None and plan:
            profile = plan.get('board_profile', 'legacy')
            if profile == 'calibration':
                board = read_object(self.bounded(directory, 'board.yaml'))['/**']['ros__parameters']
            elif profile in BOARD_PROFILES and self.board_config(profile).is_file():
                board = yaml.safe_load(self.board_config(profile).read_text())['/**']['ros__parameters']
        return dict(name=name, plan=plan, role=manifest.get('role'), mode=manifest.get('mode'), board=board,
                    samples=samples, commands=self.session_commands(plan) if plan else {})

    # 构造采集节点 argv，开启结果防覆盖；续采显式传 resume，并使用会话固定板规格。
    def calibration_launch(self, plan, resume=False):
        role, kind = plan['role'], plan['kind']
        directory = self.session_dir(plan['name'])
        args = ['ros2', 'launch', 'vservo_ros2', 'calibration.launch.py',
                f'role:={role}', f'mode:={kind}', f'session_dir:={directory}',
                f'output:={self.output(plan)}', 'protect_output:=true']
        if resume:
            args.append('resume:=true')
        if plan.get('board') or plan.get('board_profile') == 'calibration':
            args.append(f'board_config:={self.bounded(directory, "board.yaml")}')
        elif plan.get('board_profile') == 'a4':
            args.append(f'board_config:={self.board_config("a4")}')
        return args

    # 验证来源文件仍与会话哈希一致，再返回独立验证脚本 argv 和报告路径。
    def verification_run(self, plan):
        if plan['purpose'] != 'check':
            raise ValueError('请使用独立验证会话')
        role, kind = plan['role'], plan['kind']
        source = self.select(role, kind, plan['calibration'])
        if hashlib.sha256(source.read_bytes()).hexdigest() != plan['calibration_sha256']:
            raise ValueError('所选标定文件已变更，请创建独立验证会话')
        script = 'test_intrinsics.sh' if kind == 'intrinsic' else 'test_handeye.sh'
        report = self.bounded(self.reports, plan['name']+'.json')
        return ['bash', str(self.root/'scripts'/script), str(self.session_dir(plan['name'])/'samples.json'),
                str(source), str(report)], report

    # 生成可复制命令；验证命令额外记录真实退出码，区别无报告错误和验收不通过。
    def session_commands(self, plan):
        role = plan['role']
        commands = {'启动采集节点（终端备用）': shlex.join(self.calibration_launch(plan, resume=True)),
                    '采集（命令行备用）': f"ros2 service call /{role}/calibration/capture std_srvs/srv/Trigger '{{}}'"}
        if plan['purpose'] == 'train':
            commands['求解（命令行备用）'] = f"ros2 service call /{role}/calibration/solve std_srvs/srv/Trigger '{{}}'"
        else:
            # Existing scripts emit 0/2 reports and may exit 1 before a report exists.
            # Capture the real exit code in a separate file for the report viewer.
            try:
                argv, report = self.verification_run(plan)
            except (OSError, ValueError):
                return commands
            commands['独立验证（终端备用）'] = (
                f"mkdir -p {shlex.quote(str(self.reports))} && ({shlex.join(argv)}; "
                f"rc=$?; printf '%s\\n' \"$rc\" > {shlex.quote(str(report.with_suffix('.exit')))}; exit \"$rc\")")
        return commands

    # 保存/取消人工重采原因，仅作标记；不从原 samples.json 中自动删样本。
    def mark(self, name, filename, reason):
        directory = self.session_dir(name)
        if filename not in {s['filename'] for s in self.session(name)['samples']}:
            raise ValueError('会话样本不存在')
        path = self.bounded(directory, 'web_resample.json')
        markers = read_object(path) if path.exists() else {}
        if reason:
            markers[filename] = reason[:200]
        else:
            markers.pop(filename, None)
        write_json(path, markers)

    # 把固定 argv 经 shlex 转为可复制的终端命令，包含环境准备和只读入口。
    def commands(self, selected, cameras):
        prefix = f"cd {shlex.quote(str(self.root))}\nsource /opt/ros/humble/setup.bash\nsource install/setup.bash"
        raw, live, servo, bridge = self.launch_commands(selected, cameras)
        return {'终端准备': prefix, '首次启动双相机（终端 A）': shlex.join(raw),
                '用所选内参重启双相机（终端 A）': shlex.join(live),
                '启动只读机器人桥（终端 B）': shlex.join(bridge),
                '启动完整只读伺服（先停止独立相机及桥接）': shlex.join(servo)}

    # 按在线设备或标定结果补足序列号/分辨率/路径，生成原图、加载内参、伺服与只读桥接入口。
    def launch_commands(self, selected, cameras):
        raw = ['ros2', 'launch', 'vservo_ros2', 'dual_camera.launch.py']
        live = raw.copy()
        servo = ['ros2', 'launch', 'vservo_ros2', 'visual_servo.launch.py', 'execute:=false', 'target_id:=aruco_42']
        bridge = ['ros2', 'run', 'vservo_ros2', 'robot_bridge', '--ros-args', '-p', 'execute:=false']
        size = next((cameras[r]['size'] for r in ROLES if cameras[r].get('online')), None)
        if not size:
            size = next((file['metrics']['image_size'] for role in ROLES
                         if (file := assess(selected[role]['intrinsic'], role, 'intrinsic', None))['valid']), None)
        if size and size != [640, 480]:
            profile = [f'width:={size[0]}', f'height:={size[1]}']
            raw += profile
            live += profile
            servo += profile
        for role in ROLES:
            meta = (cameras[role].get('metadata') or {}).get('camera', {})
            intrinsic = assess(selected[role]['intrinsic'], role, 'intrinsic', cameras[role])
            serial = meta.get('serial') if cameras[role]['online'] else intrinsic.get('serial')
            if serial:
                arg = f'{role}_serial:={serial}'
                raw.append(arg)
                live.append(arg)
                servo.append(arg)
            for kind in KINDS:
                path = selected[role][kind]
                file = assess(path, role, kind, cameras[role])
                if kind == 'intrinsic' and file['valid'] and file['compatible'] is not False:
                    live.append(f'{role}_intrinsics:={path}')
                servo.append(f'{role}_{FILES[kind]}:={path}')
        return raw, live, servo, bridge
