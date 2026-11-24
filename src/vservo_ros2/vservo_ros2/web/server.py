"""Loopback-only browser console for calibration and visual servo."""
import argparse
from datetime import datetime
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import signal
import subprocess
from tempfile import TemporaryDirectory
import threading
import yaml
from urllib.parse import parse_qs, urlsplit

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from .ros_link import RosLink
from .processes import Processes
from .workspace import KINDS, ROLES, Workspace, assess, read_object
from ..control import validate_motion_limits

CAMERA_NODES = {('camera', f'/{role}') for role in ROLES}
CALIBRATION_NODES = {('calibration', f'/{role}') for role in ROLES}
BRIDGE = ('robot_bridge', '/')
SERVO_NODES = CAMERA_NODES | {BRIDGE, ('localizer', '/'), ('servo', '/')} | {('detector', f'/{r}') for r in ROLES}


def diagnostics(snapshot, files, domain):
    result = [dict(name='ROS_DOMAIN_ID', ok=domain.isdecimal(), detail=domain,
                   hint='所有终端和 Web 服务使用同一域，现场默认 99。')]
    values = snapshot['values']
    for role, camera in snapshot['cameras'].items():
        log = values.get(f'{role}_camera_log', '')
        busy = any(word in log.lower() for word in ('busy', 'uvc_open failed'))
        result.extend([
            dict(name=f'{role.upper()} 图像流', ok=camera['online'],
                 detail=f"{camera['hz']} Hz" if camera['online'] else '无新帧或时间戳已超时',
                 hint='检查相机节点、ROS 域、USB3 线缆和供电。'),
            dict(name=f'{role.upper()} metadata', ok=bool(camera['online'] and camera['metadata']),
                 detail=(camera['metadata'] or {}).get('source', '等待发布'),
                 hint='确认 metadata 使用 reliable / transient local。'),
            dict(name=f'{role.upper()} USB / 相机日志', ok=camera['online'] and not busy,
                 detail=log or '当前未收到相机告警',
                 hint='出现 UVC busy(-6) 时查看相机终端日志，关闭重复预览或采集进程。'),
        ])
        for action, ready in snapshot['services'][role].items():
            result.append(dict(name=f'/{role}/calibration/{action}', ok=ready,
                               detail='服务可用' if ready else '未发现 Trigger 服务',
                               hint='在标定页面启动当前会话采集。'))
        for kind, file in files[role].items():
            result.append(dict(name=f'{role.upper()} {kind}', ok=file['compatible'] is True,
                               detail=file['reason'], hint='核对相机序列号、分辨率与所选文件。'))
    result.extend([
        dict(name='base → flange TF', ok=snapshot['tf']['ok'],
             detail=snapshot['tf'].get('reason', '检查动态变换时间戳'),
             hint='检查只读机器人桥、控制器连接和 TF 时间连续性。'),
        dict(name='机器人桥状态', ok=values.get('robot') in ('READ_ONLY', 'DISARMED', 'ARMED')
             or bool((values.get('follow') or {}).get('enabled')),
             detail=values.get('robot', '未收到状态'),
             hint='启动手眼采集或双画面伺服监控。'),
    ])
    return result


def authorize_action(workspace, snapshot, name, action, confirmed):
    """Bind each Trigger to the fresh preview of the exact session and mode."""
    plan = workspace.plan(name)
    role, kind = plan['role'], plan['kind']
    status = snapshot['values'].get(f'{role}_preview') or {}
    expected = str(workspace.session_dir(name))
    output = workspace.output(plan)
    if action not in ('capture', 'solve'):
        raise ValueError('未知操作')
    if action == 'solve' and plan['purpose'] != 'train':
        raise ValueError('独立验证会话只采集，使用验证脚本')
    if not snapshot['services'][role][action]:
        raise ValueError(f'/{role}/calibration/{action} 不可用')
    if (snapshot['calibration_nodes'][role] != 1 or status.get('role') != role
            or status.get('session_dir') != expected or status.get('mode') != kind
            or status.get('output') != str(output)):
        raise ValueError('采集节点会话、输出或阶段不匹配。请启动当前 Web 会话。')
    if plan.get('board') and status.get('board') != plan['board']:
        raise ValueError('采集节点的标定板规格与会话不匹配，请重启本会话采集。')
    if action == 'solve':
        if not status.get('protect_output') or output.exists() or output.is_symlink():
            raise ValueError('输出未启用保护或结果已存在，请创建新会话')
    if kind == 'handeye':
        if confirmed is not True:
            raise ValueError('请确认本阶段的装配检查清单')
        other = next(r for r in ROLES if r != role)
        if (snapshot['values'].get(f'{other}_preview') or {}).get('mode') == 'handeye':
            raise ValueError('另一台相机的手眼采集节点正在运行，请先停止它')
        if action == 'capture' and snapshot['values'].get('robot') != 'READ_ONLY':
            raise ValueError('需要机器人桥处于 READ_ONLY')
    return plan


class Console:
    def __init__(self, workspace, link, allow_motion=False):
        self.ws = workspace
        self.link = link
        self.processes = Processes(workspace, link)
        self.allow_motion = allow_motion
        self.speed_percent = 100.0
        config_path = workspace.root/'src/vservo_ros2/config/control.yaml'
        config = yaml.safe_load(config_path.read_text()) if config_path.is_file() else {}
        limits = config.get('/robot_bridge', config.get('robot_bridge', {})).get('ros__parameters', {})
        self.motion_profile = dict(linear_mm_s=limits.get('linear_speed_m_s', 0.1)*1000,
                                   angular_deg_s=math.degrees(limits.get('angular_speed_rad_s', math.radians(30))),
                                   joint_accel_rad_s2=limits.get('max_joint_accel_rad_s2', 0.5))
        validate_motion_limits(self.motion_profile['linear_mm_s']/1000,
                               math.radians(self.motion_profile['angular_deg_s']),
                               self.motion_profile['joint_accel_rad_s2'])
        self.domain = os.environ.get('ROS_DOMAIN_ID', '0')
        self.token = secrets.token_hex(32)
        self.lock = threading.Lock()
        self.log_path = self.ws.data / 'web' / 'operations.jsonl'
        self.log('startup', 'Web 操作台已启动')

    def log(self, event, message):
        entry = dict(time=datetime.now().astimezone().isoformat(timespec='seconds'),
                     event=event, message=str(message)[:1000])
        with self.lock:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            if self.log_path.exists() and self.log_path.stat().st_size > 1024*1024:
                self.log_path.replace(self.log_path.with_suffix('.previous.jsonl'))
            with self.log_path.open('a', encoding='utf-8') as out:
                out.write(json.dumps(entry, ensure_ascii=False)+'\n')

    def state(self, query):
        snapshot = self.link.snapshot()
        selections, files, candidates = {}, {}, {}
        for role in ROLES:
            selections[role], files[role], candidates[role] = {}, {}, {}
            for kind in KINDS:
                selected = self.ws.select(role, kind, query.get(f'{role}_{kind}', [''])[0])
                selections[role][kind] = selected
                camera = snapshot['cameras'][role]
                file = assess(selected, role, kind, camera)
                file['path'] = str(selected.relative_to(self.ws.data))
                file['verification'] = self.ws.verification(role, kind, selected) if file['valid'] else None
                files[role][kind] = file
                candidates[role][kind] = [
                    dict(path=str(path.relative_to(self.ws.data)), **assess(path, role, kind, camera))
                    for path in self.ws.candidate_paths(role, kind)]
        return dict(**snapshot, files=files, candidates=candidates, domain=self.domain, token=self.token,
                    processes=self.processes.state(), allow_motion=self.allow_motion,
                    speed_percent=self.speed_percent, motion_profile=dict(self.motion_profile),
                    commands=self.ws.commands(selections, snapshot['cameras']))

    def launch_setup(self, selected, snapshot):
        selections = {r: {k: self.ws.select(r, k, selected.get(f'{r}_{k}', '')) for k in KINDS} for r in ROLES}
        return selections, self.ws.launch_commands(selections, snapshot['cameras'])

    def start_calibration(self, body, snapshot, nodes):
        plan = self.ws.plan(body['name'])
        role, kind = plan['role'], plan['kind']
        selections, (_, cameras, _, bridge) = self.launch_setup(body.get('selected', {}), snapshot)
        camera = snapshot['cameras'][role]
        expected = None
        if kind == 'handeye':
            path = selections[role]['intrinsic']
            if not assess(path, role, 'intrinsic', camera)['valid'] or f'{role}_intrinsics:={path}' not in cameras:
                raise ValueError('请为当前相机选择匹配的有效内参')
            expected = read_object(path)
        if plan['purpose'] == 'check':
            self.ws.verification_run(plan)  # Refuse a changed calibration before collecting data.
        if self.processes.running('servo'):
            if snapshot['values'].get('robot') == 'ARMED' or (snapshot['values'].get('follow') or {}).get('enabled'):
                raise ValueError('请先停止伺服运动')
            self.processes.stop('servo')
        elif nodes & CAMERA_NODES and not self.processes.running('cameras'):
            if ('camera', f'/{role}') not in nodes:
                raise ValueError('当前相机没有启动。请停止外部相机入口，再用 Web 启动双相机。')
            if expected and any((camera['metadata'] or {}).get(key) != expected.get(key)
                                for key in ('source', 'camera_matrix', 'dist_coeffs')):
                raise ValueError('外部相机未加载所选内参。请关闭原入口，由 Web 启动相机。')
            cameras = None
        if cameras:
            self.processes.start('cameras', cameras, CAMERA_NODES)
        if kind == 'handeye':
            if BRIDGE not in nodes or self.processes.running('bridge') or ('servo', '/') in nodes:
                self.processes.start('bridge', bridge, {BRIDGE})
            elif snapshot['values'].get('robot') != 'READ_ONLY':
                raise ValueError('外部机器人桥需要处于 READ_ONLY')
        self.processes.start('calibration', self.ws.calibration_launch(plan, resume=True), CALIBRATION_NODES)

    def start_servo(self, body, snapshot):
        selections, (_, _, servo, _) = self.launch_setup(body.get('selected', {}), snapshot)
        sizes = set()
        for role, paths in selections.items():
            files = {k: assess(path, role, k, snapshot['cameras'][role]) for k, path in paths.items()}
            if (any(not f['valid'] or f['compatible'] is False for f in files.values())
                    or files['intrinsic']['serial'] != files['handeye']['serial']):
                raise ValueError(f'{role.upper()} 需要匹配的有效内参与手眼结果')
            sizes.add(tuple(files['intrinsic']['metrics']['image_size']))
        if len(sizes) != 1:
            raise ValueError('两路相机需使用相同的已标定分辨率')
        for name in ('calibration', 'bridge', 'cameras', 'servo'):
            self.processes.stop(name)
        if self.allow_motion:
            servo[servo.index('execute:=false')] = 'execute:=true'
            servo.append('hardware_ready:=true')
            servo.append('auto_start:=true')
        servo.append(f'speed_scale:={self.speed_percent/100}')
        servo.extend([f"linear_speed_m_s:={self.motion_profile['linear_mm_s']/1000}",
                      f"angular_speed_rad_s:={math.radians(self.motion_profile['angular_deg_s'])}",
                      f"max_joint_accel_rad_s2:={self.motion_profile['joint_accel_rad_s2']}"])
        self.processes.start('servo', servo, SERVO_NODES | CALIBRATION_NODES)

    def verify_calibration(self, name):
        plan = self.ws.plan(name)
        args, report = self.ws.verification_run(plan)
        if len(self.ws.session(name)['samples']) < 5:
            raise ValueError('独立验证至少需要 5 个样本')
        report.parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(dir=report.parent) as temporary:
            output = Path(temporary)/report.name
            run = subprocess.run([*args[:-1], str(output)], cwd=self.ws.root, capture_output=True,
                                 text=True, timeout=35, check=False)
            if not output.is_file():
                raise ValueError(run.stderr.strip()[-1000:] or f'验证退出码 {run.returncode}')
            exitfile = output.with_suffix('.exit')
            exitfile.write_text(f'{run.returncode}\n')
            output.replace(report)
            exitfile.replace(report.with_suffix('.exit'))
        result = self.report(report.name)
        return dict(success=run.returncode == 0, message=f'独立验证退出码 {run.returncode}',
                    report=report.name, metrics=result['data'])

    def control(self, action, body):
        snapshot = self.link.snapshot()
        nodes = set(self.link.get_node_names_and_namespaces())
        if action == 'start-cameras':
            if self.processes.running('servo'):
                raise ValueError('双相机由伺服监控管理，请先结束监控')
            _, (_, cameras, _, _) = self.launch_setup(body.get('selected', {}), snapshot)
            self.processes.stop('calibration')
            self.processes.stop('cameras')
            self.processes.start('cameras', cameras, CAMERA_NODES)
        elif action == 'start-calibration':
            self.start_calibration(body, snapshot, nodes)
        elif action == 'start-monitor':
            self.start_servo(body, snapshot)
            return dict(success=True, message='监控已启动，持续伺服已开启' if self.allow_motion else '只读监控已启动')
        elif action == 'stop-monitor':
            self.processes.stop('servo')
        elif action == 'speed':
            percent = body.get('percent')
            if type(percent) not in (int, float) or not math.isfinite(percent) or not 10 <= percent <= 100:
                raise ValueError('速度倍率应为 10–100 之间的数值')
            profile = dict(self.motion_profile)
            has_profile = any(key in body for key in profile)
            for key in profile:
                if key in body:
                    value = body[key]
                    if type(value) not in (int, float) or not math.isfinite(value):
                        raise ValueError('运动参数应为有限数值')
                    profile[key] = float(value)
            validate_motion_limits(profile['linear_mm_s']/1000,
                                   math.radians(profile['angular_deg_s']), profile['joint_accel_rad_s2'])
            if self.processes.running('servo'):
                if self.link.get_node_names_and_namespaces().count(BRIDGE) != 1:
                    raise ValueError('需恰好一个机器人桥')
                result = (self.link.set_speed(float(percent), profile) if has_profile
                          else self.link.set_speed(float(percent)))
                if not result['success']:
                    return result
            self.speed_percent = float(percent)
            self.motion_profile = profile
            return dict(success=True, message=f'运动速度已应用，倍率 {percent:g}%', percent=percent,
                        motion_profile=dict(profile))
        elif action in ('start-motion', 'stop-motion'):
            enable = action == 'start-motion'
            if not self.allow_motion or not self.processes.running('servo'):
                raise ValueError('需要已启用运动选项的 Web 伺服监控')
            if enable and nodes & CALIBRATION_NODES:
                raise ValueError('请先结束标定采集')
            if self.link.get_node_names_and_namespaces().count(BRIDGE) != 1:
                raise ValueError('需恰好一个机器人桥')
            return self.link.arm(enable)
        return dict(success=True, message='请求已执行，实时状态见页面')

    def reports(self):
        paths = []
        if self.ws.reports.is_dir():
            for path in sorted(self.ws.reports.rglob('*'), reverse=True):
                if path.suffix in ('.json', '.exit') and path.is_file():
                    paths.append(str(path.with_suffix('.json').relative_to(self.ws.reports)))
                if len(paths) >= 200:
                    break
        return sorted(set(paths), reverse=True)

    def report(self, name):
        if Path(name).suffix != '.json':
            raise ValueError('报告格式无效')
        path = self.ws.bounded(self.ws.reports, name)
        data = read_object(path) if path.is_file() else None
        exitfile = self.ws.bounded(self.ws.reports, str(Path(name).with_suffix('.exit')))
        if not path.is_file() and not exitfile.is_file():
            raise FileNotFoundError(name)
        code = int(exitfile.read_text()[:8]) if exitfile.is_file() else None
        return dict(data=data, exit_code=code,
                    inferred_code=(0 if data['passed'] else 2) if data and type(data.get('passed')) is bool and code is None else None)

    def mutate(self, route, body):
        if route == '/api/page':
            self.log('page', '页面已打开')
            return {}
        if route == '/api/diagnostics':
            state = self.state({k: [v] for k, v in body.get('selected', {}).items()})
            checks = diagnostics(state, state['files'], self.domain)
            self.log('diagnostics', f"{sum(c['ok'] for c in checks)}/{len(checks)} 项通过")
            return checks
        if route == '/api/session':
            role, kind = body['role'], body['kind']
            default_board = 'compact' if (role, kind) == ('eth', 'handeye') else 'a4'
            result = self.ws.new_session(role, kind, body['purpose'], body.get('selected', ''),
                                         board_profile=body.get('board_profile', default_board))
            self.log('session', result['name'])
            return result
        if route == '/api/resample':
            with self.lock:
                self.ws.mark(body['name'], body['filename'], body.get('reason', ''))
            self.log('resample', f"{body['name']}/{body['filename']}")
            return {}
        if route in ('/api/capture', '/api/solve'):
            action = route.rsplit('/', 1)[-1]
            try:
                with self.processes.lock:
                    plan = authorize_action(self.ws, self.link.snapshot(), body['name'], action, body.get('confirmed'))
                    result = self.link.trigger(plan['role'], action)
                if result['success'] and action == 'solve':
                    file = assess(self.ws.output(plan), plan['role'], plan['kind'], None)
                    result['result'] = file
                    result['output'] = str(self.ws.output(plan))
            except (OSError, ValueError, KeyError) as exc:
                result = dict(success=False, message=str(exc))
            self.log(action, f"{body.get('name', '')} success={result['success']} {result['message']}")
            return result
        action = route.removeprefix('/api/')
        if route == '/api/verify' or action in ('start-cameras', 'start-calibration', 'start-monitor', 'stop-monitor',
                                              'start-motion', 'stop-motion', 'speed'):
            with self.processes.lock:
                try:
                    if action == 'verify':
                        if self.processes.running('servo') and self.allow_motion:
                            raise ValueError('结束可运动的伺服监控后再运行标定验证')
                        result = self.verify_calibration(body['name'])
                    else:
                        result = self.control(action, body)
                except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as exc:
                    result = dict(success=False, message=str(exc))
            self.log(action, result['message'])
            return result
        raise FileNotFoundError(route)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, console, assets):
        self.console = console
        self.assets = assets
        self.slots = threading.BoundedSemaphore(16)
        super().__init__(address, Handler)

    def process_request(self, request, address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            request.settimeout(8)
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # Avoid noisy image polling logs.

    def send_bytes(self, body, content_type, status=200, attachment=False):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; img-src 'self' blob:; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
        if attachment:
            self.send_header('Content-Disposition', 'attachment')
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, result, status=200, attachment=False):
        self.send_bytes(json.dumps(result, ensure_ascii=False, allow_nan=False).encode(),
                        'application/json; charset=utf-8', status, attachment)

    def do_GET(self):
        self.handle_request(False)

    def do_POST(self):
        self.handle_request(True)

    def handle_request(self, post):
        try:
            host = self.headers.get('Host', '')
            if urlsplit('//'+host).hostname not in ('localhost', '127.0.0.1'):
                self.send_json({'error': '请通过本机地址或 SSH 转发访问'}, 403)
                return
            path = urlsplit(self.path)
            app = self.server.console
            if post:
                origin = self.headers.get('Origin')
                if (origin != f'http://{host}' or self.headers.get_content_type() != 'application/json'
                        or not hmac.compare_digest(self.headers.get('X-Vservo-Console', ''), app.token)):
                    self.send_json({'error': '请求来源或会话凭据无效，请刷新页面'}, 403)
                    return
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 8192:
                    raise ValueError('请求大小无效')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('请求应为 JSON 对象')
                self.send_json(app.mutate(path.path, body))
                return
            self.get_resource(path.path, parse_qs(path.query), app)
        except FileNotFoundError:
            self.send_json({'error': '未找到资源或报告'}, 404)
        except (ValueError, KeyError, TypeError, AttributeError, OSError) as exc:
            if not isinstance(exc, (BrokenPipeError, ConnectionResetError, TimeoutError)):
                self.send_json({'error': str(exc)}, 400)

    def get_resource(self, route, query, app):
        if route in ('/charuco-a4.pdf', '/charuco-compact.pdf', '/aruco-42.pdf'):
            self.send_bytes((app.ws.root/'docs'/route[1:]).read_bytes(), 'application/pdf', attachment=True)
            return
        if route == '/camera-layout.svg':
            self.send_bytes((app.ws.root/'docs'/'双相机共平台安装示意.svg').read_bytes(), 'image/svg+xml')
            return
        if route == '/api/process-log':
            self.send_json(app.processes.log_tail(query['name'][0]))
            return
        if route in ('/', '/app.js', '/style.css'):
            asset = self.server.assets / ('index.html' if route == '/' else route[1:])
            self.send_bytes(asset.read_bytes(), mimetypes.guess_type(asset.name)[0]+'; charset=utf-8')
        elif route == '/api/state':
            self.send_json(app.state(query))
        elif route == '/api/image':
            role, variant = query['role'][0], query.get('variant', ['raw'])[0]
            frame = app.link.jpeg(role, variant)
            if frame is None:
                raise FileNotFoundError()
            self.send_bytes(frame, 'image/jpeg')
        elif route == '/api/sessions':
            directories = sorted(app.ws.sessions.iterdir(), reverse=True) if app.ws.sessions.is_dir() else []
            self.send_json([p.name for p in directories[:200] if p.is_dir()
                            and ((p/'samples.json').is_file() or (p/'web_session.json').is_file())])
        elif route == '/api/session':
            self.send_json(app.ws.session(query['name'][0]))
        elif route == '/api/sample':
            name, filename = query['name'][0], query['filename'][0]
            if filename not in {s['filename'] for s in app.ws.session(name)['samples']}:
                raise FileNotFoundError()
            path = app.ws.bounded(app.ws.session_dir(name), filename)
            image = cv2.imread(str(path))
            if image is None:
                raise FileNotFoundError()
            height, width = image.shape[:2]
            if width > 320:
                image = cv2.resize(image, (320, max(1, round(height*320/width))), interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode('.jpg', image)
            if not ok:
                raise ValueError('缩略图编码失败')
            self.send_bytes(jpeg.tobytes(), 'image/jpeg')
        elif route == '/api/reports':
            self.send_json(app.reports())
        elif route == '/api/report':
            report = app.report(query['name'][0])
            if 'download' in query:
                if report['data'] is None:
                    raise FileNotFoundError()
                self.send_json(report['data'], attachment=True)
            else:
                self.send_json(report)
        elif route == '/api/logs':
            with app.lock:
                rows = app.log_path.read_text().splitlines()[-100:]
            self.send_json([json.loads(row) for row in reversed(rows)])
        else:
            raise FileNotFoundError()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=os.environ.get('WS', '/home/jetson/fairino_ws/Vservo_ros2'))
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--preview-fps', type=float, default=5)
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument('--allow-motion', dest='allow_motion', action='store_true', help='Enable automatic servo on monitor startup')
    motion.add_argument('--read-only', dest='allow_motion', action='store_false', help='Start monitor without robot motion')
    parser.set_defaults(allow_motion=True)
    args = parser.parse_args()
    if not 0 < args.port <= 65535 or not 1 <= args.preview_fps <= 10:
        parser.error('port must be 1–65535 and preview-fps 1–10')
    workspace = Workspace(args.workspace)
    if not (workspace.root/'src/vservo_ros2/package.xml').is_file():
        parser.error('workspace must point to Vservo_ros2')
    assets = Path(get_package_share_directory('vservo_ros2'))/'web'
    # The HTTP loop owns shutdown. ROS-only signal handlers would stop the
    # executor while leaving the listening socket and stale page running.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    link = RosLink(args.preview_fps)
    def spin_ros():
        try:
            rclpy.spin(link)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()
    console = Console(workspace, link, allow_motion=args.allow_motion)
    try:
        with Server(('127.0.0.1', args.port), console, assets) as server:
            print(f'Web console: http://127.0.0.1:{args.port}', flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        console.processes.close()
        link.close()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=3)
        link.destroy_node()
