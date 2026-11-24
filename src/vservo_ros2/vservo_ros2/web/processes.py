# Web 所启动节点组的生命周期管理；只管理自己登记的子进程，启动前检查 ROS 域内同名冲突。
"""Fixed ROS entry points owned by the Web console."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


# 记录节点组的 Popen、启动参数和冲突集合，另记 DDS 端点消退的宽限时间。
class Processes:
    # RLock 串行工作流切换，children 保存活动子进程，retired 保存已退出端点的等待期限。
    def __init__(self, workspace, link):
        self.ws = workspace
        self.link = link
        self.lock = threading.RLock()
        self.children = {}
        self.retired = {}

    # 用 poll 生成进程状态快照，不等待进程退出，使 HTTP 轮询在工作流切换期间仍可响应。
    def state(self):
        # The HTTP poll can continue while a workflow transition is waiting on ROS.
        codes = {name: child.poll() for name, (child, _, _) in self.children.copy().items()}
        return {name: dict(running=code is None, exit_code=code) for name, code in codes.items()}

    # 只在已登记子进程仍未退出时返回 True。
    def running(self, name):
        entry = self.children.get(name)
        return entry is not None and entry[0].poll() is None

    # 同名同参数时复用；变参则停止旧组，再等冲突端点消失，以参数列表启动独立进程组并保存日志。
    def start(self, name, argv, conflicts):
        argv = tuple(argv)
        if self.running(name):
            if self.children[name][1] == argv:
                return
            self.stop(name)
        # A process can exit before Fast DDS removes its endpoints from the graph.
        deadline = max([time.monotonic() + 1, *(self.retired.get(node, 0) for node in conflicts)])
        while occupied := set(self.link.get_node_names_and_namespaces()).intersection(conflicts):
            if time.monotonic() >= deadline:
                raise ValueError('ROS 域内已有节点，请先停止原入口：'+', '.join(
                    namespace.rstrip('/')+'/'+node for node, namespace in sorted(occupied)))
            time.sleep(0.1)
        path = self.ws.data/'web'/f'{name}.log'
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('wb') as log:
            child = subprocess.Popen(argv, cwd=self.ws.root, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.children[name] = child, argv, conflicts

    # 先 SIGINT 有序退出，超时再 TERM/KILL；检查进程组残留，随后为 DDS 图消退留出窗口。
    def stop(self, name):
        entry = self.children.get(name)
        if entry is None:
            return
        child, argv, conflicts = entry
        for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            try:
                if sig == signal.SIGINT and argv[1] == 'launch' and child.poll() is None:
                    child.send_signal(sig)  # Let ros2 launch shut its nodes down in order.
                else:
                    os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            try:
                child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            # A crashed wrapper can leave a launch child alive in the same group.
            if sig == signal.SIGKILL:
                break
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                continue
            break
        self.retired.update(dict.fromkeys(conflicts, time.monotonic() + 12))
        self.children.pop(name, None)

    # 仅允许固定节点组名，最多读日志末尾 8 KiB，避免轮询时读取整份大日志。
    def log_tail(self, name):
        if name not in ('cameras', 'calibration', 'bridge', 'servo'):
            raise ValueError('未知节点组')
        path = self.ws.data/'web'/f'{name}.log'
        if not path.is_file():
            return ''
        with Path(path).open('rb') as stream:
            stream.seek(max(0, path.stat().st_size - 8192))
            return stream.read().decode(errors='replace')

    # 退出 Web 时依次关闭伺服、标定、桥接、相机等自有进程组。
    def close(self):
        with self.lock:
            for name in ('servo', 'calibration', 'bridge', 'cameras'):
                self.stop(name)
