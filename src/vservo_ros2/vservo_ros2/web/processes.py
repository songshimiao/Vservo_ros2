"""Fixed ROS entry points owned by the Web console."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


class Processes:
    def __init__(self, workspace, link):
        self.ws = workspace
        self.link = link
        self.lock = threading.RLock()
        self.children = {}
        self.retired = {}

    def state(self):
        # The HTTP poll can continue while a workflow transition is waiting on ROS.
        codes = {name: child.poll() for name, (child, _, _) in self.children.copy().items()}
        return {name: dict(running=code is None, exit_code=code) for name, code in codes.items()}

    def running(self, name):
        entry = self.children.get(name)
        return entry is not None and entry[0].poll() is None

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

    def log_tail(self, name):
        if name not in ('cameras', 'calibration', 'bridge', 'servo'):
            raise ValueError('未知节点组')
        path = self.ws.data/'web'/f'{name}.log'
        if not path.is_file():
            return ''
        with Path(path).open('rb') as stream:
            stream.seek(max(0, path.stat().st_size - 8192))
            return stream.read().decode(errors='replace')

    def close(self):
        with self.lock:
            for name in ('servo', 'calibration', 'bridge', 'cameras'):
                self.stop(name)
