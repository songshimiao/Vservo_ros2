"""Persistent visual follow with ETH/EIH targets and bounded joint-space reset."""
import importlib.util
import json
from pathlib import Path
import time
import threading
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, TimeoutError as FutureTimeoutError
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .geometry import fill_tf, read_json, validate, pose_matrix, error
from .kinematics import Kinematics, solve_ik_worker, warmup_ik_worker, check_home_path_worker
from .control import in_workspace, JointLimiter, cartesian_velocity_scale, validate_motion_limits
from .ros_utils import param, fresh, LATCHED

LOCALIZATION_TIMEOUT_S = 0.5
CONTROL_PERIOD_S = 0.01
CONTROL_TIMEOUT_S = 0.05


class RobotNode(Node):
    def __init__(self, dedicated_control=False):
        super().__init__('robot_bridge')
        self.control_lock = threading.RLock()
        self.control_stop = threading.Event()
        self.control_thread = None
        self.dedicated_control = dedicated_control
        self.execute = param(self, 'execute', False)
        self.ack = param(self, 'hardware_ready', False)
        self.armed = False
        self.ip = param(self, 'robot_ip', '192.168.58.2')
        sdk = Path(param(self, 'sdk_file', '/home/jetson/fairino_ws/fr_visual_servo/scripts/robot/fairino/Robot.py')).expanduser()
        self.minimum = param(self, 'workspace_min', [-0.6, -0.6, 0.1])
        self.maximum = param(self, 'workspace_max', [0.6, 0.6, 0.8])
        self.max_joint_speed = param(self, 'max_joint_speed_rad_s', 2.0)
        self.max_joint_accel = param(self, 'max_joint_accel_rad_s2', 0.5)
        self.linear_speed = param(self, 'linear_speed_m_s', 0.1)
        self.angular_speed = param(self, 'angular_speed_rad_s', float(np.deg2rad(30)))
        validate_motion_limits(self.linear_speed, self.angular_speed, self.max_joint_accel)
        if (not np.isfinite([self.max_joint_speed, self.max_joint_accel]).all()
                or not 0 < self.max_joint_speed <= 2.0 or self.max_joint_accel <= 0):
            raise ValueError('Joint speed must be in (0, 2.0] rad/s and acceleration must be positive')
        self.speed_scale = param(self, 'speed_scale', 1.0)
        if not np.isfinite(self.speed_scale) or not 0.1 <= self.speed_scale <= 1.0:
            raise ValueError('speed_scale must be between 0.1 and 1.0')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.model = Kinematics()
        self.ik_pool = None
        self.ik_future = None
        self.planner_lock = threading.Lock()
        self.home_pool = None
        self.home_future = None
        self.home_anchor = None
        self.home_path_checker = check_home_path_worker
        self.home = np.deg2rad(np.asarray(param(self, 'home_joint_deg',
                             [90.0, -110.0, 110.0, -180.0, -90.0, 0.0]), float))
        self.home_delay = param(self, 'target_loss_home_delay_s', 0.5)
        if (self.home.shape != (6,) or not np.isfinite(self.home).all()
                or np.any(self.home <= self.model.limits[:, 0]) or np.any(self.home >= self.model.limits[:, 1])
                or not np.isfinite(self.home_delay) or self.home_delay < 0):
            raise ValueError('Invalid reset joint angles or target-loss delay')
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.calibrations = {}
        static_transforms = []
        for role, default_serial in [('eih', 'CV27864000EE'), ('eth', 'CP08863000Z7')]:
            path = param(self, f'{role}_handeye', '')
            serial = param(self, f'{role}_serial', default_serial)
            if path:
                data = read_json(path)
                parent = 'flange' if role == 'eih' else 'base'
                if (data.get('mode') != role or not data.get('validated') or data.get('units') != 'm'
                        or data.get('parent_frame') != parent or data.get('child_frame') != f'{role}_optical_frame'
                        or data['camera']['serial'] != serial):
                    raise ValueError(f'{role} hand-eye metadata mismatch')
                self.calibrations[role] = data
                msg = TransformStamped()
                msg.header.frame_id, msg.child_frame_id = parent, f'{role}_optical_frame'
                msg.header.stamp = self.get_clock().now().to_msg()
                static_transforms.append(fill_tf(msg, data['matrix']))
        if static_transforms:
            # /tf_static retains one TFMessage. Publish the complete tree in
            # one sample so late DDS listeners receive both camera extrinsics.
            self.static.sendTransform(static_transforms)
        if self.execute and (not self.ack or len(self.calibrations) != 2):
            raise ValueError('execute requires hardware_ready and both validated hand-eye files')
        self.follow_requested = param(self, 'auto_start', False) and self.execute and self.ack
        spec = importlib.util.spec_from_file_location('vservo_fairino_sdk', sdk)
        if spec is None or spec.loader is None:
            raise ValueError(f'Cannot load Fairino SDK: {sdk}')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.rpc = module.RPC(self.ip)
        self.joints = self.create_publisher(JointState, '/robot/joint_states', 5)
        self.status = self.create_publisher(String, '/robot/status', 5)
        self.speed_status = self.create_publisher(String, '/robot/speed_status', LATCHED)
        self.follow_status = self.create_publisher(String, '/robot/servo_state', LATCHED)
        self.command = None
        self.command_lock = threading.Lock()
        self.command_epoch = 0
        self.desired_seen = False
        self.last_ik_goal = None
        self.command_group = MutuallyExclusiveCallbackGroup()
        # SDK enable/end and ServoJ may wait for the controller. Receive the
        # watchdog heartbeat independently of both SDK and IK callback groups.
        self.localization_group = MutuallyExclusiveCallbackGroup()
        self.command_error = ''
        self.q = None
        self.limiter = None
        self.localization = None
        self.localization_received = None
        self.loss_since = None
        self.returning_home = self.home_complete = False
        self.fault = None
        self.mode = 'WAITING_TARGET' if self.follow_requested else 'DISARMED' if self.execute else 'READ_ONLY'
        self.last_tick = time.monotonic()
        self.last_servo_sent = None
        self.last_feedback = self.last_status = 0.0
        self.timing = dict(period_ms=0.0, work_ms=0.0, servoj_ms=0.0, enable_ms=0.0,
                           command_gap_ms=0.0, stage='idle', timeout_ms=CONTROL_TIMEOUT_S*1000)
        self.fault_timing = None
        self.create_subscription(PoseStamped, '/servo/next_flange_pose', self.on_command, 1,
                                 callback_group=self.command_group)
        self.create_subscription(PoseStamped, '/servo/desired_flange_pose', self.on_desired, 1,
                                 callback_group=self.command_group)
        self.create_subscription(String, '/localization/state', self.on_localization, 1,
                                 callback_group=self.localization_group)
        self.create_service(SetBool, '~/arm', self.arm)
        if not dedicated_control:
            self.create_timer(CONTROL_PERIOD_S, self.tick)
        self.create_timer(0.2, self.publish_speed)
        self.create_timer(0.2, self.publish_follow_state)
        self.publish_speed()
        self.publish_follow_state()
        if dedicated_control:
            # Numerical IK must not hold the control worker's Python GIL.
            # Spawn avoids forking an initialized ROS/DDS/SDK process.
            self.ik_pool = ProcessPoolExecutor(max_workers=1,
                                              mp_context=multiprocessing.get_context('spawn'))
            self.ik_future = self.ik_pool.submit(warmup_ik_worker)
        self.get_logger().info(f'FR5 bridge connected. execute={self.execute}; armed=False')

    def on_localization(self, msg):
        with self.command_lock:
            previous = self.localization
            self.localization, self.localization_received = msg.data, time.monotonic()
            if msg.data != previous or msg.data not in ('COARSE', 'FINE'):
                self.command_epoch += 1
                self.command = None
                self.last_ik_goal = None

    def publish_follow_state(self):
        with self.command_lock:
            localization, received = self.localization, self.localization_received
        age = None if received is None else max(0.0, time.monotonic()-received)
        self.follow_status.publish(String(data=json.dumps(dict(
            enabled=self.follow_requested, drive_armed=self.armed, mode=self.mode,
            fault=self.fault, reason=self.command_error if self.mode == 'WAITING_COMMAND' else '',
            localization_state=localization, localization_age_s=age,
            localization_timeout_s=LOCALIZATION_TIMEOUT_S,
            control_timing=dict(self.timing), fault_timing=self.fault_timing,
            home_joint_deg=np.rad2deg(self.home).tolist()))))

    def validate_parameters(self, parameters):
        limits = {name: self.get_parameter(name).value for name in
                  ('linear_speed_m_s', 'angular_speed_rad_s', 'max_joint_accel_rad_s2')}
        for parameter in parameters:
            if parameter.name == 'speed_scale':
                value = parameter.value
                if type(value) is not float or not np.isfinite(value) or not 0.1 <= value <= 1.0:
                    return SetParametersResult(successful=False, reason='速度倍率应在 10%–100% 之间')
            if parameter.name in limits:
                if type(parameter.value) is not float:
                    return SetParametersResult(successful=False, reason='运动参数应为浮点数')
                limits[parameter.name] = parameter.value
        try:
            validate_motion_limits(limits['linear_speed_m_s'], limits['angular_speed_rad_s'],
                                   limits['max_joint_accel_rad_s2'])
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def publish_speed(self):
        self.speed_scale = self.get_parameter('speed_scale').value
        self.linear_speed = self.get_parameter('linear_speed_m_s').value
        self.angular_speed = self.get_parameter('angular_speed_rad_s').value
        self.max_joint_accel = self.get_parameter('max_joint_accel_rad_s2').value
        self.speed_status.publish(String(data=json.dumps(dict(
            percent=round(self.speed_scale*100, 3), max_joint_speed_rad_s=self.max_joint_speed*self.speed_scale,
            max_joint_accel_rad_s2=self.max_joint_accel, linear_speed_m_s=self.linear_speed,
            angular_speed_rad_s=self.angular_speed,
            effective_linear_mm_s=self.linear_speed*self.speed_scale*1000,
            effective_angular_deg_s=float(np.degrees(self.angular_speed)*self.speed_scale)))))

    def on_command(self, msg):
        self.receive_pose(msg, desired=False)

    def on_desired(self, msg):
        self.receive_pose(msg, desired=True)

    def receive_pose(self, msg, desired):
        with self.command_lock:
            if not desired and self.desired_seen:
                return
            if desired:
                self.desired_seen = True
            epoch = self.command_epoch
            seed = self.last_ik_goal if desired and self.last_ik_goal is not None else self.q
        try:
            if msg.header.frame_id != 'base' or not fresh(self, msg.header.stamp, 0.3):
                raise ValueError('Invalid command stamp/frame')
            T = pose_matrix(msg.pose)
            current_q = self.q
            if current_q is None or not in_workspace(T, self.minimum, self.maximum):
                raise ValueError('Command outside workspace or no robot state')
            t, a = error(self.model.fk(current_q), T)
            if not desired and (t > 0.005 or a > np.deg2rad(3)):
                raise ValueError('Command step exceeds 5 mm / 3 degrees')
            goal = self.solve_ik(T, current_q if seed is None else seed)
            if not desired and np.max(np.abs(goal-current_q)) > np.deg2rad(5):
                raise ValueError('IK branch jump')
            if not fresh(self, msg.header.stamp, 0.3):
                raise ValueError('Target expired while solving IK')
            with self.command_lock:
                if epoch == self.command_epoch:
                    self.command = (msg.header.stamp, time.monotonic(), goal)
                    self.last_ik_goal = goal if desired else None
                    self.command_error = ''
        except Exception as exc:
            with self.command_lock:
                if epoch == self.command_epoch:
                    self.command = None
                    self.command_error = str(exc)
            self.status.publish(String(data=f'HOLD: {exc}'))

    def solve_ik(self, target, seed):
        if self.ik_pool is None:
            return self.model.ik(target, seed)
        with self.planner_lock:
            if self.home_future is not None and not self.home_future.done():
                raise ValueError('Reset path worker busy')
            if self.ik_future is not None and not self.ik_future.done():
                raise ValueError('IK worker busy / warming up')
            self.ik_future = self.ik_pool.submit(solve_ik_worker, target, seed)
            future = self.ik_future
        try:
            return future.result(timeout=0.2)
        except FutureTimeoutError:
            future.cancel()
            raise ValueError('IK worker exceeded 200 ms') from None

    def arm(self, request, response):
        with self.control_lock:
            return self._arm(request, response)

    def _arm(self, request, response):
        if not request.data:
            failure = self.disarm()
            response.success = failure is None
            response.message = failure or '持续伺服已停止'
        elif not self.execute or not self.ack:
            response.success, response.message = False, '机器人桥未开放实体运动'
        elif self.follow_requested:
            response.success, response.message = True, '持续伺服已开启'
        else:
            self.follow_requested = True
            self.loss_since = self.fault = None
            self.fault_timing = None
            self.returning_home = self.home_complete = False
            self.mode = 'WAITING_TARGET'
            response.success, response.message = True, '持续伺服已开启，等待有效目标或复位'
        self.publish_follow_state()
        return response

    def disarm(self):
        with self.control_lock:
            return self._disarm()

    def _disarm(self):
        self.follow_requested = False
        failure = None
        if self.armed:
            self.armed = False
            try:
                code = self.rpc.ServoMoveEnd()
                if code != 0:
                    raise RuntimeError(f'controller returned {code}')
            except Exception as exc:
                failure = f'ServoMoveEnd failed: {exc}; use hardware stop'
                self.get_logger().error(failure)
        self.limiter = None
        self.last_servo_sent = None
        self.timing['command_gap_ms'] = 0.0
        with self.command_lock:
            self.command_epoch += 1
            self.command = None
            self.last_ik_goal = None
        self.loss_since = None
        self.returning_home = self.home_complete = False
        self.cancel_home_plan()
        self.mode = 'DISARMED' if self.execute else 'READ_ONLY'
        return failure

    def cancel_home_plan(self):
        with self.planner_lock:
            self.home_anchor = None
            if self.home_future is not None:
                self.home_future.cancel()
                if self.home_future.done():
                    self.home_future = None

    def prepare_home(self):
        # The 81-point preflight is deliberately outside the 100 Hz SDK worker.
        # First finish bounded braking so its start pose remains meaningful.
        if self.armed and self.limiter is not None and np.max(np.abs(self.limiter.v)) > 0.01:
            return 'BRAKING_FOR_HOME', None
        with self.planner_lock:
            if self.home_future is not None and self.home_anchor is None:
                if not self.home_future.done():
                    return 'PREPARING_HOME', None
                self.home_future = None
            if self.home_future is None:
                if self.ik_future is not None and not self.ik_future.done():
                    return 'PREPARING_HOME', None
                pool = self.ik_pool
                if pool is None:
                    # Legacy executor/manual-tick mode used by offline clients.
                    if self.home_pool is None:
                        self.home_pool = ThreadPoolExecutor(max_workers=1)
                    pool = self.home_pool
                self.home_anchor = self.q.copy()
                self.home_future = pool.submit(self.home_path_checker, self.home_anchor, self.home,
                                               self.minimum, self.maximum)
                return 'PREPARING_HOME', None
            if not self.home_future.done():
                return 'PREPARING_HOME', None
            valid = self.home_future.result()  # Already completed: never wait here.
            moved = np.max(np.abs(self.q-self.home_anchor)) > 0.005
            self.home_future, self.home_anchor = None, None
        if moved:
            return 'PREPARING_HOME', None
        if valid is not True:
            raise RuntimeError('Reset path validation failed')
        self.returning_home = True
        return 'RETURNING_HOME', self.home

    def follow_goal(self, now):
        with self.command_lock:
            localization, received, command = self.localization, self.localization_received, self.command
        if received is None:
            return 'WAITING_LOCALIZATION', None
        if now-received > LOCALIZATION_TIMEOUT_S:
            raise RuntimeError(f'Localization heartbeat expired (age={now-received:.3f}s > '
                               f'{LOCALIZATION_TIMEOUT_S:.3f}s)')
        if self.returning_home:
            stopped = self.limiter is not None and np.max(np.abs(self.limiter.v)) < 0.01
            if np.max(np.abs(self.q-self.home)) < np.deg2rad(0.3) and stopped:
                self.returning_home, self.home_complete = False, True
                return 'WAITING_TARGET', self.home
            return 'RETURNING_HOME', self.home
        if localization in ('COARSE', 'FINE'):
            self.cancel_home_plan()
            self.loss_since = None
            self.home_complete = False
            if (command is not None and fresh(self, command[0], 0.3)
                    and now-command[1] <= 0.12):
                return 'FOLLOWING_ETH' if localization == 'COARSE' else 'FOLLOWING_EIH', command[2]
            return 'WAITING_COMMAND', None
        if localization != 'HOLD':
            self.loss_since = None
            self.cancel_home_plan()
            return 'WAITING_COMMAND', None
        if self.loss_since is None:
            self.loss_since = now
        if now-self.loss_since < self.home_delay:
            return 'WAITING_TARGET', None
        if self.home_complete:
            return 'WAITING_TARGET', self.home
        return self.prepare_home()

    def send_servo_goal(self, goal, current):
        if self.control_stop.is_set():
            return
        if not self.armed:
            self.timing['stage'] = 'sdk_enable'
            begin = time.monotonic()
            code = self.rpc.ServoMoveStart()
            self.timing['enable_ms'] = (time.monotonic()-begin)*1000
            if code != 0:
                raise RuntimeError(f'ServoMoveStart returned {code}')
            self.armed = True
            self.last_servo_sent = None
            self.last_tick = time.monotonic()
            self.limiter = JointLimiter(self.q, self.max_joint_speed*self.get_parameter('speed_scale').value,
                                        self.max_joint_accel)
            # Revalidate after a potentially slow enable, before the first
            # position command. A lost heartbeat still faults immediately.
            self.mode, refreshed_goal = self.follow_goal(time.monotonic())
            goal = self.q if refreshed_goal is None else refreshed_goal
        self.timing['stage'] = 'planning'
        if not in_workspace(current, self.minimum, self.maximum):
            raise RuntimeError('Current pose outside workspace')
        if self.limiter is None or np.max(np.abs(self.limiter.q-self.q)) > 0.05:
            raise RuntimeError('Commanded/measured joint following error exceeds 0.05 rad')
        self.limiter.max_speed = self.max_joint_speed*self.get_parameter('speed_scale').value
        self.limiter.acceleration = self.get_parameter('max_joint_accel_rad_s2').value
        scale = self.get_parameter('speed_scale').value
        cartesian_scale = cartesian_velocity_scale(
            self.model, self.limiter.q, self.limiter.desired_velocity(goal), self.limiter.period,
            self.get_parameter('linear_speed_m_s').value*scale,
            self.get_parameter('angular_speed_rad_s').value*scale)
        q_next = self.limiter.step(goal, velocity_scale=cartesian_scale)
        if (np.any(q_next <= self.model.limits[:, 0]) or np.any(q_next >= self.model.limits[:, 1])
                or not in_workspace(self.model.fk(q_next), self.minimum, self.maximum)):
            raise RuntimeError('Joint / workspace limit')
        if self.control_stop.is_set():
            return
        self.timing['stage'] = 'servoj'
        begin = time.monotonic()
        self.check_control_deadline(begin)
        code = self.rpc.ServoJ(np.rad2deg(q_next).tolist(), [0.0]*4,
                               acc=0.0, vel=0.0, cmdT=0.01, filterT=0.0, gain=0.0)
        end = time.monotonic()
        self.timing['servoj_ms'] = (end-begin)*1000
        if code != 0:
            raise RuntimeError(f'ServoJ returned {code}')
        # A slow synchronous SDK call must also trip the watchdog; resetting
        # the timestamp unconditionally would hide a real communications stall.
        self.check_control_deadline(end)
        if end-begin > CONTROL_TIMEOUT_S:
            self.raise_control_watchdog(end-begin)
        self.last_servo_sent = end

    def check_control_deadline(self, now):
        gap = 0.0 if self.last_servo_sent is None else now-self.last_servo_sent
        self.timing['command_gap_ms'] = gap*1000
        if self.armed and gap > CONTROL_TIMEOUT_S:
            self.raise_control_watchdog(gap)

    def raise_control_watchdog(self, gap):
        self.timing['command_gap_ms'] = gap*1000
        raise RuntimeError(f'Control watchdog expired (command_gap={gap*1000:.1f}ms > '
                           f'{CONTROL_TIMEOUT_S*1000:.0f}ms; period={self.timing["period_ms"]:.1f}ms; '
                           f'previous_work={self.timing["work_ms"]:.1f}ms; '
                           f'ServoJ={self.timing["servoj_ms"]:.1f}ms; stage={self.timing["stage"]})')

    def tick(self):
        with self.control_lock:
            self._tick()

    def _tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        self.timing['period_ms'] = dt*1000
        try:
            if self.follow_requested and self.armed:
                self.check_control_deadline(now)
            self.timing['stage'] = 'read_state'
            start = self.get_clock().now()
            code, values = self.rpc.GetActualJointPosDegree()
            end = self.get_clock().now()
            if code != 0 or (end-start).nanoseconds > 80_000_000:
                raise RuntimeError(f'Robot read error/latency: {code}')
            self.q = np.deg2rad(np.asarray(values, float))
            T = self.model.fk(self.q)
            stamp = Time(nanoseconds=(start.nanoseconds+end.nanoseconds)//2).to_msg()
            if now-self.last_feedback >= 0.02:
                msg = TransformStamped()
                msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'base', 'flange'
                self.dynamic.sendTransform(fill_tf(msg, T))
                js = JointState()
                js.header.stamp, js.name = stamp, [f'joint{i+1}' for i in range(6)]
                js.position = self.q.tolist()
                self.joints.publish(js)
                self.last_feedback = now
            if self.follow_requested:
                self.timing['stage'] = 'select_goal'
                self.mode, goal = self.follow_goal(now)
                if goal is not None or self.armed:
                    self.send_servo_goal(self.q if goal is None else goal, T)
            else:
                self.mode = 'FAULT' if self.fault else 'DISARMED' if self.execute else 'READ_ONLY'
            if now-self.last_status >= 0.1:
                self.status.publish(String(data=f'FAULT: {self.fault}' if self.fault else self.mode))
                self.last_status = now
        except Exception as exc:
            self.fault_timing = dict(self.timing, current_work_ms=(time.monotonic()-now)*1000)
            self.disarm()
            self.fault, self.mode = str(exc), 'FAULT'
            self.q = None
            self.status.publish(String(data=f'FAULT: {exc}'))
            self.get_logger().error(str(exc))
        finally:
            self.timing['work_ms'] = (time.monotonic()-now)*1000

    def start_control(self):
        """Run SDK cycles independently of the ROS executor's ready queue."""
        if not self.dedicated_control:
            raise RuntimeError('Cannot run a control worker alongside the ROS control timer')
        if self.control_thread is not None:
            raise RuntimeError('Control worker already started')
        if self.ik_future is not None:
            # Complete spawn/imports before starting the first motion cycle.
            self.ik_future.result(timeout=5)
        self.control_stop.clear()
        self.last_tick = time.monotonic()
        self.control_thread = threading.Thread(target=self.control_loop, name='fr5-control', daemon=True)
        self.control_thread.start()

    def control_loop(self):
        deadline = time.monotonic()
        try:
            while not self.control_stop.is_set():
                self.tick()
                deadline += CONTROL_PERIOD_S
                now = time.monotonic()
                if deadline <= now:
                    # Drop missed slots; do not burst queued ServoJ commands.
                    deadline = now+CONTROL_PERIOD_S
                self.control_stop.wait(max(0.0, deadline-now))
        finally:
            self.disarm()

    def stop_control(self):
        self.control_stop.set()
        if self.control_thread is not None:
            self.control_thread.join()

    def destroy_node(self):
        self.stop_control()
        if self.ik_pool is not None:
            self.ik_pool.shutdown(wait=True, cancel_futures=True)
        if self.home_pool is not None:
            self.home_pool.shutdown(wait=True, cancel_futures=True)
        self.disarm()
        if hasattr(self.rpc, 'CloseRPC'):
            self.rpc.CloseRPC()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    node = None
    try:
        node = RobotNode(dedicated_control=True)
        executor.add_node(node)
        node.start_control()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_control()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
