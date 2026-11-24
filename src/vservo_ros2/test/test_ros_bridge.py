"""Exercise the ROS bridge against a fake RPC; zero physical motion."""
import time
import threading
import json
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import SetBool
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from vservo_ros2.geometry import transform, inverse, write_json, fill_pose, error, tf_matrix
from vservo_ros2.robot_node import RobotNode
from vservo_ros2.web.ros_link import RosLink
from vservo_ros2.localizer_node import LocalizerNode
from vservo_ros2.servo_node import ServoNode


@pytest.fixture
def fake_sdk(tmp_path):
    sdk = tmp_path / 'Robot.py'
    sdk.write_text('''
class RPC:
    def __init__(self, ip):
        self.calls = []
        self.positions = [20,-90,75,-60,-80,15]
    def GetActualJointPosDegree(self): return 0, list(self.positions)
    def ServoMoveStart(self): self.calls.append('start'); return 0
    def ServoMoveEnd(self): self.calls.append('end'); return 0
    def ServoJ(self, joint_pos, axisPos, acc, vel, cmdT, filterT, gain):
        assert axisPos == [0.0]*4 and cmdT == 0.01
        self.positions = list(joint_pos)
        self.calls.append('servo'); return 0
    def CloseRPC(self): self.calls.append('close')
''')
    paths = {}
    for role, serial in [('eih', 'CV27864000EE'), ('eth', 'CP08863000Z7')]:
        path = tmp_path / (role+'.json')
        write_json(path, dict(mode=role, units='m', validated=True, matrix=np.eye(4).tolist(),
                             parent_frame='flange' if role == 'eih' else 'base',
                             child_frame=f'{role}_optical_frame', camera={'serial': serial}))
        paths[role] = path
    return sdk, paths


def make_node(fake_sdk, execute, auto_start=False, dedicated_control=False):
    sdk, paths = fake_sdk
    args = ['--ros-args', '-p', f'sdk_file:={sdk}', '-p', f'execute:={str(execute).lower()}',
            '-p', 'hardware_ready:=true', '-p', f'auto_start:={str(auto_start).lower()}',
            '-p', 'workspace_min:=[-2.0,-2.0,-2.0]',
            '-p', 'workspace_max:=[2.0,2.0,2.0]']
    for role, path in paths.items():
        args += ['-p', f'{role}_handeye:={path}']
    rclpy.init(args=args)
    node = RobotNode(dedicated_control=dedicated_control)
    node.on_localization(String(data='COARSE'))
    return node


def test_readonly_never_starts_or_sends_motion(fake_sdk):
    node = make_node(fake_sdk, False)
    try:
        with pytest.raises(RuntimeError, match='alongside the ROS control timer'):
            node.start_control()
        node.tick()
        assert node.q is not None
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = 'base', node.get_clock().now().to_msg()
        fill_pose(msg.pose, node.model.fk(node.q))
        node.on_command(msg)
        assert node.command is not None
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert not response.success
        node.tick()
        assert node.rpc.calls == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_late_tf_listener_receives_both_handeye_transforms(fake_sdk):
    robot = make_node(fake_sdk, False)
    probe = Node('late_handeye_listener')
    buffer = Buffer()
    listener = TransformListener(buffer, probe)
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(probe)
    try:
        deadline = time.monotonic()+3
        pairs = [('flange', 'eih_optical_frame'), ('base', 'eth_optical_frame')]
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if all(buffer.can_transform(parent, child, Time()) for parent, child in pairs):
                break
        for parent, child in pairs:
            assert buffer.can_transform(parent, child, Time()), buffer.all_frames_as_yaml()
            assert np.allclose(tf_matrix(buffer.lookup_transform(parent, child, Time())), np.eye(4))
        assert robot.rpc.calls == []
    finally:
        executor.shutdown()
        probe.destroy_node()
        robot.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('role', ['eih', 'eth'])
def test_either_camera_alone_can_start_sdk_servo(fake_sdk, role):
    robot = make_node(fake_sdk, True)
    robot.rpc.positions = np.rad2deg(robot.home+0.02).tolist()
    localizer, servo, fixture = LocalizerNode(), ServoNode(), Node('single_camera_start_fixture')
    publisher = fixture.create_publisher(PoseWithCovarianceStamped, f'/{role}/target_pose', 1)
    executor = SingleThreadedExecutor()
    nodes = [robot, localizer, servo, fixture]
    for node in nodes: executor.add_node(node)
    expected = 'FINE' if role == 'eih' else 'COARSE'
    try:
        deadline, last_publish = time.monotonic()+5, 0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            if robot.q is not None and time.monotonic()-last_publish > 0.04:
                current = robot.model.fk(robot.q)
                board = current @ transform(t=[0, 0, 0.30])
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = f'{role}_optical_frame'
                msg.header.stamp = fixture.get_clock().now().to_msg()
                fill_pose(msg.pose.pose, inverse(current) @ board if role == 'eih' else board)
                publisher.publish(msg)
                last_publish = time.monotonic()
            if robot.localization == expected and robot.command is not None:
                break
        assert robot.localization == expected and robot.command is not None, localizer.selector.diagnostics
        assert localizer.selector.diagnostics['source'] == role
        assert robot.rpc.calls == []
        assert robot.arm(SetBool.Request(data=True), SetBool.Response()).success
        robot.last_tick = time.monotonic()
        robot.tick()
        assert robot.mode == ('FOLLOWING_EIH' if role == 'eih' else 'FOLLOWING_ETH')
        assert robot.armed and robot.follow_requested
        assert robot.rpc.calls.count('start') == 1 and 'servo' in robot.rpc.calls
        assert not robot.returning_home and robot.home_future is None
    finally:
        executor.shutdown()
        for node in reversed(nodes): node.destroy_node()
        rclpy.shutdown()


def test_follow_waits_on_stale_command_without_exiting_session(fake_sdk):
    node = make_node(fake_sdk, True)
    try:
        node.tick()
        node.command = (node.get_clock().now().to_msg(), time.monotonic(), node.q.copy())
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert response.success
        node.last_tick = time.monotonic()
        node.tick()
        assert 'servo' in node.rpc.calls
        before = node.rpc.calls.count('servo')
        node.command = (node.get_clock().now().to_msg(), time.monotonic()-1, node.q.copy())
        node.last_tick = time.monotonic()
        node.tick()
        assert node.armed and node.follow_requested
        assert node.mode == 'WAITING_COMMAND'
        assert node.rpc.calls.count('servo') == before+1
        assert 'end' not in node.rpc.calls
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_speed_parameter_rejects_invalid_values_and_scales_motion(fake_sdk):
    node = make_node(fake_sdk, True)
    try:
        node.tick()
        assert node.set_parameters_atomically([Parameter('speed_scale', value=0.2)]).successful
        for value in (float('nan'), float('inf'), 0.0, 1.1, True):
            assert not node.set_parameters_atomically([Parameter('speed_scale', value=value)]).successful
        assert node.get_parameter('speed_scale').value == 0.2
        node.command = (node.get_clock().now().to_msg(), time.monotonic(), node.q+0.01)
        assert node.arm(SetBool.Request(data=True), SetBool.Response()).success
        node.last_tick = time.monotonic()
        node.tick()
        assert node.limiter.max_speed == pytest.approx(0.40)
        assert node.set_parameters_atomically([Parameter('speed_scale', value=0.5)]).successful
        node.last_tick = time.monotonic()
        node.tick()
        assert node.limiter.max_speed == pytest.approx(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_web_ros_speed_service_confirms_applied_value(fake_sdk):
    robot = make_node(fake_sdk, False)
    link = RosLink(1)
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(link)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic()+5
        while not link.speed_client.service_is_ready() and time.monotonic() < deadline:
            time.sleep(0.02)
        result = link.set_speed(30.0)
        assert result['success'], result
        assert robot.get_parameter('speed_scale').value == 0.3
        while time.monotonic() < deadline:
            with link.lock:
                feedback = link.values.get('speed', (None, {}))[1]
            if feedback.get('percent') == 30:
                break
            time.sleep(0.02)
        assert feedback['percent'] == 30
        assert feedback['max_joint_speed_rad_s'] == pytest.approx(0.6)
        profile = dict(linear_mm_s=150.0, angular_deg_s=45.0, joint_accel_rad_s2=0.8)
        assert link.set_speed(80.0, profile)['success']
        assert robot.get_parameter('linear_speed_m_s').value == 0.15
        assert robot.get_parameter('angular_speed_rad_s').value == pytest.approx(np.deg2rad(45))
        assert robot.get_parameter('max_joint_accel_rad_s2').value == 0.8
        assert not link.set_speed(50.0, dict(profile, linear_mm_s=201))['success']
        assert robot.get_parameter('speed_scale').value == 0.8
        assert robot.get_parameter('linear_speed_m_s').value == 0.15
        assert robot.rpc.calls == []
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        link.close()
        link.destroy_node()
        robot.destroy_node()
        rclpy.shutdown()


def test_full_target_drives_faster_and_runtime_controls_change_actual_motion(fake_sdk):
    """Compare generated SDK positions, rather than just parameter feedback."""
    node = make_node(fake_sdk, True, auto_start=True)
    results = {}
    try:
        initial = np.deg2rad([20, -90, 75, -60, -80, 15])
        destination = node.model.fk(initial+np.array([0.9, 0, 0, 0, 0, 0]))
        for name, full, scale, linear in [('legacy', False, 1.0, 0.1), ('full', True, 1.0, 0.1),
                                          ('faster', True, 1.0, 0.2), ('slow', True, 0.25, 0.1)]:
            node.disarm()
            node.desired_seen = False
            node.rpc.positions = np.rad2deg(initial).tolist()
            node.tick()
            node.set_parameters_atomically([Parameter('speed_scale', value=scale),
                                            Parameter('linear_speed_m_s', value=linear),
                                            Parameter('angular_speed_rad_s', value=float(np.deg2rad(60)))])
            assert node.arm(SetBool.Request(data=True), SetBool.Response()).success
            velocities, positions = [], []
            previous = initial.copy()
            for i in range(250):
                node.on_localization(String(data='COARSE'))
                if i % 2 == 0:
                    target = destination
                    if not full:
                        from vservo_ros2.control import step
                        target = step(node.model.fk(node.q), destination)
                    msg = PoseStamped()
                    msg.header.frame_id, msg.header.stamp = 'base', node.get_clock().now().to_msg()
                    fill_pose(msg.pose, target)
                    (node.on_desired if full else node.on_command)(msg)
                    assert node.command is not None, node.command_error
                node.last_tick = time.monotonic()
                node.tick()
                assert node.fault is None, node.fault
                q = np.deg2rad(node.rpc.positions)
                velocities.append((q-previous)/0.01)
                positions.append(node.model.fk(q)[:3, 3])
                previous = q
            velocities = np.array(velocities)
            assert np.max(np.abs(velocities)) <= 2*scale+1e-8
            assert np.max(np.abs(np.diff(velocities, axis=0)))/0.01 <= 0.5+1e-7
            results[name] = (np.linalg.norm(previous-initial),
                             np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1))/0.01)
        assert results['full'][0] > results['legacy'][0]*1.5, results
        assert results['faster'][0] > results['full'][0]*1.3, results
        assert results['full'][0] > results['slow'][0]*2, results
        assert results['full'][1] == pytest.approx(0.1, rel=0.03), results
        print('actual SDK trajectory speed comparison:', results)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_auto_follow_returns_home_once_and_resumes_without_manual_rearm(fake_sdk):
    node = make_node(fake_sdk, True, auto_start=True)
    try:
        node.on_localization(String(data='HOLD'))
        node.loss_since = time.monotonic()-1
        modes = []
        for _ in range(1500):
            node.on_localization(String(data='HOLD'))
            node.last_tick = time.monotonic()
            node.tick()
            modes.append(node.mode)
            assert node.fault is None, node.fault
            assert node.follow_requested
            if node.home_complete:
                break
        assert 'RETURNING_HOME' in modes and node.home_complete
        assert np.allclose(np.rad2deg(node.q), [90, -110, 110, -180, -90, 0], atol=0.3)
        assert node.rpc.calls.count('start') == 1 and 'end' not in node.rpc.calls
        node.on_localization(String(data='COARSE'))
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = 'base', node.get_clock().now().to_msg()
        fill_pose(msg.pose, node.model.fk(node.q))
        node.on_command(msg)
        node.last_tick = time.monotonic()
        node.tick()
        assert node.mode == 'FOLLOWING_ETH' and node.follow_requested and node.armed
        response = node.arm(SetBool.Request(data=False), SetBool.Response())
        assert response.success and not node.follow_requested and not node.armed
        count = node.rpc.calls.count('servo')
        node.tick()
        assert node.rpc.calls.count('servo') == count
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_live_angular_acceleration_and_override_change_sdk_trajectory(fake_sdk):
    node = make_node(fake_sdk, True, auto_start=True)
    try:
        node.tick()
        initial = node.q.copy()
        target = node.model.fk(initial+np.array([1.5, 0, 0, 0, 0, 0]))
        assert node.set_parameters_atomically([
            Parameter('linear_speed_m_s', value=0.2),
            Parameter('angular_speed_rad_s', value=float(np.deg2rad(5)))]).successful
        speeds, accelerations = [], []
        previous_q, previous_v = initial, np.zeros(6)
        for i in range(300):
            if i == 100:
                assert node.armed
                assert node.set_parameters_atomically([
                    Parameter('angular_speed_rad_s', value=float(np.deg2rad(60))),
                    Parameter('max_joint_accel_rad_s2', value=1.0)]).successful
            if i == 200:
                assert node.set_parameters_atomically([Parameter('speed_scale', value=0.1)]).successful
            node.on_localization(String(data='COARSE'))
            msg = PoseStamped()
            msg.header.frame_id, msg.header.stamp = 'base', node.get_clock().now().to_msg()
            fill_pose(msg.pose, target)
            node.on_desired(msg)
            assert node.command is not None, node.command_error
            node.last_tick = time.monotonic()
            node.tick()
            assert node.fault is None, node.fault
            q = np.deg2rad(node.rpc.positions)
            velocity = (q-previous_q)/0.01
            speeds.append(np.array(error(node.model.fk(previous_q), node.model.fk(q)))/0.01)
            accelerations.append(np.max(np.abs(velocity-previous_v))/0.01)
            previous_q, previous_v = q, velocity
        speeds = np.array(speeds)
        assert np.degrees(speeds[90:100, 1]).mean() == pytest.approx(5, abs=0.1)
        assert speeds[190:200, 0].mean() == pytest.approx(0.2, rel=0.03)
        assert speeds[290:300, 0].mean() == pytest.approx(0.02, rel=0.03)
        assert max(accelerations[:100]) <= 0.5+1e-7
        assert max(accelerations[100:]) <= 1+1e-7
        assert max(accelerations[100:200]) > 0.9
        assert node.rpc.calls.count('start') == 1 and 'end' not in node.rpc.calls
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_controller_error_stops_session_instead_of_retrying_motion(fake_sdk):
    node = make_node(fake_sdk, True, auto_start=True)
    try:
        node.rpc.GetActualJointPosDegree = lambda: (1, [])
        node.tick()
        assert node.mode == 'FAULT' and not node.follow_requested
        assert 'start' not in node.rpc.calls and 'servo' not in node.rpc.calls
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_pipeline_component(kind, handeye, ready, stopped):
    rclpy.init(args=['--ros-args', '-p', f'eih_handeye:={handeye}',
                    '-p', 'workspace_min:=[-2.0,-2.0,-2.0]', '-p', 'workspace_max:=[2.0,2.0,2.0]'])
    node = LocalizerNode() if kind == 'localizer' else ServoNode()
    try:
        ready.set()
        while not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('first', ['eih', 'eth'])
def test_timed_ros_pipeline_tracks_either_camera_hands_off_and_only_homes_on_both_loss(fake_sdk, first):
    import multiprocessing
    robot = make_node(fake_sdk, True, auto_start=True, dedicated_control=True)
    robot.rpc.positions = np.rad2deg(robot.home+0.02).tolist()
    fixture = Node('persistent_follow_fixture')
    publishers = {role: fixture.create_publisher(PoseWithCovarianceStamped, f'/{role}/target_pose', 1)
                  for role in ('eih', 'eth')}
    visibility = {role: fixture.create_publisher(String, f'/{role}/target_visibility', 1)
                  for role in ('eih', 'eth')}
    def lose_role(role):
        visibility[role].publish(String(data=json.dumps(dict(
            frame_id=f'{role}_optical_frame', visible=False,
            stamp_ns=fixture.get_clock().now().nanoseconds, reason='Occluded in test'))))
    second = 'eth' if first == 'eih' else 'eih'
    board = robot.model.fk(robot.home+np.array([0.04, 0, 0, 0, 0, 0])) @ transform(t=[0, 0, 0.30])
    def publish_roles(roles):
        current = robot.model.fk(robot.q)
        for role in roles:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = f'{role}_optical_frame'
            msg.header.stamp = fixture.get_clock().now().to_msg()
            fill_pose(msg.pose.pose, inverse(current) @ board if role == 'eih' else board)
            publishers[role].publish(msg)
    # Use the production process boundaries. A combined test process creates
    # additional GIL contention between independent ROS nodes that do not share
    # an interpreter on the robot.
    context = multiprocessing.get_context('spawn')
    stopped = context.Event()
    children, ready_events = [], []
    for kind in ('localizer', 'servo'):
        ready = context.Event()
        child = context.Process(target=run_pipeline_component,
                                args=(kind, str(fake_sdk[1]['eih']), ready, stopped))
        child.start()
        children.append(child)
        ready_events.append(ready)
    solve = robot.solve_ik
    def slow_ik(target, seed):
        time.sleep(0.06)
        return solve(target, seed)
    robot.solve_ik = slow_ik
    executor = MultiThreadedExecutor(num_threads=3)
    nodes = [robot, fixture]
    for node in nodes:
        executor.add_node(node)
    observed_home = False
    observed_follow = False
    try:
        assert all(ready.wait(6) for ready in ready_events)
        robot.ik_future.result(timeout=6)
        robot.localization = robot.localization_received = None
        robot.start_control()
        deadline, last_publish = time.monotonic()+8, 0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            assert robot.fault is None, robot.fault
            assert robot.follow_requested
            observed_home |= robot.home_complete
            if observed_home and time.monotonic()-last_publish > 0.04:
                publish_roles([first])
                last_publish = time.monotonic()
            if robot.mode == f'FOLLOWING_{first.upper()}' and robot.q[0]-robot.home[0] > 0.02:
                observed_follow = True
                break
        assert observed_home and observed_follow
        # The later camera must not steal control while the first stays valid.
        deadline, last_publish = time.monotonic()+0.4, 0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            if time.monotonic()-last_publish > 0.04:
                publish_roles([first, second])
                last_publish = time.monotonic()
            assert robot.localization == ('FINE' if first == 'eih' else 'COARSE')
            assert not robot.returning_home and robot.home_future is None
        # Lose only the active camera: the remaining camera takes over directly.
        lose_role(first)
        deadline, last_publish, handed_off = time.monotonic()+2, 0, False
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            if time.monotonic()-last_publish > 0.04:
                publish_roles([second])
                last_publish = time.monotonic()
            assert robot.fault is None, robot.fault
            assert robot.localization != 'HOLD'
            assert not robot.returning_home and robot.home_future is None
            if robot.mode == f'FOLLOWING_{second.upper()}':
                handed_off = True
                break
        assert handed_off
        lose_role(second)
        # Stop publishing observations: both cameras lose the target while the
        # SDK is already active, then prepare/reset without ending the session.
        modes = []
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            assert robot.fault is None, robot.fault
            modes.append(robot.mode)
            if robot.home_complete:
                break
        assert 'PREPARING_HOME' in modes and 'RETURNING_HOME' in modes
        assert robot.home_complete and np.allclose(robot.q, robot.home, atol=np.deg2rad(0.3))
        assert robot.rpc.calls.count('start') == 1 and 'end' not in robot.rpc.calls
        assert robot.arm(SetBool.Request(data=False), SetBool.Response()).success
        count = robot.rpc.calls.count('servo')
        deadline = time.monotonic()+0.1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
        assert robot.rpc.calls.count('servo') == count
    finally:
        robot.stop_control()
        stopped.set()
        for child in children:
            child.join(timeout=3)
            if child.is_alive():
                child.terminate()
                child.join(timeout=2)
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


def test_waiting_for_visible_target_tf_never_starts_home_and_resets_loss_timer(fake_sdk):
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.on_localization(String(data='HOLD'))
        robot.loss_since = time.monotonic()-1
        robot.on_localization(String(data='WAITING_TF'))
        robot.tick()
        assert robot.mode == 'WAITING_COMMAND' and robot.loss_since is None
        assert robot.follow_requested and not robot.returning_home and robot.home_future is None
        assert robot.rpc.calls == []
        robot.on_localization(String(data='HOLD'))
        robot.tick()
        assert robot.mode == 'WAITING_TARGET'
        assert 0 <= time.monotonic()-robot.loss_since < 0.1
        assert robot.home_future is None and robot.rpc.calls == []
    finally:
        robot.destroy_node()
        rclpy.shutdown()


def test_full_target_rejects_invalid_pose_and_discards_ik_across_camera_switch(fake_sdk):
    node = make_node(fake_sdk, False)
    try:
        node.tick()
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = 'base', node.get_clock().now().to_msg()
        fill_pose(msg.pose, node.model.fk(node.q))
        node.on_desired(msg)
        assert node.command is not None and node.desired_seen
        fill_pose(msg.pose, transform(t=[3, 0, 0]))
        node.on_desired(msg)
        assert node.command is None
        fill_pose(msg.pose, node.model.fk(node.q))
        node.on_command(msg)
        assert node.command is None  # Legacy stream cannot replace a failed full target.
        solve = node.model.ik
        def switch_camera(target, seed):
            goal = solve(target, seed)
            node.on_localization(String(data='FINE'))
            return goal
        node.model.ik = switch_camera
        node.on_desired(msg)
        assert node.command is None and node.last_ik_goal is None
        assert node.rpc.calls == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_heartbeat_is_received_during_slow_sdk_enable(fake_sdk):
    robot = make_node(fake_sdk, True)
    robot.rpc.positions = np.rad2deg(robot.home).tolist()
    fixture = Node('heartbeat_during_enable_fixture')
    publisher = fixture.create_publisher(String, '/localization/state', 1)
    fixture.create_timer(0.05, lambda: publisher.publish(String(data='HOLD')))
    finished = threading.Event()
    received_during_enable = []
    enable = robot.rpc.ServoMoveStart
    def slow_enable():
        before = robot.localization_received
        time.sleep(0.65)
        received_during_enable.append(robot.localization_received-before)
        result = enable()
        finished.set()
        return result
    robot.rpc.ServoMoveStart = slow_enable
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(robot)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic()+5
        while robot.localization != 'HOLD' and time.monotonic() < deadline:
            time.sleep(0.01)
        assert robot.localization == 'HOLD'
        assert robot.arm(SetBool.Request(data=True), SetBool.Response()).success
        assert finished.wait(4)
        assert received_during_enable[0] > 0.4, received_during_enable
        time.sleep(0.15)
        assert robot.fault is None, robot.fault
        assert robot.follow_requested and robot.armed
        assert robot.rpc.calls.count('start') == 1 and 'end' not in robot.rpc.calls
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        fixture.destroy_node()
        robot.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('during_enable', [False, True])
def test_real_heartbeat_loss_still_faults_before_more_position_commands(fake_sdk, during_enable):
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.tick()
        robot.command = (robot.get_clock().now().to_msg(), time.monotonic(), robot.q.copy())
        if during_enable:
            enable = robot.rpc.ServoMoveStart
            def lose_heartbeat():
                robot.localization_received = time.monotonic()-0.6
                return enable()
            robot.rpc.ServoMoveStart = lose_heartbeat
        else:
            robot.last_tick = time.monotonic()
            robot.tick()
            assert robot.armed
            robot.localization_received = time.monotonic()-0.6
        before = robot.rpc.calls.count('servo')
        robot.last_tick = time.monotonic()
        robot.tick()
        assert robot.mode == 'FAULT' and not robot.follow_requested and not robot.armed
        assert robot.fault.startswith('Localization heartbeat expired (age=')
        assert robot.rpc.calls.count('servo') == before
        assert robot.rpc.calls[-1] == 'end'
    finally:
        robot.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('overrides', [{}, dict(linear_speed_m_s=0.003, angular_speed_rad_s=0.1,
                                              max_joint_accel_rad_s2=0.8)])
def test_launch_motion_profile_reaches_ros_parameter_values(fake_sdk, tmp_path, overrides):
    from pathlib import Path
    import runpy
    import yaml
    package = Path(__file__).resolve().parents[1]
    launch = runpy.run_path(str(package/'launch/visual_servo.launch.py'))
    parameters = launch['robot_control_parameters'](package/'config/control.yaml', overrides)
    parameters['sdk_file'] = str(fake_sdk[0])
    path = tmp_path/'launch_robot_parameters.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    rclpy.init(args=['--ros-args', '--params-file', str(path)])
    robot = RobotNode()
    try:
        assert robot.get_parameter('linear_speed_m_s').value == overrides.get('linear_speed_m_s', 0.1)
        assert robot.get_parameter('angular_speed_rad_s').value == overrides.get('angular_speed_rad_s', np.deg2rad(30))
        assert robot.get_parameter('max_joint_accel_rad_s2').value == overrides.get('max_joint_accel_rad_s2', 0.5)
        assert robot.get_parameter('max_joint_speed_rad_s').value == 2.0
        assert robot.minimum == [-0.6, -0.6, 0.1]
        assert robot.rpc.calls == []
    finally:
        robot.destroy_node()
        rclpy.shutdown()


def test_dedicated_control_keeps_sending_during_blocked_ros_executor(fake_sdk):
    robot = make_node(fake_sdk, True, auto_start=True, dedicated_control=True)
    robot.rpc.positions = np.rad2deg(robot.home).tolist()
    fixture = Node('blocked_ros_fixture')
    publisher = fixture.create_publisher(String, '/localization/state', 1)
    fixture.create_timer(0.05, lambda: publisher.publish(String(data='HOLD')))
    blocked_while_armed = []
    def block_callback():
        blocked_while_armed.append(robot.armed)
        time.sleep(0.08)
    fixture.create_timer(0.25, block_callback)
    sent = []
    send = robot.rpc.ServoJ
    def record_send(*args, **kwargs):
        sent.append(time.monotonic())
        return send(*args, **kwargs)
    robot.rpc.ServoJ = record_send
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(fixture)
    robot.localization = robot.localization_received = None
    robot.start_control()
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            assert robot.fault is None, robot.fault
        assert any(blocked_while_armed)
        assert len(sent) > 80
        assert max(np.diff(sent)) < 0.05
        assert robot.arm(SetBool.Request(data=False), SetBool.Response()).success
        count = len(sent)
        time.sleep(0.05)
        assert len(sent) == count
    finally:
        executor.shutdown()
        fixture.destroy_node()
        robot.destroy_node()
        assert not robot.control_thread.is_alive()
        rclpy.shutdown()


def test_slow_servoj_still_trips_watchdog_with_latency_details(fake_sdk):
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.tick()
        robot.command = (robot.get_clock().now().to_msg(), time.monotonic(), robot.q.copy())
        send = robot.rpc.ServoJ
        def slow_send(*args, **kwargs):
            time.sleep(0.07)
            return send(*args, **kwargs)
        robot.rpc.ServoJ = slow_send
        robot.last_tick = time.monotonic()
        robot.tick()
        before = robot.rpc.calls.count('servo')
        assert robot.fault.startswith('Control watchdog expired (command_gap=')
        assert not robot.armed and not robot.follow_requested
        assert robot.fault_timing['servoj_ms'] >= 65
        assert robot.fault_timing['command_gap_ms'] >= 65
        robot.tick()
        assert robot.rpc.calls.count('servo') == before
        assert robot.rpc.calls[-1] == 'end'
    finally:
        robot.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('command_missing', [False, True])
def test_watchdog_checks_command_gap_instead_of_callback_start_interval(fake_sdk, command_missing):
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.tick()
        robot.command = (robot.get_clock().now().to_msg(), time.monotonic(), robot.q.copy())
        robot.last_tick = time.monotonic()
        robot.tick()
        assert robot.armed
        before = robot.rpc.calls.count('servo')
        # A late callback-start interval alone is not a missing output command.
        robot.last_tick = time.monotonic()-(0.01 if command_missing else 0.07)
        robot.last_servo_sent = time.monotonic()-(0.07 if command_missing else 0.001)
        robot.tick()
        if command_missing:
            assert robot.fault.startswith('Control watchdog expired (command_gap=')
            assert robot.rpc.calls.count('servo') == before
            assert not robot.armed and not robot.follow_requested
        else:
            assert robot.fault is None and robot.armed
            assert robot.rpc.calls.count('servo') == before+1
    finally:
        robot.destroy_node()
        rclpy.shutdown()


def test_ik_worker_timeout_does_not_queue_repeated_stale_jobs(fake_sdk):
    from concurrent.futures import Future
    robot = make_node(fake_sdk, False)
    class BusyPool:
        calls = 0
        def submit(self, *args):
            self.calls += 1
            future = Future()
            future.set_running_or_notify_cancel()
            return future
    pool = BusyPool()
    robot.ik_pool = pool
    try:
        with pytest.raises(ValueError, match='200 ms'):
            robot.solve_ik(np.eye(4), np.zeros(6))
        with pytest.raises(ValueError, match='busy'):
            robot.solve_ik(np.eye(4), np.zeros(6))
        assert pool.calls == 1
    finally:
        robot.ik_pool = None
        robot.destroy_node()
        rclpy.shutdown()


def test_slow_reset_preflight_keeps_armed_servo_stream_and_completes_home(fake_sdk):
    robot = make_node(fake_sdk, True, auto_start=True)
    sent, planning_counts = [], []
    robot.rpc.positions = np.rad2deg(robot.home+0.02).tolist()
    send = robot.rpc.ServoJ
    def realistic_send(*args, **kwargs):
        time.sleep(0.009)
        sent.append(time.monotonic())
        return send(*args, **kwargs)
    def slow_check(*args):
        before = len(sent)
        time.sleep(0.08)
        planning_counts.append(len(sent)-before)
        return True
    robot.rpc.ServoJ = realistic_send
    robot.home_path_checker = slow_check
    try:
        robot.tick()
        robot.command = (robot.get_clock().now().to_msg(), time.monotonic(), robot.q.copy())
        robot.tick()
        assert robot.armed
        robot.on_localization(String(data='HOLD'))
        robot.loss_since = time.monotonic()-1
        modes = []
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            robot.on_localization(String(data='HOLD'))
            robot.tick()
            assert robot.fault is None, robot.fault
            modes.append(robot.mode)
            if robot.home_complete:
                break
            time.sleep(0.002)
        assert planning_counts and planning_counts[0] >= 3
        assert 'PREPARING_HOME' in modes and 'RETURNING_HOME' in modes
        assert robot.home_complete and robot.follow_requested and robot.armed
        assert np.max(np.diff(sent)) < 0.05
        assert robot.rpc.calls.count('start') == 1 and 'end' not in robot.rpc.calls
    finally:
        robot.destroy_node()
        rclpy.shutdown()


def test_invalid_reset_path_is_rejected_before_enabling_motion(fake_sdk):
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.minimum, robot.maximum = [0.6]*3, [0.7]*3
        robot.on_localization(String(data='HOLD'))
        robot.loss_since = time.monotonic()-1
        deadline = time.monotonic()+1
        while time.monotonic() < deadline and robot.fault is None:
            robot.on_localization(String(data='HOLD'))
            robot.tick()
            time.sleep(0.001)
        assert robot.fault and 'Reset joint path' in robot.fault
        assert not robot.follow_requested and not robot.armed
        assert 'start' not in robot.rpc.calls and 'servo' not in robot.rpc.calls
    finally:
        robot.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('manual_stop', [False, True])
def test_pending_reset_plan_is_discarded_after_stop_or_reacquisition(fake_sdk, manual_stop):
    from concurrent.futures import Future
    robot = make_node(fake_sdk, True, auto_start=True)
    try:
        robot.tick()
        pending = Future()
        pending.set_running_or_notify_cancel()
        robot.home_future, robot.home_anchor = pending, robot.q.copy()
        if manual_stop:
            assert robot.arm(SetBool.Request(data=False), SetBool.Response()).success
        else:
            robot.on_localization(String(data='COARSE'))
            assert robot.follow_goal(time.monotonic())[0] == 'WAITING_COMMAND'
        assert robot.home_anchor is None and not robot.returning_home
        pending.set_result(True)
        robot.tick()
        assert not robot.returning_home and not robot.armed
        assert 'start' not in robot.rpc.calls and 'servo' not in robot.rpc.calls
    finally:
        robot.destroy_node()
        rclpy.shutdown()
