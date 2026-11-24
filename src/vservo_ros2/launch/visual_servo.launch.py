from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def robot_control_parameters(path, overrides):
    # Resolve robot defaults before applying launch overrides. An exact
    # /robot_bridge YAML selector otherwise beats launch's generated /** file.
    config = yaml.safe_load(Path(path).read_text())
    return {**config.get('/**', {}).get('ros__parameters', {}),
            **config.get('/robot_bridge', config.get('robot_bridge', {})).get('ros__parameters', {}),
            **overrides}


def generate_launch_description():
    share = get_package_share_directory('vservo_ros2')
    args = [DeclareLaunchArgument(k) for k in ('eih_intrinsics', 'eth_intrinsics', 'eih_handeye', 'eth_handeye')]
    args += [DeclareLaunchArgument('execute', default_value='false'),
             DeclareLaunchArgument('hardware_ready', default_value='false'),
             DeclareLaunchArgument('speed_scale', default_value='1.0'),
             *[DeclareLaunchArgument(k, default_value='') for k in
               ('linear_speed_m_s', 'angular_speed_rad_s', 'max_joint_accel_rad_s2')],
             DeclareLaunchArgument('auto_start', default_value='false'),
             DeclareLaunchArgument('eih_serial', default_value='CV27864000EE'),
             DeclareLaunchArgument('eth_serial', default_value='CP08863000Z7'),
             DeclareLaunchArgument('robot_ip', default_value='192.168.58.2'),
             DeclareLaunchArgument('sdk_file', default_value='/home/jetson/fairino_ws/fr_visual_servo/scripts/robot/fairino/Robot.py'),
             DeclareLaunchArgument('board_config', default_value=share+'/config/board.yaml'),
             DeclareLaunchArgument('control_config', default_value=share+'/config/control.yaml'),
             DeclareLaunchArgument('eih_plugin', default_value=''),
             DeclareLaunchArgument('eth_plugin', default_value=''),
             DeclareLaunchArgument('aruco_marker_m', default_value='0.080'),
             *[DeclareLaunchArgument(k, default_value=v) for k, v in [('width', '640'), ('height', '480'), ('fps', '30')]],
             DeclareLaunchArgument('target_id', default_value='charuco_board')]
    args.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/dual_camera.launch.py'),
                launch_arguments={k: LC(k) for k in ('eih_intrinsics', 'eth_intrinsics', 'eih_serial', 'eth_serial',
                                                    'width', 'height', 'fps')}.items()))
    def robot_action(context):
        overrides = {key: float(LC(key).perform(context)) for key in
                     ('linear_speed_m_s', 'angular_speed_rad_s', 'max_joint_accel_rad_s2')
                     if LC(key).perform(context)}
        return [Node(package='vservo_ros2', executable='robot_bridge', name='robot_bridge', output='screen',
                     additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
                     parameters=[robot_control_parameters(LC('control_config').perform(context), overrides),
                                 {**{k: LC(k) for k in ('eih_handeye', 'eth_handeye', 'eih_serial', 'eth_serial', 'robot_ip', 'sdk_file')},
                     'speed_scale': ParameterValue(LC('speed_scale'), value_type=float),
                     **{k: ParameterValue(LC(k), value_type=bool) for k in ('execute', 'hardware_ready', 'auto_start')}}])]
    args.append(OpaqueFunction(function=robot_action))
    for role in ('eih', 'eth'):
        args.append(Node(package='vservo_ros2', executable='detector', namespace=role, name='detector',
                         parameters=[LC('board_config'), {'role': role, 'plugin': LC(f'{role}_plugin'), 'target_id': LC('target_id'),
                         'aruco_marker_m': ParameterValue(LC('aruco_marker_m'), value_type=float)}]))
    args.append(Node(package='vservo_ros2', executable='localizer', name='localizer', parameters=[LC('control_config')]))
    args.append(Node(package='vservo_ros2', executable='servo', name='servo',
                     parameters=[LC('control_config'), {'eih_handeye': LC('eih_handeye')}]))
    return LaunchDescription(args)
