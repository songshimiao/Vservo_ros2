from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('eih_serial', default_value='CV27864000EE'),
            DeclareLaunchArgument('eth_serial', default_value='CP08863000Z7'),
            DeclareLaunchArgument('eih_intrinsics', default_value=''),
            DeclareLaunchArgument('eth_intrinsics', default_value=''),
            DeclareLaunchArgument('width', default_value='640'),
            DeclareLaunchArgument('height', default_value='480'),
            DeclareLaunchArgument('fps', default_value='30')]
    for role in ('eih', 'eth'):
        args.append(Node(package='vservo_ros2', executable='camera', name='camera', namespace=role,
                         output='screen', parameters=[{'role': role, 'serial': LC(f'{role}_serial'),
                         'intrinsics': LC(f'{role}_intrinsics'),
                         **{k: ParameterValue(LC(k), value_type=int) for k in ('width', 'height', 'fps')}}]))
    return LaunchDescription(args)
