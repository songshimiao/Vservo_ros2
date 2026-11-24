# 单路标定入口：角色命名空间 + 模式/会话/输出 + 板规格，布尔值显式转型后传给节点。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# 声明可覆盖参数并组装 LaunchDescription；此函数生成启动计划，实际节点由 ROS launch 创建。
def generate_launch_description():
    config = get_package_share_directory('vservo_ros2') + '/config/board.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='eih'),
        DeclareLaunchArgument('mode', default_value='intrinsic'),
        DeclareLaunchArgument('session_dir'), DeclareLaunchArgument('output'),
        DeclareLaunchArgument('protect_output', default_value='false'),
        DeclareLaunchArgument('resume', default_value='false'),
        DeclareLaunchArgument('board_config', default_value=config),
        Node(package='vservo_ros2', executable='calibration', name='calibration',
             namespace=LC('role'), output='screen', parameters=[LC('board_config'),
             {**{k: LC(k) for k in ('role', 'mode', 'session_dir', 'output')},
              'protect_output': ParameterValue(LC('protect_output'), value_type=bool),
              'resume': ParameterValue(LC('resume'), value_type=bool)}]),
    ])
