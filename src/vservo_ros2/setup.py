# ROS Python 包安装描述：注册资源索引，安装 launch/config/Web 文件，生成 ros2 run 可用的节点入口。
from glob import glob
from setuptools import find_packages, setup

setup(
    name='vservo_ros2', version='0.1.0', packages=find_packages(),
    # 数据文件装入 share/vservo_ros2；运行时通过 ament 索引定位，不依赖启动所在目录。
    data_files=[('share/ament_index/resource_index/packages', ['resource/vservo_ros2']),
                ('share/vservo_ros2', ['package.xml']),
                ('share/vservo_ros2/launch', glob('launch/*.py')),
                ('share/vservo_ros2/config', glob('config/*.yaml')),
                ('share/vservo_ros2/web', glob('vservo_ros2/web/static/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Jetson Robotics', maintainer_email='robotics@example.invalid',
    description='FR5 dual camera calibration and coarse-to-fine visual servo',
    license='Proprietary', tests_require=['pytest'],
    # 左侧是 ros2 run 的可执行名，右侧是 Python 模块:函数；据此可从命令追踪到源码。
    entry_points={'console_scripts': [
        'web_console = vservo_ros2.web.server:main',
        'camera = vservo_ros2.camera_node:main',
        'calibration = vservo_ros2.calibration_node:main',
        'detector = vservo_ros2.detector_node:main',
        'localizer = vservo_ros2.localizer_node:main',
        'robot_bridge = vservo_ros2.robot_node:main',
        'servo = vservo_ros2.servo_node:main',
        'tools = vservo_ros2.cli:main',
        'accuracy_recorder = vservo_ros2.accuracy_node:main',
    ]},
)
