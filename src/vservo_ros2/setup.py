from glob import glob
from setuptools import find_packages, setup

setup(
    name='vservo_ros2', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/vservo_ros2']),
                ('share/vservo_ros2', ['package.xml']),
                ('share/vservo_ros2/launch', glob('launch/*.py')),
                ('share/vservo_ros2/config', glob('config/*.yaml')),
                ('share/vservo_ros2/web', glob('vservo_ros2/web/static/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Jetson Robotics', maintainer_email='robotics@example.invalid',
    description='FR5 dual camera calibration and coarse-to-fine visual servo',
    license='Proprietary', tests_require=['pytest'],
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
