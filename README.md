# FR5 双相机视觉伺服 ROS 2 工程

基于同级 `fr_visual_servo`，运行于 Jetson Ubuntu 22.04 / ROS 2 Humble / Python 3.10。
Gemini 335 为 ETH（固定外部相机），负责大范围搜索与粗定位；Gemini 305 为 EIH
（法兰相机），在近距离接管精定位。标定使用 ChArUco，Web 伺服监控默认使用单个 ArUco ID 42，预留 YOLO + 米制 6D 位姿插件。

工程包含可运行的 ROS 2 节点、采集/求解服务、独立验证脚本及默认只读的 FR5 桥接。
新安装的相机必须现场标定；本工程不附带虚构的手眼矩阵或精度结论。

## 源码阅读顺序

源码已按模块、类和函数补充中文注释，重点解释输入输出、坐标系、单位、异步任务和失败分支。
建议先读算法再读 ROS 封装，最后读 Web；现场距离和速度以
[`control.yaml`](src/vservo_ros2/config/control.yaml) 及节点实际参数为准。

| 顺序 | 文件（相对 `src/vservo_ros2/vservo_ros2/`） | 阅读重点 |
|------|-----------------------------------------|----------|
| 1 | [`geometry.py`](src/vservo_ros2/vservo_ros2/geometry.py)、[`kinematics.py`](src/vservo_ros2/vservo_ros2/kinematics.py) | `A_T_B` 方向、米/弧度、FK 与 IK |
| 2 | [`board.py`](src/vservo_ros2/vservo_ros2/board.py)、[`calibration.py`](src/vservo_ros2/vservo_ros2/calibration.py)、[`accuracy.py`](src/vservo_ros2/vservo_ros2/accuracy.py) | 检测门槛、两种手眼公式、重复性与独立参考 |
| 3 | [`camera_node.py`](src/vservo_ros2/vservo_ros2/camera_node.py)、[`detector_node.py`](src/vservo_ros2/vservo_ros2/detector_node.py)、[`localizer_node.py`](src/vservo_ros2/vservo_ros2/localizer_node.py)、[`localization.py`](src/vservo_ros2/vservo_ros2/localization.py) | 图像时间、TF、首次可见主导、丢失交接、WAITING_TF |
| 4 | [`servo_node.py`](src/vservo_ros2/vservo_ros2/servo_node.py)、[`control.py`](src/vservo_ros2/vservo_ros2/control.py)、[`robot_node.py`](src/vservo_ros2/vservo_ros2/robot_node.py) | 完整目标、预览微步、关节限幅、控制线程、复位和故障 |
| 5 | [`calibration_node.py`](src/vservo_ros2/vservo_ros2/calibration_node.py)、[`cli.py`](src/vservo_ros2/vservo_ros2/cli.py) | 原始样本保存、静止检查、求解及独立验证 |
| 6 | [`web/server.py`](src/vservo_ros2/vservo_ros2/web/server.py)、[`web/ros_link.py`](src/vservo_ros2/vservo_ros2/web/ros_link.py)、[`web/workspace.py`](src/vservo_ros2/vservo_ros2/web/workspace.py)、[`web/static/app.js`](src/vservo_ros2/vservo_ros2/web/static/app.js) | HTTP 动作如何进入 ROS、会话绑定、调速提交、图像显示 |

数据流为 `camera → detector → localizer → servo → robot_bridge → ServoJ`；
`robot_bridge` 同时发布 `base → flange` 动态 TF 和手眼静态 TF，供定位换算使用。
`test/` 中每个测试也有中文说明，可结合对应模块阅读输入与预期行为。
`vendor/` 是本工程维护的相机与 DH 副本；未纳入版本库的厂家 `Robot.py` SDK 保持原样，
其使用方式和单位转换已在桥接调用处注明。

请按 [完整装配、标定、验证和伺服操作手册](docs/操作手册.md) 顺序执行。
接口、数据格式和 YOLO 接入见 [接口与扩展](docs/接口与扩展.md)。
本次设备验证结果见 [验证记录](docs/验证记录.md)。
从中文网页启动相机、采集、求解、独立验证及伺服监控，参见 [Web 操作台](docs/Web操作台.md)。

当前 ID 42 动态跟随采用先看到目标的相机主导，主相机丢失后由仍有目标的相机接管，法兰中心距标记 300 mm。
Web 启动监控即开启持续伺服，双路丢失时按指定六轴角度复位，随后等待并自动恢复。
Web 支持运行中调整线速度、角速度、关节加速度及 10%–100% 倍率，关节上限 2 rad/s。
默认 100 mm/s、30°/s、0.5 rad/s²。思维导图、交接行为和西瓜瓜蒂接入见
[实时跟随方案](docs/ID42实时跟随与瓜蒂方案.md)。每次继续开发先读 [Handoff](Handoff.md)。

## 构建

在 Jetson 终端执行，所有终端使用同一 ROS_DOMAIN_ID：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=99
cd /home/jetson/fairino_ws/Vservo_ros2
colcon build --symlink-install --packages-select vservo_ros2
source install/setup.bash
bash scripts/test_offline.sh
```

依赖：`rclpy`、`sensor_msgs`、`geometry_msgs`、`std_srvs`、`tf2_ros`、`cv_bridge`、
`launch_ros`、NumPy、SciPy、PyYAML，以及带 aruco 的 OpenCV ≥4.7、Orbbec SDK v2。
Jetson 现有 NumPy 1.26.4 / OpenCV 4.10.0 / pyorbbecsdk2 环境可复用，避免盲目升级 pip 包。
FR5 SDK 路径通过 `sdk_file` 参数指定，默认复用旧工程的 `scripts/robot/fairino/Robot.py`。
相机适配与 DH 参数已复制到本包 `vendor/`，不通过修改旧工程实现。

## 目录

```text
src/vservo_ros2/       ament_python 包
  config/             标定板与控制参数
  launch/             双相机、标定、完整伺服入口
  vservo_ros2/         算法、ROS 节点、离线工具
  test/               合成几何、回归、ROS/SDK 隔离测试
scripts/              内参、手眼、精度验证脚本
docs/                 中文操作、接口、验证记录
data/                 现场数据（不提交版本库）
```

流程：安装硬件 → 两路内参 → EIH 手眼 → ETH 手眼 → 独立验证 → 只读粗精定位

###### → 检查伺服目标 → 现场确认后显式允许运动。无数据或检测丢失时停止产生有效命令。
