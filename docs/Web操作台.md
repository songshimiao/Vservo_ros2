# FR5 双相机 Web 操作台

操作台在 Jetson 的 ROS 2 Humble / Python 3.10 环境运行。
网页可启动双相机和 ROS 采集节点，保存样本、求解、运行独立验证并查看报告。
双画面伺服监控显示定位与法兰目标，点击「启动或重载监控」会默认开启持续伺服。

## 启动

在 Jetson 上执行。

```bash
source /opt/ros/humble/setup.bash
export WS=/home/jetson/fairino_ws/Vservo_ros2
cd "$WS"
colcon build --symlink-install --packages-select vservo_ros2
source install/setup.bash
export ROS_DOMAIN_ID=99
ros2 run vservo_ros2 web_console --workspace "$WS" --port 8088
```

服务监听 Jetson 的 `127.0.0.1:8088`。在操作电脑另开终端建立 SSH 转发，然后在浏览器打开 `http://127.0.0.1:8088/`。

```bash
ssh -N -L 127.0.0.1:8088:127.0.0.1:8088 jetson@192.168.66.80
```

如果本机端口被占用，可将转发命令的第一个端口改成 18088，浏览器随之访问 18088。
关闭 SSH 转发结束浏览器访问。
Web 启动参数还支持 `--preview-fps`，默认每路编码 5 Hz。
浏览器按当前页面轮询 JPEG，每路最多约 4 次/秒。画面中的 Hz 来自 ROS 图像接收窗口。

Web 进程以启动它的 Jetson 用户读写 `$WS/data`。
HTTP 修改请求检查同源与本次进程生成的会话凭据。
通过可信 SSH 账户访问。运行期间的会话、报告和操作日志保存在 `data/` 下，受工程的 Git 忽略规则保护。

## 2026-11-23 当前持续伺服逻辑

Web 默认开放运动。点击「启动或重载监控」后自动请求持续伺服。仅观察时以 `--read-only` 启动 Web。已有的 `--allow-motion` 参数继续可用。程序重启后需要重新点击启动监控。

双画面默认在有效 ID42 上显示标签和坐标轴：X 红、Y 绿、Z 蓝，原点在标记中心，+Z 指入标记，显示轴长 40 mm。每路使用自己的内参和检测位姿，在相同时间戳的原图上绘制；目标丢失或结果过期后回到无轴原图。预览频率沿用 Web 默认每路 5 Hz 编码、页面最多约 4 次/秒轮询。

2026-11-24 坐标轴显示功能新增了 Web 后端订阅，需要先结束旧监控、重启 Web 服务、刷新页面，再核对并应用所需速度后启动监控。本功能仅刷新或重载监控不足以生效；重启 Web 后原来临时选择的速度需重新设置。

同日相机接管修复后，两路手眼静态 TF 一次完整发布，EIH 和 ETH 各自看到有效目标都能独立开始。主相机明确检测丢失后，下个定位周期选择另一台有效相机；只有两路都丢失才按原确认时间触发复位。页面逐路显示目标定位和 TF 状态；若已看到目标但缺少图像时刻的变换，则等待并暂停复位。此次修复只需结束旧监控、刷新后重新启动监控，无需再次重启 Web。

1. 任意相机先获得有效目标即可主导跟随。两路同时可用时按首次有效图像时刻选择，主相机持续有效就保持主导权。两路偏差作为观察信息。
2. 主相机目标丢失时，使用另一台仍有效的相机。原主相机重新看到目标时保留当前主导权，待当前主相机丢失才再次交接。两种来源都以法兰中心距目标 300 mm 为期望位姿。
3. 两路观测都失效后，保留持续伺服请求。默认等待 0.5 s 确认丢失，再按受限关节轨迹复位到 `[90, -110, 110, -180, -90, 0]` 度。复位会完整执行，到达后等待新目标并自动继续跟随。
4. Web 可设置线速度（1–200 mm/s）、角速度（1–60°/s）、关节加速度（0.1–2 rad/s²）和倍率（10%–100%）。默认 100 mm/s、30°/s、0.5 rad/s²。倍率同时缩放线速度、角速度和关节速度上限，100% 对应关节上限 2 rad/s，50% 为 1 rad/s；加速度独立设置。启动前或运行中点击「应用速度」，桥接确认后生效，复位遵循相同限制。
5. 页面显示当前主相机、正在复位、等待目标或故障原因。「停止伺服」会撤销持续请求并结束运动，「恢复持续伺服」重新开启请求。「结束监控」关闭本次节点。

机器人桥在视觉丢失、等待指令和复位完成时保持会话。控制器报错、机器人状态读取超时、定位节点心跳超时、关节跟随误差或工作空间越界会进入故障停止，页面显示原因。复位前检查模型路径，执行时逐步检查关节与工作空间范围。现场需要检查复位路径上的实体障碍物。

双路丢失后的复位准备现分为「正在减速」「正在检查复位路径」「正在复位」。81 点路径检查在后台规划进程执行，SDK 已使能时准备期间持续发送受限保持指令，检查不会阻塞控制周期。待检查期间重新看到目标会取消尚未执行的复位计划并继续跟随；已经开始的复位仍会完整执行。人工停止也会使待定计划失效。50 ms watchdog 与原有路径/限位检查继续生效。

以下保留此前的版本记录，当前目标选择与复位行为以本节为准。

2026-11-24 心跳修复后，桥接将定位心跳接收与 SDK/IK 分组执行，避免 SDK 等待导致心跳无法处理。页面显示心跳年龄和 500 ms 超时门槛。真实定位通信中断仍会停止伺服，故障需要人工处理后恢复。此次补丁结束旧监控、刷新网页、核对并应用速度，再启动/重载监控即可加载，无需重启 Web 服务。启动时显式选择的速度现可正确覆盖 YAML 默认值。

同日的 Control watchdog 修复将 SDK 控制移到独立线程、数值 IK 移到独立进程。页面新增指令间隔、控制周期、循环耗时、ServoJ 调用和使能耗时，故障后显示冻结的故障时记录。50 ms 指令间隔/SDK 耗时保护继续生效。若再次报错，保留完整含 `command_gap`、`ServoJ` 的故障文本及页面耗时，便于区分调度停顿和控制器通信等待。应用方式同上，无需重启 Web。

速度改动后须先结束旧监控、重启 Web、刷新页面，再应用速度并点击「启动或重载监控」。页面显示桥接确认的有效线速度、角速度、关节速度与加速度上限，这些数值为设置值。实际速度还受行程、姿态、关节加速度及控制器跟随影响，降低速度时按加速度限制逐步减速。Web 重启后倍率回到 100%，其余初值读取 `control.yaml`。

本版桥接对完整法兰目标求 IK，在 100 Hz 控制周期内生成受限关节指令，消除了旧 1 mm 微步引起的反复制动。IK 与 SDK 定时控制分组执行。旧 `/servo/next_flange_pose` 保留为兼容预览；实际新版桥接使用 `/servo/desired_flange_pose`。

## 2026-11-23 实时跟随与速度

用户已完成前期实体机械臂伺服测试。本轮跟随参考点改为法兰中心，精跟随距离 100 mm，法兰 +Z 朝向 ID 42。ETH 引导法兰接近 300 mm 中间目标，EIH 连续 3 帧看到同一目标且相机距目标不超过 450 mm 时接管。接管后由 EIH 持续跟随，ETH 的位姿差异作为辅助提示。详情及思维导图见 [ID42 实时跟随与瓜蒂方案](ID42实时跟随与瓜蒂方案.md)。

重启 Web 并重载监控后，在「双画面监控」选择 10%–100% 倍率并点击「应用速度」。可以在启动前设置，也可以在运行中修改。页面显示桥接确认的倍率和关节速度上限。倍率基于 `control.yaml` 既有速度限制，降低倍率按加速度限制逐步完成。Web 重启后倍率回到 100%。

EIH 短时丢失先 HOLD，优先重新捕获 EIH，500 ms 后可用新 ETH 观测重新引导。连续 2 s 没有被使用的有效观测时，重新从 ETH 获取。看门狗若已撤销运动使能，恢复目标后需重新点击开始伺服。到达跟随误差容限后继续处理新目标，移动 ID 42 会继续更新法兰微步。

本节为当前运行逻辑。下文保留早期标定与测试记录。

## 2026-11-17 Web 操作流程

1. 启动 Web 后端并打开页面。在「工作流总览」点「启动或重载双相机」。
页面显示两路画面状态、序列号、帧率及已选标定文件。页面也可接收从终端后启动的相机。

2. ETH 手眼默认选 [紧凑 ChArUco](charuco-compact.pdf)（175 × 125 mm，25 mm 格，18 mm 标记），适合固定在法兰或夹具上。
其他新训练会话默认选 [A4 ChArUco](charuco-a4.pdf)（266 × 190 mm，38 mm 格，28 mm 标记）。
可按当前相机的工作距离和安装空间切换规格，也可选旧板（105 × 75 mm，15 mm 格，11 mm 标记）。
按实际尺寸 100% 打印并量取校准线、格长与标记外边长。
独立验证自动采用所选标定文件记录的规格。
手眼训练与验证期间保持标定板的固定关系。

3. 在「内参与验证」分别为 EIH、ETH 新建训练会话。
点「启动本会话采集」，待页面显示可采集后保存样本并求解，随后点「在本页选用新结果」。
进入「独立验证」，新建会话并采满至少 5 张，再点「运行独立验证」。报告由页面直接显示。

4. 在「手眼与装配」完成对应装配和六项检查。
点击「启动本会话采集」会启动所需相机与只读机器人桥，并切换由 Web 管理的采集节点。
用现场示教器改变姿态，每次停稳至少 1 秒。采集按钮旁显示当前会话、角点、法兰静止和阻断原因。
求解、选用和独立验证与内参流程相同。EIH 训练与验证完成后再切换 ETH 的板固定方式。

5. 在总览选择四份有效的内参与手眼文件，打开「双画面监控」并点「启动或重载监控」。 
[ArUco ID 42](aruco-42.pdf) 按实际尺寸 100% 打印，确认黑框外边长 80 mm。
先让 ETH 看到目标并产生 COARSE 位姿，再检查粗定位悬停处 EIH 能否看到同一标记。
EIH 捕获并确认 3 帧后接管。两路同步观测时，ETH 辅助核对偏差。
画面显示当前定位来源、交接状态、配对时差和规划输出。
手眼标定仍用 ChArUco，保留至少 8 个角点的采集门槛。 [交接条件与故障记录](首次标定流程跑通bug记录.md#42-eth-粗定位与-eih-精定位交接)

6. 需要从网页驱动实体机械臂时，按 [操作手册的闭环步骤](操作手册.md) 确认装配、限位和控制配置，
在 Jetson 启动 Web 时添加 `--allow-motion`。
在监控页启动后，桥接先进入 `DISARMED`。
等收到有效法兰目标后，点「开始伺服」并确认现场，再由桥接执行 ServoJ。
「停止伺服」撤销使能并保留监控。「结束监控」关闭该会话启动的节点。

[共平台安装俯视图](双相机共平台安装示意.svg) 以世界 +X 指向作业区域，+Y 为平台移动方向，+Z 垂直纸面向外。EIH 随法兰运动，ETH 刚性固定在同一平台。重新安装后重新做两路手眼标定和独立验证。

「系统诊断」可看由 Web 启动的节点日志。Web 进程退出时会结束它启动的节点。已有终端启动的相机可供 Web 订阅，切换到完整监控前需要退出与其争用同一相机或 ROS 节点名的外部入口。

## 2026-11-16 ETH 验证未通过

### 当前工作进度

SSH 连入 Jetson 后通过运行中的 Web `/api/state` 核对了总览。两路相机已加载对应的 640×480 内参。只读机器人桥为 `READ_ONLY`，`base → flange` TF 在线。当前没有运行标定采集节点。

| 步骤 | 核对结果 |
| --- | --- |
| EIH 305 内参 | 已标定，独立验证通过，最大重投影误差 0.228 px |
| ETH 335 内参 | 已标定，独立验证通过，最大重投影误差 0.142 px |
| EIH 手眼 | 尚无手眼文件，待标定和独立验证 |
| ETH 手眼求解 | `eth_handeye_train_20261116_154424_b8af20ac` 采集 16 组。训练最大 3.252 mm / 1.679°，留出最大 5.766 mm / 1.420°，均通过求解门槛 |
| ETH 手眼独立验证 | `eth_handeye_check_20261116_163225_9d02c37f` 采集 7 组。报告 `passed: false`，退出码 2 |

### 退出码 2 的直接原因

脚本对每组数据用 `inverse(base_T_flange) × base_T_eth_camera × eth_camera_T_board` 推回板在法兰坐标系中的位姿，再与训练求解保存的 `reference_matrix`（`flange_T_board`）比较。所有样本的平移最大值和转角最大值都在门槛内才通过。此次报告已写入，随后 `ros2 run` 显示进程退出码 2。

| 指标 | 本次实测 | 默认门槛 |
| --- | --- | --- |
| 最大平移误差 | 2.167 mm | ≤10 mm，通过 |
| 最大转角误差 | 2.820° | ≤2°，未通过 |
| 转角 RMS | 1.372° | 用于观察分布，判定取最大值 |

逐组计算定位到验证会话的 `0002.png`（16:44:58）。它的平移误差为 1.291 mm，旋转误差为 2.820°。其余 6 组的旋转误差为 0.664°–1.273°。该帧检测到 8 个角点，恰好达到采集最低要求。其图像重投影 RMS 为 0.114 px。像素拟合检查的是这张图上的投影，手眼验证还比较基座、法兰、相机与板的三维姿态，因此两项数值可以同时出现。

`data/calibration/eth_handeye.json` 和 ETH 训练会话中的求解文件 SHA-256 相同，也与本次 Web 验证会话记录的 SHA-256 相同。训练与验证的相机序列号、板规格和使用的内参一致。在 Jetson 上将同一条验证命令输出到临时文件，复现了相同 JSON 和退出码 2。已确定触发项为 `0002.png` 的姿态超限。单张图与位姿记录不足以确定物理成因。复测时优先检查该类视角下是否能看到更多分布均匀的角点，再检查法兰夹具、标定板和相机支架是否松动或重新安装。

### 保留现场，从 ETH 独立验证复测

1. 从 Jetson 上当前运行的双相机、只读机器人桥和 Web 操作台开始。检查 335 支架、机器人基座和板相对法兰的安装是否仍与 15:44 训练会话一致。保持原安装可直接复测。若移动过板或相机，在「手眼与装配」选择「ETH · 335」，新建训练会话、重新求解，再保持新安装完成独立验证。
2. 在「验证报告」选择 `eth_handeye_check_20261116_163225_9d02c37f.json`，在「标定会话」打开同名会话查看 `0002.png`。原报告和样本保留作比较。页面的「标记重采」是备注，统计仍会读取原清单。若需要查看原始报告及退出码，在 Jetson 执行。

   ```bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   python3 -m json.tool data/reports/eth_handeye_check_20261116_163225_9d02c37f.json
   cat data/reports/eth_handeye_check_20261116_163225_9d02c37f.exit
   ```

3. 在总览把 ETH 手眼选为 `calibration/eth_handeye.json`。打开「手眼与装配」→「ETH · 335」→「独立验证」→「创建独立验证会话」。完成清单，在 Jetson 终端 C 启动新会话。输入新名字，旧会话用于回看。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p '新的 ETH 手眼验证会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eth mode:=handeye \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eth_handeye.json" protect_output:=true
   ```

4. 用现场示教器选至少 5 组新的代表性姿态，包含超限样本附近的视角。让板完整入镜、角点清晰且在板上分布均匀，每次停稳至少 1 秒。点「保存有效样本」，或在 Jetson 终端 D 用下面的命令。若出现 `extrapolation into the future`，保持机器人静止，等下一帧再试，并检查法兰 TF 是否持续更新。角点不足和 TF 失败的请求在采集时已经被拒绝。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 采够后到终端 C 按 Ctrl+C。终端 D 执行以下验证命令，输入新会话名。独立验证会话只采集，求解文件沿用第 3 步在总览选择的路径。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p '新的 ETH 手眼验证会话名: ' SESSION
   REPORT="$PWD/data/reports/$SESSION.json"
   mkdir -p "$PWD/data/reports"
   rc=0
   bash scripts/test_handeye.sh "$PWD/data/sessions/$SESSION/samples.json" \
     "$PWD/data/calibration/eth_handeye.json" "$REPORT" || rc=$?
   printf '%s\n' "$rc" > "${REPORT%.json}.exit"
   printf '退出码 %s\n' "$rc"
   if test -f "$REPORT"; then python3 -m json.tool "$REPORT"; fi
   ```

6. 在「验证报告」刷新列表查看新报告。退出码 0 且 `passed: true` 时，回总览确认 ETH 独立验证状态。退出码 2 时查看超限指标，若相近视角仍反复超限，做一轮覆盖更多距离和倾角的 ETH 训练，再用新会话独立验证。退出码 1 时先看终端的文件、会话或配置错误。补齐 EIH 手眼和独立验证后，可进入只读伺服检查。

   Web 总览通过会话计划中的文件路径和 SHA-256 关联报告，并检查报告相对样本清单的时间、`independent_session`、`passed` 和 `.exit`。总览采用验证时选择的那份文件路径。例如同一 JSON 从 `sessions/` 复制到 `calibration/` 后，页面为两个路径分别关联验证报告。

## 旧版终端联动操作参考

两个角色各自经历「标定 → 独立验证」。总览按 EIH、ETH 顺序列出手眼步骤，已有 ETH 数据可先完成 ETH 复测，再补 EIH。切换手眼角色时，先停旧采集节点。训练和验证使用两个会话目录，并在同一个装配阶段完成。

- [连接 Web](#打开页面并确认运行状态) · [双相机与机器人桥](#启动双相机和法兰-tf)
- [305 内参标定](#eih-内参标定) · [305 内参独立验证](#eih-内参独立验证) · [335 内参标定](#eth-内参标定) · [335 内参独立验证](#eth-内参独立验证)
- [EIH 手眼标定](#eih-手眼标定) · [EIH 手眼独立验证](#eih-手眼独立验证) · [ETH 手眼标定](#eth-手眼标定) · [ETH 手眼独立验证](#eth-手眼独立验证)
- [只读伺服检查](#只读伺服检查) · [定位精度采集](操作手册.md#75-定位精度)

### 打开页面并确认运行状态

1. 在操作电脑终端连接 Jetson。已经开启 Web 后端时直接进入第 3 步。

   ```bash
   ssh jetson@192.168.66.80
   ```

2. 在 Jetson 的 Web 终端启动后端。首次安装需执行本页「启动」中的构建命令。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 run vservo_ros2 web_console --workspace "$PWD" --port 8088
   ```

3. 在操作电脑另开终端执行 SSH 转发。转发保持运行，用浏览器打开 `http://127.0.0.1:8088/`。

   ```bash
   ssh -N -L 127.0.0.1:8088:127.0.0.1:8088 jetson@192.168.66.80
   ```

4. 打开「工作流总览」和「系统诊断」，核对两路图像、序列号、尺寸、所选文件和机器人状态。未开启标定会话时，采集和求解服务显示未发现。点各步骤的「进入步骤」可查看该角色的当前状态。

### 启动双相机和法兰 TF

1. 在 Jetson 终端 A 启动双相机。若已有同一套相机节点，保留该进程。首次采集内参时使用下列命令。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 launch vservo_ros2 dual_camera.launch.py
   ```

2. 有两份内参后，在总览选定文件。到终端 A 按 Ctrl+C，然后按「用所选内参重启双相机」命令启动加载。两份文件都位于 `data/calibration/` 时可直接执行下列命令。会话目录中的结果由总览命令填写实际路径。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 launch vservo_ros2 dual_camera.launch.py \
     eih_intrinsics:="$PWD/data/calibration/eih_intrinsics.json" \
     eth_intrinsics:="$PWD/data/calibration/eth_intrinsics.json"
   ```

3. 做手眼标定或验证时，在 Jetson 终端 B 开启机器人桥。已有 `READ_ONLY` 桥接和有效法兰 TF 时保留该进程。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 run vservo_ros2 robot_bridge --ros-args -p execute:=false
   ```

4. 在 Jetson 终端 D 检查话题和 TF。每条 `--once` 命令收到一条消息会退出。看完 TF 后按 Ctrl+C。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 topic echo /eih/metadata --once --qos-durability transient_local
   ros2 topic echo /eth/metadata --once --qos-durability transient_local
   ros2 topic echo /robot/status --once
   ros2 run tf2_ros tf2_echo base flange
   ```

   做手眼时，两路相机应加载 `source: charuco_calibrated` 和 `validated: true` 的内参。终端 A、B 保持运行，终端 C 启动当前采集会话，终端 D 用于服务检查和验证。每次切换会话，在终端 C 按 Ctrl+C 后再运行新会话命令。

### EIH 内参标定

1. 从 EIH 的相机画面开始。让 Gemini 305 在线，保持作业分辨率 640×480。手持板或使用可摆位支架，在画面中改变板的位置、距离和倾角。保持终端 A 的相机节点运行。确保终端 C 当前没有采集节点。
2. 打开「内参与验证」→「EIH · 305」。点「采集与求解」→「新建标定会话」或「重新标定 · 新建会话」。页面会显示以 `eih_intrinsic_train_` 开头的会话名。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 内参标定会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eih mode:=intrinsic \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eih_intrinsics.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。把板摆到新姿态并保持画面清晰后点「保存有效图片」。最少 16 组，建议 25–40 组。覆盖图像中央和边缘，包含远近和不同倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后点「求解内参」，或在终端 D 执行下面的命令。求解成功时输出 `$PWD/data/sessions/$SESSION/eih_intrinsics.json`。查看训练与留出重投影 RMS。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/solve std_srvs/srv/Trigger '{}'
   ```

6. 在页面点「在本页选用新结果」，或在总览下拉框选择刚生成的 `sessions/…/eih_intrinsics.json`。到终端 C 按 Ctrl+C 停止训练节点。进入本角色的独立内参验证。

### EIH 内参独立验证

1. 从 EIH 的求解结果开始。让 Gemini 305 在线，保持作业分辨率 640×480。手持板或使用可摆位支架，在画面中改变板的位置、距离和倾角。保持终端 A 的相机节点运行。停止该角色训练节点所在终端 C（按 Ctrl+C）。在「工作流总览」的「所选标定与历史对比」选择本次要验证的 EIH 内参 JSON。准备另取一组板姿态。
2. 打开「内参与验证」→「EIH · 305」。点「独立验证」→「创建独立验证会话」。页面会显示以 `eih_intrinsic_check_` 开头的会话名。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 内参独立验证会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eih mode:=intrinsic \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eih_intrinsics.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。把板摆到新姿态并保持画面清晰后点「保存有效图片」。采至少 5 组新样本，覆盖实际使用范围。覆盖图像中央和边缘，包含远近和不同倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后到终端 C 按 Ctrl+C，固定本次样本清单。在终端 D 执行页面「独立验证（采满至少 5 组后执行）」命令。下列是等效命令，会从 Web 会话计划读取已选标定文件，记录 JSON 报告与退出码。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 内参独立验证会话名: ' SESSION
   SESSION_DIR="$PWD/data/sessions/$SESSION"
   CALIBRATION=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["calibration"])' \
     "$SESSION_DIR/web_session.json")
   REPORT="$PWD/data/reports/$SESSION.json"
   mkdir -p "$PWD/data/reports"
   rc=0
   bash scripts/test_intrinsics.sh "$SESSION_DIR/samples.json" \
     "$PWD/data/$CALIBRATION" "$REPORT" || rc=$?
   printf '%s\n' "$rc" > "${REPORT%.json}.exit"
   printf '退出码 %s\n' "$rc"
   if test -f "$REPORT"; then python3 -m json.tool "$REPORT"; fi
   ```

6. 在「验证报告」点击「刷新报告」，选择本次会话。退出码 `0` 且 `passed: true` 时通过。内参检查单帧重投影误差的最大值，默认 ≤1 px。总览选择与会话计划相同的标定文件后，显示对应独立报告。两台内参分别通过后，使用所选文件启动双相机，再进入手眼。

### ETH 内参标定

1. 从 ETH 的相机画面开始。让 Gemini 335 在线，保持作业分辨率 640×480。手持板或使用可摆位支架，在画面中改变板的位置、距离和倾角。保持终端 A 的相机节点运行。确保终端 C 当前没有采集节点。
2. 打开「内参与验证」→「ETH · 335」。点「采集与求解」→「新建标定会话」或「重新标定 · 新建会话」。页面会显示以 `eth_intrinsic_train_` 开头的会话名。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 内参标定会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eth mode:=intrinsic \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eth_intrinsics.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。把板摆到新姿态并保持画面清晰后点「保存有效图片」。最少 16 组，建议 25–40 组。覆盖图像中央和边缘，包含远近和不同倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后点「求解内参」，或在终端 D 执行下面的命令。求解成功时输出 `$PWD/data/sessions/$SESSION/eth_intrinsics.json`。查看训练与留出重投影 RMS。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/solve std_srvs/srv/Trigger '{}'
   ```

6. 在页面点「在本页选用新结果」，或在总览下拉框选择刚生成的 `sessions/…/eth_intrinsics.json`。到终端 C 按 Ctrl+C 停止训练节点。进入本角色的独立内参验证。

### ETH 内参独立验证

1. 从 ETH 的求解结果开始。让 Gemini 335 在线，保持作业分辨率 640×480。手持板或使用可摆位支架，在画面中改变板的位置、距离和倾角。保持终端 A 的相机节点运行。停止该角色训练节点所在终端 C（按 Ctrl+C）。在「工作流总览」的「所选标定与历史对比」选择本次要验证的 ETH 内参 JSON。准备另取一组板姿态。
2. 打开「内参与验证」→「ETH · 335」。点「独立验证」→「创建独立验证会话」。页面会显示以 `eth_intrinsic_check_` 开头的会话名。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 内参独立验证会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eth mode:=intrinsic \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eth_intrinsics.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。把板摆到新姿态并保持画面清晰后点「保存有效图片」。采至少 5 组新样本，覆盖实际使用范围。覆盖图像中央和边缘，包含远近和不同倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后到终端 C 按 Ctrl+C，固定本次样本清单。在终端 D 执行页面「独立验证（采满至少 5 组后执行）」命令。下列是等效命令，会从 Web 会话计划读取已选标定文件，记录 JSON 报告与退出码。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 内参独立验证会话名: ' SESSION
   SESSION_DIR="$PWD/data/sessions/$SESSION"
   CALIBRATION=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["calibration"])' \
     "$SESSION_DIR/web_session.json")
   REPORT="$PWD/data/reports/$SESSION.json"
   mkdir -p "$PWD/data/reports"
   rc=0
   bash scripts/test_intrinsics.sh "$SESSION_DIR/samples.json" \
     "$PWD/data/$CALIBRATION" "$REPORT" || rc=$?
   printf '%s\n' "$rc" > "${REPORT%.json}.exit"
   printf '退出码 %s\n' "$rc"
   if test -f "$REPORT"; then python3 -m json.tool "$REPORT"; fi
   ```

6. 在「验证报告」点击「刷新报告」，选择本次会话。退出码 `0` 且 `passed: true` 时通过。内参检查单帧重投影误差的最大值，默认 ≤1 px。总览选择与会话计划相同的标定文件后，显示对应独立报告。两台内参分别通过后，使用所选文件启动双相机，再进入手眼。

### EIH 手眼标定

1. 从 EIH 的相机画面开始。把板刚性固定在台面或基座侧，将 305 固定在法兰。训练和验证期间保持板相对 `base` 的位置与朝向，机械臂带动 305 观察板。保持终端 A 的双路相机加载已验证内参，终端 B 显示 `READ_ONLY` 并持续提供 `base → flange` TF。确保终端 C 当前没有采集节点。如果另一角色仍在手眼采集，在其启动终端按 Ctrl+C，确认它结束后再改变装配。
2. 打开「手眼与装配」→「EIH · 305」。点「采集与求解」→「新建标定会话」或「重新标定 · 新建会话」。页面会显示以 `eih_handeye_train_` 开头的会话名。逐项确认板尺寸和打印比例、相机与供电、视野、tool0、只读 TF、固定方式和停稳时间，然后勾选检查清单。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 手眼标定会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eih mode:=handeye \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eih_handeye.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。通过现场示教器改变机器人姿态，每次停稳至少 1 秒后点「保存有效样本」。最少 16 组，建议 25–40 组。绕至少两个不同方向旋转，配合平移，让板完整、清晰地出现在画面里。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后点「求解手眼」，或在终端 D 执行下面的命令。求解成功时输出 `$PWD/data/sessions/$SESSION/eih_handeye.json`。核对父系 `flange`、子系 `eih_optical_frame`，并查看训练及留出误差。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/solve std_srvs/srv/Trigger '{}'
   ```

6. 在页面点「在本页选用新结果」，或在总览下拉框选择刚生成的 `sessions/…/eih_handeye.json`。到终端 C 按 Ctrl+C 停止训练节点。保持板的安装，进入本角色的独立手眼验证。

### EIH 手眼独立验证

1. 从 EIH 的求解结果开始。把板刚性固定在台面或基座侧，将 305 固定在法兰。训练和验证期间保持板相对 `base` 的位置与朝向，机械臂带动 305 观察板。保持终端 A 的双路相机加载已验证内参，终端 B 显示 `READ_ONLY` 并持续提供 `base → flange` TF。停止该角色训练节点所在终端 C（按 Ctrl+C）。在「工作流总览」的「所选标定与历史对比」选择本次要验证的 EIH 手眼 JSON。保留训练时的标定板安装关系。
2. 打开「手眼与装配」→「EIH · 305」。点「独立验证」→「创建独立验证会话」。页面会显示以 `eih_handeye_check_` 开头的会话名。逐项确认板尺寸和打印比例、相机与供电、视野、tool0、只读 TF、固定方式和停稳时间，然后勾选检查清单。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 手眼独立验证会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eih mode:=handeye \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eih_handeye.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。通过现场示教器改变机器人姿态，每次停稳至少 1 秒后点「保存有效样本」。采至少 5 组新样本，覆盖实际使用范围。包含不同距离、位置和倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eih/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后到终端 C 按 Ctrl+C，固定本次样本清单。在终端 D 执行页面「独立验证（采满至少 5 组后执行）」命令。下列是等效命令，会从 Web 会话计划读取已选标定文件，记录 JSON 报告与退出码。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'EIH 手眼独立验证会话名: ' SESSION
   SESSION_DIR="$PWD/data/sessions/$SESSION"
   CALIBRATION=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["calibration"])' \
     "$SESSION_DIR/web_session.json")
   REPORT="$PWD/data/reports/$SESSION.json"
   mkdir -p "$PWD/data/reports"
   rc=0
   bash scripts/test_handeye.sh "$SESSION_DIR/samples.json" \
     "$PWD/data/$CALIBRATION" "$REPORT" || rc=$?
   printf '%s\n' "$rc" > "${REPORT%.json}.exit"
   printf '退出码 %s\n' "$rc"
   if test -f "$REPORT"; then python3 -m json.tool "$REPORT"; fi
   ```

6. 在「验证报告」点击「刷新报告」，选择本次会话。退出码 `0` 且 `passed: true` 时通过。手眼看平移和转角的最大值，默认分别 ≤10 mm 和 ≤2°。总览选择与会话计划相同的标定文件后，显示对应独立报告。通过后再拆板或切换角色。

### ETH 手眼标定

1. 从 ETH 的相机画面开始。将板刚性固定在末端法兰或夹具。训练和验证期间保持板相对 `flange` 的位置与朝向，同时固定 335 支架与机器人基座。保持终端 A 的双路相机加载已验证内参，终端 B 显示 `READ_ONLY` 并持续提供 `base → flange` TF。确保终端 C 当前没有采集节点。如果另一角色仍在手眼采集，在其启动终端按 Ctrl+C，确认它结束后再改变装配。
2. 打开「手眼与装配」→「ETH · 335」。点「采集与求解」→「新建标定会话」或「重新标定 · 新建会话」。页面会显示以 `eth_handeye_train_` 开头的会话名。逐项确认板尺寸和打印比例、相机与供电、视野、tool0、只读 TF、固定方式和停稳时间，然后勾选检查清单。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 手眼标定会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eth mode:=handeye \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eth_handeye.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。通过现场示教器改变机器人姿态，每次停稳至少 1 秒后点「保存有效样本」。最少 16 组，建议 25–40 组。绕至少两个不同方向旋转，配合平移，让板完整、清晰地出现在画面里。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后点「求解手眼」，或在终端 D 执行下面的命令。求解成功时输出 `$PWD/data/sessions/$SESSION/eth_handeye.json`。核对父系 `base`、子系 `eth_optical_frame`，并查看训练及留出误差。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/solve std_srvs/srv/Trigger '{}'
   ```

6. 在页面点「在本页选用新结果」，或在总览下拉框选择刚生成的 `sessions/…/eth_handeye.json`。到终端 C 按 Ctrl+C 停止训练节点。保持板的安装，进入本角色的独立手眼验证。

### ETH 手眼独立验证

1. 从 ETH 的求解结果开始。将板刚性固定在末端法兰或夹具。训练和验证期间保持板相对 `flange` 的位置与朝向，同时固定 335 支架与机器人基座。保持终端 A 的双路相机加载已验证内参，终端 B 显示 `READ_ONLY` 并持续提供 `base → flange` TF。停止该角色训练节点所在终端 C（按 Ctrl+C）。在「工作流总览」的「所选标定与历史对比」选择本次要验证的 ETH 手眼 JSON。保留训练时的标定板安装关系。
2. 打开「手眼与装配」→「ETH · 335」。点「独立验证」→「创建独立验证会话」。页面会显示以 `eth_handeye_check_` 开头的会话名。逐项确认板尺寸和打印比例、相机与供电、视野、tool0、只读 TF、固定方式和停稳时间，然后勾选检查清单。
3. 在 Jetson 新终端 C 执行下面的命令。输入页面显示的完整会话名。也可直接复制页面「启动采集节点（新终端）」中的 `ros2 launch` 行。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 手眼独立验证会话名: ' SESSION
   ros2 launch vservo_ros2 calibration.launch.py role:=eth mode:=handeye \
     session_dir:="$PWD/data/sessions/$SESSION" \
     output:="$PWD/data/sessions/$SESSION/eth_handeye.json" protect_output:=true
   ```

4. 等页面显示本会话和角点预览。通过现场示教器改变机器人姿态，每次停稳至少 1 秒后点「保存有效样本」。采至少 5 组新样本，覆盖实际使用范围。包含不同距离、位置和倾角。查看成功反馈与样本数。命令行采集可在 Jetson 终端 D 执行以下命令，每个新姿态调用一次。页面按钮和命令行任选一个。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 service call /eth/calibration/capture std_srvs/srv/Trigger '{}'
   ```

5. 样本采齐后到终端 C 按 Ctrl+C，固定本次样本清单。在终端 D 执行页面「独立验证（采满至少 5 组后执行）」命令。下列是等效命令，会从 Web 会话计划读取已选标定文件，记录 JSON 报告与退出码。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   read -r -p 'ETH 手眼独立验证会话名: ' SESSION
   SESSION_DIR="$PWD/data/sessions/$SESSION"
   CALIBRATION=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["calibration"])' \
     "$SESSION_DIR/web_session.json")
   REPORT="$PWD/data/reports/$SESSION.json"
   mkdir -p "$PWD/data/reports"
   rc=0
   bash scripts/test_handeye.sh "$SESSION_DIR/samples.json" \
     "$PWD/data/$CALIBRATION" "$REPORT" || rc=$?
   printf '%s\n' "$rc" > "${REPORT%.json}.exit"
   printf '退出码 %s\n' "$rc"
   if test -f "$REPORT"; then python3 -m json.tool "$REPORT"; fi
   ```

6. 在「验证报告」点击「刷新报告」，选择本次会话。退出码 `0` 且 `passed: true` 时通过。手眼看平移和转角的最大值，默认分别 ≤10 mm 和 ≤2°。总览选择与会话计划相同的标定文件后，显示对应独立报告。通过后再拆板或切换角色。

### 只读伺服检查

1. 从总览确认所选两路内参、两路手眼都通过独立验证。把任务目标固定到作业台，让两台相机看到同一个目标。台面标定板或法兰标定板如需拆装，在对应手眼验证结束后操作。
2. 在终端 C 停止采集节点，在终端 A 停止独立双相机，在终端 B 停止独立机器人桥。分别在相应终端按 Ctrl+C。Web 和 SSH 转发保持运行。
3. 在总览选择四份文件，复制「启动完整只读伺服（先停止独立相机及桥接）」到 Jetson 终端 A。下面是四份文件都在 `data/calibration/` 时的完整指令。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 launch vservo_ros2 visual_servo.launch.py execute:=false \
     eih_intrinsics:="$PWD/data/calibration/eih_intrinsics.json" \
     eth_intrinsics:="$PWD/data/calibration/eth_intrinsics.json" \
     eih_handeye:="$PWD/data/calibration/eih_handeye.json" \
     eth_handeye:="$PWD/data/calibration/eth_handeye.json"
   ```

4. 打开「双画面监控」。先确认两路图像在线和机器人 `READ_ONLY`，再查看 `COARSE`、`FINE`、`HOLD` 或 `INCONSISTENT`，以及对应的目标和下一步法兰位姿。也可在 Jetson 终端 D 检查话题，每条命令收一条即退出。

   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/jetson/fairino_ws/Vservo_ros2
   source install/setup.bash
   export ROS_DOMAIN_ID=99
   ros2 topic echo /robot/status --once
   ros2 topic echo /localization/state --once
   ros2 topic echo /localization/diagnostics --once
   ros2 topic echo /localization/target_pose --once
   ros2 topic echo /servo/next_flange_pose --once
   ```

   只有定位器和伺服节点产生有效输出时，最后两个命令才会收到消息。处于 HOLD 或一直无消息时，可按 Ctrl+C，查看 Web 的诊断和伺服状态。检查结束后到终端 A 按 Ctrl+C。双画面监控和定位精度采样也可继续共用该只读进程。

## 会话、报告与诊断

- 重标定会在 `data/sessions/` 创建带时间戳和随机后缀的新目录。
- 训练结果写入本会话，求解时通过原子创建保护已有文件。
- 选用新结果只影响本浏览器生成的命令。
- 旧结果保留，可在总览查看时间、序列号、误差和矩阵。

- 打开“标定会话”查看历史样本、角点数和手眼重投影误差。
- 坏样本可加重采标记，原始清单和图像保留。
- 标记供现场筛选记录，求解仍使用原始清单，建议另建会话采集要用于求解的数据。

- 验证报告页面读取 `data/reports/` 下的 JSON，支持下载。
- Web 生成的验证命令记录真实退出码。
- 历史报告缺少退出码时，页面按 `passed` 字段显示推断值。
- 总览以本页创建的独立会话、所选文件的 SHA-256、通过的报告和脚本退出码判断验证完成。历史报告仍可从报告页查阅。

- 诊断检查 ROS 域、相机图像和身份、服务发现、法兰 TF、标定文件及机器人状态。
- 相机日志来自近期 ROS 日志。驱动若在节点退出前只写到终端，USB busy 的具体原因仍以相机终端日志为准。

- 清晰度和高亮比例来自图像统计，仅作模糊及反光排查提示。是否可采样以实际 Trigger 返回为准。ROS 断流或位姿陈旧时，操作台清空过期画面与位姿。

## 运动边界与接口

- Web 提供的命令使用 `execute:=false`。
- 浏览器可显式调用当前匹配会话的 `/<role>/calibration/capture` 与 `solve` 两个 Trigger。
- 手眼采集还要求机器人桥报告 `READ_ONLY`，并确认相应的装配清单。
- 旧版命令可作为备用。当前版本的 Web 运动入口通过 `--allow-motion` 启用。

新诊断话题包括
- `/<role>/calibration/preview_status`（String JSON）
- `/<role>/calibration/preview_image`（CompressedImage）
- `/localization/diagnostics`（String JSON）。
前两个提供同一采样时刻的角点和条件，后一个显示定位器实际使用的配对条件和差值。
现有采集、求解服务及旧 launch 参数继续可用，Web 命令增加 `protect_output:=true`。

## 验证方式

```bash
colcon test --packages-select vservo_ros2 --event-handlers console_direct+
colcon test-result --verbose
bash scripts/test_offline.sh
```

在浏览器分别检查总览、两路画面、角点预览、采集与求解反馈、报告及诊断。验证手眼实测效果需要按对应装配方式固定板并采集现场数据。

## 2026-11-09 验收记录

| 项目 | 结果 |
| --- | --- |
| Jetson 原生构建 | `colcon build --symlink-install --packages-select vservo_ros2` 通过 |
| 回归与离线测试 | 在独立 ROS 域 97 执行，`colcon test` 和 `scripts/test_offline.sh` 均为 29 项通过，含 8 项 Web 后端测试 |
| 实机相机与 Chrome | Gemini 305、335 同时发布 640×480，实测约 29.5 Hz，现有内参匹配。检查了双画面、已标定跳过、重标入口、命令复制和诊断 |
| 断流 | 停止测试启动的相机后，Chrome 清除两路旧图像并显示离线 |
| 标定交互 | 在独立 ROS 域 98 和临时工作区，以合成板图检查无标定提示、角点叠加、采集拒绝、求解失败和 22 张采集后的真实内参服务求解成功。检查了结果选择、样本页面和手眼装配页 |
| 报告 | 用另一组合成板姿态执行生成的独立内参验证命令，检查 JSON、实际退出码、下载响应和总览关联 |
| 运动隔离 | Web 只创建四个标定 Trigger 客户端。带有效同源凭据访问运动路由返回 404，缺少同源凭据的 POST 返回 403。伺服状态页面以隔离测试消息检查 |

运行现有 ROS 集成测试时，建议使用与工作现场不同的 `ROS_DOMAIN_ID`。首次并行测试与实机节点共享域时，定位集成用例未进入 FINE。隔离域复测通过。现场手眼采样、绝对定位精度和机器人闭环仍需按操作手册验证。

## 文件清单

| 操作 | 路径 |
| --- | --- |
| 新增后端 | `src/vservo_ros2/vservo_ros2/web/__init__.py`、`server.py`、`ros_link.py`、`workspace.py`（同目录） |
| 新增静态页面 | `src/vservo_ros2/vservo_ros2/web/static/index.html`、`app.js`、`style.css`（同目录） |
| 新增测试 | `src/vservo_ros2/test/test_web_console.py` |
| 修改打包 | `src/vservo_ros2/setup.py`、`src/vservo_ros2/package.xml` |
| 修改标定诊断和输出保护 | `src/vservo_ros2/launch/calibration.launch.py`、`src/vservo_ros2/vservo_ros2/calibration_node.py`、同目录的 `board.py`、`geometry.py` |
| 修改定位诊断 | `src/vservo_ros2/vservo_ros2/localization.py`、同目录的 `localizer_node.py` |
| 更新文档 | `README.md`、`docs/Web操作台.md` |
