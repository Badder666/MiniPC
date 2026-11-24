# 六轴控制器 RViz2 可视化

使用被控机械臂 `new_hj_arm2` 的 URDF。模型与七个 STL 网格已从旧工程原样复制到 `src/new_hj_arm2`，因此新工作区可以独立构建，不需要 MoveIt。这里显示的是被控机械臂，不能用它计算自定义控制器的重力补偿。

## 安装与启动

把新工作区源码同步到小电脑 `/home/badder/minipc/New_Custom_Controller` 后执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rviz2 ros-humble-robot-state-publisher \
  ros-humble-tf2-ros ros-humble-diagnostic-msgs ros-humble-visualization-msgs
cd /home/badder/minipc/New_Custom_Controller
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 command_port:=/dev/ttyACM1
```

`port` 是自定义控制器，`command_port` 是机械臂下位机；推荐分别填写稳定的 `/dev/serial/by-id/...` 路径。启动时默认 `forward:=false`，只读控制器与机械臂反馈，不发送命令。

开启实际下发：

```bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 command_port:=/dev/ttyACM1 forward:=true
```

只有控制器、未接机械臂时，可加 `read_feedback:=false`：

```bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 read_feedback:=false
```

不要同时运行另一套 USB 接收或串口桥接程序占用相同设备。沿用旧操作输入时加 `legacy_input:=true`，并先加载旧工作区的消息环境，详见 README。

## 画面含义

- **Target Pose**：绿色半透明模型，表示控制器位置经过上位机轴序、比例和零位映射后的机械臂目标。
- **Actual Pose**：不透明原模型，表示机械臂下位机返回的实际关节位置；未收到反馈时不发布虚假的实际关节姿态。
- **Controller Status**：显示模式、控制器接收频率、接收间隔、序号、设备时间、有效位、六轴原始位置/目标/实际/误差和按键位图。

标记文字使用英文以兼容 RViz 默认字体。单位均为弧度，误差为目标减实际。`MOTOR` 列表明当前机械臂关节对应的控制器电机。

未收到数据时，目标模型显示机械臂折叠参考姿态 `[0, 2.4, -2.4, 0, 0, 0]`，数值显示 `--`，状态为 `WAITING / STALE`。这是 URDF/Motion Planning 配置中的 home 姿态。映射配置对六轴采用负比例，并使用偏置 `[0, 2.4, -2.4, 0, 0, 0]`，所以控制器零位时目标就是折叠姿态。

无效轴保留上一目标位置并标为 `INVALID`。目标超出 URDF 关节限位显示 `LIMIT`，显示层不钳位、不改变实际控制命令。

超过默认 250 ms 没有新状态或反馈，显示为过期，误差停止显示，已经获得的姿态保留为最后一次值。此阈值仅用于可视化，不关闭电机或改变下位机输出。

## 映射与 TF

可视化节点和下发桥接同时加载 `config/command_bridge.yaml`。支持通过 `mapping_config:=/绝对路径/配置.yaml` 更换配置。当前比例为 `[-1, -1, +1, -1, -1, +1]`，其中 J3 使用 +1 以匹配实际控制器方向；零位偏置让控制器全零落在折叠姿态。关节映射公式与下发一致：

```text
target[j] = scales[j] × motor_positions[axis_order[j]] + offsets[j]
```

URDF 关节为 `J1..J6`，原桥接反馈名称为 `joint_1..joint_6`。显示层按配置名称校验并重排，再发布为 URDF 关节名；不能将 `motor_1..motor_6` 原始话题直接接到此 URDF。

两套模型使用独立 TF：

```text
world → target/world → target/base_link → target/Link1 → ... → target/Link6
world → actual/world → actual/base_link → actual/Link1 → ... → actual/Link6
```

RViz 固定坐标为 `world`。目标和实际模型各自订阅描述话题，并使用对应 TF 前缀；不能另加重复的 `world→base_link` 变换。

| 话题 | 用途 |
| --- | --- |
| `/custom_controller/state` | 自定义控制器原始六轴、按键和有效位 |
| `/custom_controller/arm_joint_states` | 下位机实际关节反馈 |
| `/custom_controller/target/joint_states` | 目标模型关节状态 |
| `/custom_controller/actual/joint_states` | 实际模型关节状态 |
| `/custom_controller/status_markers` | RViz 状态文字 |
| `/custom_controller/diagnostics` | 控制器和机械臂反馈在线诊断 |

`urdf_path:=...` 可以替换描述文件，目前需包含六个转动关节 `J1..J6`；默认网格资源由 `new_hj_arm2` 包安装。不能直接指定 Windows 文件路径到 Linux 小电脑。

## 验证

Windows 上可验证模型引用、目标材质、关节名称、无效轴保持、数据过期、反馈重排与误差计算：

```powershell
$env:PYTHONPATH = "$PWD/src/custom_controller_usb"
python -m unittest discover -s src/custom_controller_usb/test -p test_visualization_state.py -v
```

ROS2 小电脑上构建后观察画面和话题：

```bash
ros2 topic hz /custom_controller/target/joint_states
ros2 topic echo /custom_controller/diagnostics
ros2 run tf2_ros tf2_echo world target/Link6
```

默认关闭 RViz 的 `Controller Status` MarkerArray 显示，以免状态文字覆盖模型；诊断仍发布在 `/custom_controller/diagnostics`。需要文字调试时可启动并设置 `show_status_text:=true`。
