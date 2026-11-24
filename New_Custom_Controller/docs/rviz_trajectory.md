# 控制器实时姿态与轨迹范围

## 启动

关闭原来的 RViz 启动终端（Ctrl+C），在虚拟机终端执行：

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py port:=/dev/ttyACM0
```

默认读取 115200 波特率、25 通道 JustFloat。无需 CSV；默认不转发控制指令、不读取下位机反馈。
同一个串口只能由一个接收程序使用；VOFA+ 要关闭该串口连接，USB 设备应连接到虚拟机。

## 显示内容

- Controller Live Pose：v2 控制器模型，直接随 USB 编码器位置更新。
- Planned + Live Trajectories：青色线和点是 `safe_sampling_conservative.csv` 中通过碰撞初筛的规划采样路径；黄色线和点是本次启动后手动操作得到的实际末端轨迹。
- 蓝色框：本次会话末端参考点的 XYZ 最小/最大范围。只表示观测范围，不代表框内都是安全姿态。

规划路径默认文件为：

```text
/home/badder/minipc/trajectory/safe_sampling_conservative.csv
```

路径中的 `J1..J6` 按控制器 URDF 关节角解释，并通过同一套 URDF 正向运动学转换为末端轨迹。规划器使用了保守的 STL 初筛，青色路径用于手动验证，不代表已经完成实机碰撞认证。

也可以指定其他规划结果：

```bash
ros2 launch custom_controller_usb visualization.launch.py \
  planned_trajectory_csv:=/home/badder/minipc/trajectory/your_path.csv \
  show_planned_trajectory:=true
```

参考点是 URDF 最后一个旋转关节的原点（joint6/link6 原点），不是整个连杆的碰撞包络。仅转动 J6 时模型姿态改变，但该原点可能不移动。

轨迹要求六轴全部有效（I19=63）；无效反馈、接收间隔超过 0.25 秒或设备时间回退会断开连线。
默认最多保留 3000 个点，超过后均匀压缩；XYZ 范围始终覆盖整个会话。重启后清空。

## 参数与校准

```bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 protocol:=justfloat trajectory_max_points:=6000
```

`config/controller_mapping.yaml` 是控制器显示专用映射：

```text
q_urdf = scales * q_encoder + offsets
scales = [-1, 1, 1, 1, -1, 1]
offsets = [0, 0, 0, 0, 0, 0]
```

与控制器 URDF 重力参考模型保持一致；CAD 零姿态与实物编码器零点仍需逐轴校准。不要使用被控机械臂的 command_bridge.yaml 偏置作为控制器自身模型偏置。

`protocol:=state` 仅用于旧的 42 字节 CRC 状态帧固件。

## 排查

```bash
ros2 topic echo /custom_controller/state --once
ros2 topic hz /custom_controller/state
ros2 topic echo /custom_controller/target/joint_states --once
```

`state` 无数据时先检查 USB 映射、串口占用和协议；有数据时检查 `valid_motor_mask`，再检查 RViz 的 Controller Live Pose 图层。
