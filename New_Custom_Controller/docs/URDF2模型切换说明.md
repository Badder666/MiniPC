# URDF2 模型切换说明

已将 `MiniPC/URDF2/urdf/URDF2.urdf` 和七个 STL 网格接入 ROS2 上位机。
虚拟机工作目录：`/home/badder/minipc/New_Custom_Controller`。

## 模型及映射

- 运行模型：`src/custom_controller_usb/config/controller_model_urdf2/controller.urdf`。
- 原始导出模型：同目录的 `URDF2_original.urdf`，仅供对照。
- 公用映射：`src/custom_controller_usb/config/controller_mapping_urdf2.yaml`。
- `visualization.launch.py` 和 `usb_receiver.launch.py` 默认加载新模型及公用映射。
- 原模型仍保存在 `config/controller_model/`，修改前的启动文件和 setup.py 在 `backups/before_urdf2_*/`。

新 CAD 底座的坐标系与旧版不同。运行模型添加了 `controller_mount` 固定根节点，
使用 `rpy = [-1.5708, 0, -pi]` 对齐旧版竖直安装方向；Pinocchio 使用重力 `[0, 0, -9.81]`。
原始质量、质心、惯量和六个活动关节定义均保留，仅适配网格路径与安装坐标系。

六轴编码器到模型关节角的比例为 `[-1, -1, 1, -1, 1, -1]`，偏置暂为零。
2026-10-09：J2 电机角度设置已修正，RViz 和 Pinocchio 共用 `-1`，去掉原来的除4校正。电机设置改变后需重新确认零位。
新模型 J4 的物理转轴相对旧模型反向，因此 J4 的位置和力矩映射符号同时反转。
位置比例、偏置仍需结合实物零位核对，不能仅从 CAD 文件确定。

`torque_signs` 沿用原桥接约定，仅反转 J4。它不包含完整的电机/关节传动比力矩换算，
本次运行只计算结果，未验证新模型真实输出方向和补偿效果。

## 编译与启动

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_controller_usb --symlink-install
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 protocol:=justfloat forward:=false \
  gravity_feedforward:=true gravity_dry_run:=true torque_scale:=0.3 \
  show_planned_trajectory:=false \
  trajectory_output_csv:=/tmp/controller_urdf2_live.csv
```

`gravity_dry_run:=true` 表示用实时编码值计算力矩，不下发电机力矩。
确认姿态映射和输出力矩方向后，用同一命令的 `gravity_dry_run:=false` 启用力矩回传。
比例由启动参数 `torque_scale` 设置，修改后重新启动；当前接收节点不会动态应用参数变更。

本次后台启动的日志：`/tmp/controller_urdf2.log`，进程号：`/tmp/controller_urdf2.pid`。
查看和停止本次启动：

```bash
tail -f /tmp/controller_urdf2.log
# 在另一个终端停止，释放串口后再重新启动：
kill -INT "$(cat /tmp/controller_urdf2.pid)"
```

## 验证结果与范围

- 成功编译 `custom_controller_usb`。
- 模型 `nq=nv=6`，顺序为 `joint1` 到 `joint6`，网格路径全部有效。
- 100 组姿态：重力与势能数值梯度最大误差约 `5.34e-10 Nm`。
- 同一批姿态：Pinocchio 与可视化末端位置最大误差约 `3.89e-16 m`。
- J1 为竖直 yaw 轴，计算重力矩接近零。
- 可视化、轨迹及串口接收测试共 12 项通过。接收测试使用独立 `ROS_DOMAIN_ID=71`，避免混入实机话题。
- `/dev/ttyACM0` 的实时帧已驱动 Pinocchio 持续计算，运行参数查询确认已加载新模型和新映射。

模型六个关节的 `lower/upper/effort/velocity` 都是 CAD 导出的零占位值。
本次重力计算不使用这些限位，没有据此推断安全运动范围；旧模型规划轨迹在本次启动中不显示。
数值一致性检查不代表质量、质心和实物零位已经标定准确。
