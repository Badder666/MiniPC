# RViz 零位快照

把控制器的当前六轴编码值记录为模型零位，自动计算并保存 `offsets`。运动比例、方向仍沿用当前配置。

## 操作

1. 将达妙板 USB 连接到 Ubuntu 虚拟机，确认灰色实时模型随实物运动。
2. 把实物摆到 **URDF/CAD 六轴关节角均为零时的姿态**，保持不动。记录任意姿态会把该姿态定义为模型零位，影响重力计算的正确性。
3. 在 RViz 工具栏选择 **Interact（交互）**。
4. 右击模型旁的绿色方块 `ZERO SNAPSHOT (right click)`，选择 **Capture current pose as ZERO**。
5. 成功后标签变成 `ZERO SAVED (right click to recapture)`。灰色实时模型与绿色零位参考模型应重合，再转动实物检查方向和姿态。

没有新鲜、有效的六轴反馈时会拒绝记录；终端日志显示原因。此操作仅校准上位机角度映射，不修改电机内部编码零点。

如果方块没有显示，检查左侧 `Zero Snapshot Button` 是否勾选，并展开确认
`Interactive Markers Namespace` 为 `/custom_controller/zero_calibration`。
ROS2 Humble 使用这个命名空间属性，不使用 `Update Topic` 配置交互标记。

## 保存内容和生效方式

按模型关节顺序计算：

```text
q[j] = scales[j] * motor_position[axis_order[j]] + offsets[j]
offsets[j] = -scales[j] * captured_motor_position[axis_order[j]]
```

J2 当前比例为 `-1.0`，记录时也使用这个比例。快照不修改比例或力矩方向。

当前配置文件：

```text
~/minipc/New_Custom_Controller/src/custom_controller_usb/config/controller_mapping_urdf2.yaml
```

自动更新 `offsets`、`zero_encoder_positions`，并在同目录的 `zero_snapshots/` 保存原 YAML 和 JSON 快照。可视化与接收节点立即同步偏置，Pinocchio 使用更新后的模型角度；重启后仍保留，不需要重新编译或烧录。

记录后清空 RViz 中已有的轨迹范围，因为旧轨迹使用旧的零位。正在记录的 CSV 不会自动分文件，需要分析轨迹时应重新开始一次记录，避免混用两套坐标。

当前启动的是校准显示，`gravity_feedforward:=false`。确认新的零位和实际姿态对应后，再按重力补偿运行说明启动补偿。

## 终端替代操作

摆好实物后，也可以在已加载工作空间环境的 Ubuntu 终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/minipc/New_Custom_Controller/install/setup.bash
ros2 service call /custom_controller/capture_zero std_srvs/srv/Trigger '{}'
```

返回 `success: true` 表示记录成功；`message` 显示偏置和快照路径。每次重新记录前会备份上一份配置。

## 当前校准显示的启动和停止

停止后台显示：

```bash
kill -INT "$(cat /tmp/controller_offset_calibration.pid)"
```

在虚拟机桌面终端重新启动：

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py \
  protocol:=justfloat port:=/dev/ttyACM0 forward:=false \
  gravity_feedforward:=false show_planned_trajectory:=false \
  trajectory_output_csv:=/tmp/controller_offset_calibration_live.csv
```

检查服务和当前偏置：

```bash
ros2 service list
ros2 param get /custom_controller/usb_receiver offsets
ros2 param get /custom_controller/visualization offsets
```

新增功能已编译，相关测试覆盖有效反馈记录、失效反馈拒绝、映射一致、文件持久化及保存失败时偏置回滚。
