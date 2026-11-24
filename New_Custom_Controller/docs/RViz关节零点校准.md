# RViz 关节零点校准

RViz 同时显示两套控制器模型：

- `Controller Live Pose`：灰色实时姿态，角度为 `q = scale * encoder + offset`。
- `Controller Offset Zero Pose`：半透明绿色参考姿态，使用保存的 `zero_encoder_positions` 经过相同映射得到。未记录快照时六轴编码参考值默认为零，因此角度为 `q = offset`；记录快照后显示 CAD 六轴零姿态。没有 USB 数据时也会显示。

两套模型使用相同底座位置，方便叠加比较。可在 RViz 左侧 Displays 中取消对应显示项的勾选，
临时隐藏其中一套；也可调整绿色模型的 Alpha（透明度）。初始 offsets 全零时，绿色显示的是 CAD 模型零姿态。
参考模型仅用于显示，不发送电机目标或力矩。

## 修改偏置

在 Ubuntu 编辑实际运行的配置：

```bash
nano ~/minipc/New_Custom_Controller/src/custom_controller_usb/config/controller_mapping_urdf2.yaml
```

修改唯一的 `offsets` 字段，依次为 J1 到 J6，单位为模型关节的弧度：

```yaml
offsets: [0.0, 0.0, 0.174533, 0.0, 0.0, 0.0]
```

上述示例给 J3 增加 10 度，不是推荐标定值。常见换算为 `角度 * pi / 180`。
若摆到期望零位时编码值为 `p0`，希望此时模型关节角为零，则填写 `offset = -scale * p0`。
若希望匹配的 CAD 关节角为 `q0`，则填写 `offset = q0 - scale * p0`。
例如 J2 的 scale 为 -1，期望零位读到 encoder=1.2，则 offset=+1.2 rad。

未记录快照时，绿色参考表示“encoder=0 对应的模型姿态”。使用 [零位快照](RViz零位快照.md)
记录后，参考编码值更新为当时的实际值，新偏置使它对应 CAD 六轴零姿态。
校准依据应是实物摆到 CAD 零姿态后与灰色实时模型对照。

`offsets` 同时供可视化和 Pinocchio 使用；无需再填写 `position_offsets`。
旧接口 `position_offsets` 仍作为未提供 `offsets` 时的兼容默认值。
手动修改配置后须重启节点；通过快照菜单记录则立即生效。新偏置也会改变重力计算。此次编译使用 symlink-install，
修改已有 YAML 的数值无需重新编译，无需烧录达妙板。

## 启动校准显示

先用 Ctrl+C 停止当前前台程序。此次后台校准显示用以下命令停止：

```bash
kill -INT "$(cat /tmp/controller_offset_calibration.pid)"
```

在虚拟机桌面终端执行：

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py \
  protocol:=justfloat port:=/dev/ttyACM0 forward:=false \
  gravity_feedforward:=false show_planned_trajectory:=false \
  trajectory_output_csv:=/tmp/controller_offset_calibration_live.csv
```

该命令只接收状态和显示，不发送补偿力矩。USB 接入虚拟机后灰色模型显示实物当前姿态。
重新启用补偿时，使用重力补偿运行说明中的 `gravity_feedforward:=true gravity_dry_run:=false`
及所需 `torque_scale` 参数；应先确认改完偏置后的姿态符合实物。

## 编译与检查

本次代码修改已经编译。以后修改 Python 代码可执行：

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_controller_usb --symlink-install
source install/setup.bash
```

检查生效的共同偏置及参考关节状态：

```bash
ros2 param get /custom_controller/visualization offsets
ros2 param get /custom_controller/usb_receiver offsets
ros2 topic echo /custom_controller/zero/joint_states --once
```

显示使用独立的 `zero/` TF 前缀和 `/custom_controller/zero/robot_description`，
与实时 `target/` 模型分开。验证覆盖无 USB 时参考状态发布、非零偏置下实时映射、
Pinocchio 共用偏置以及旧 `position_offsets` 默认值兼容。
