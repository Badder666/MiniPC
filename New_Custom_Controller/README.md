# 六轴自定义控制器 ROS2 工作区

## RViz2 可视化

已加入 `new_hj_arm2` 的 URDF 和网格资源。重新构建后，可同时显示绿色半透明目标模型、实际反馈模型，以及六轴数值、有效位、按键和通信状态：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_controller_usb visualization.launch.py \
  port:=/dev/ttyACM0 command_port:=/dev/ttyACM1
```

可视化入口默认只观察，不下发命令；实际控制加 `forward:=true`。未接机械臂时加 `read_feedback:=false`。显示与下发使用相同的映射配置，详见 [docs/rviz_visualization.md](docs/rviz_visualization.md)。

## URDF 静态重力力矩估算

自定义控制器的 `v2/urdf/v2.urdf` 含六个关节的质量和质心数据，可以在上位机先估算静态重力项。该工具按 URDF 链顺序读取 `jonit1`、`joint2` 至 `joint6`，输出六个关节的重力力矩；补偿时应使用输出值的相反数：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run custom_controller_usb gravity_torque \
  --urdf /home/badder/minipc/v2/urdf/v2.urdf \
  --q 0 0 0 0 0 0
```

`v2.urdf` 的全零关节姿态是折叠姿态；上面的输入是自定义控制器电机位置，不是可视化机械臂的关节映射值。可视化使用的 `new_hj_arm2` 仍可用 `[0, 2.4, -2.4, 0, 0, 0]` 表示控制器全零对应的机械臂姿态。当前工具只计算和打印 N*m 结果，不会下发力矩。`v2.urdf` 的关节限位字段是导出器生成的全零占位值，不能用于安全限位；正式使用前需要根据实际零位、方向、减速比和实测静态力矩标定。算法说明见 [docs/gravity_torque_estimation.md](docs/gravity_torque_estimation.md)。

## 新增下发链路

```text
六个 DM4310 / RC8P → 自定义控制器 USB JustFloat 状态帧（104 字节）
           → usb_receiver → /custom_controller/state
           → command_bridge：轴序、比例、零位和按键映射
           → USB 命令帧（64 字节）→ Engineer_2026_H7_Template_new
机械臂反馈 ← /custom_controller/arm_joint_states ← 下位机 27 字节反馈
```

将本目录源码同步到小电脑后重新构建：

```bash
cd /home/badder/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_controller_usb usb_receiver.launch.py \
  port:=/dev/ttyACM0 command_port:=/dev/ttyACM1
```

`port` 是自定义控制器，`command_port` 是机械臂下位机，必须是两个不同设备。建议使用各自的 `/dev/serial/by-id/...`。启动文件默认启用下发；只采集时加 `forward:=false`。

映射配置在 `src/custom_controller_usb/config/command_bridge.yaml`，也可用 `mapping_config:=/绝对路径/command_bridge.yaml` 覆盖。默认电机 1..6 对应 J1..J6，比例为 `[-1, -1, +1, -1, -1, +1]`；零输入偏置为 `[0, 2.4, -2.4, 0, 0, 0]`，使控制器全零对应机械臂折叠姿态。公式为 `scales[j] × positions[axis_order[j]] + offsets[j]`。无效轴保留下位机上一目标。下位机沿用已有的关节到电机换算及斜坡限速。

完整 16 位原始按键下发到 `MiniPC_Data.controller_buttons`。按键接线未确定，`main_button_bits` 与 `handle_button_bits` 的源位默认全部为 `-1`，表示禁用动作映射。接线确定后将源位设为 0..15，例如把 `handle_button_bits[3]` 设为 11，可让控制器 bit 11 对应原有下位机 `btn4` 夹爪动作。外部操作输入的面板字段仍可使用。

键鼠/摇杆程序发布 `/custom_controller/operator_input`，消息为 `custom_controller_interfaces/msg/OperatorInput`。摇杆范围 0..4095、中位 2048。显式发布该消息时需要设置摇杆中位；未收到操作输入时桥接使用中位和释放状态。操作输入随新的控制器状态一起下发。

沿用旧 `/tc_info`、`/ccctrl_positions` 操作输入时，启用可选适配节点：

```bash
source /opt/ros/humble/setup.bash
source /home/badder/minipc/NEW_test_6_15/install/setup.bash
source /home/badder/minipc/New_Custom_Controller/install/setup.bash
ros2 launch custom_controller_usb usb_receiver.launch.py \
  port:=/dev/ttyACM0 command_port:=/dev/ttyACM1 legacy_input:=true
```

适配只读取旧键鼠、摇杆和面板字段，旧关节位置不会覆盖新电机的位置。旧桥接程序不能同时占用机械臂串口。

两个节点断开时丢弃半帧与待发送数据，每秒尝试重连；桥接重新连接后等待新控制器状态，不重放旧目标。没有新增 500 ms 无帧停机逻辑。

Windows 上不依赖 ROS2 的测试：

```powershell
$env:PYTHONPATH = "$PWD/src/custom_controller_usb"
python -m unittest discover -s src/custom_controller_usb/test -p test_protocol.py -v
python -m unittest discover -s src/custom_controller_usb/test -p test_command_protocol.py -v
```

跨语言测试需要同级下位机源码及主机 `gcc`，涵盖 Python 发送与 C 接收、有效轴、按键映射、短写、损坏恢复、旧帧兼容及缓冲溢出。本次 Windows 测试和固件构建已通过；新增 ROS2 节点和机械臂动作仍需小电脑联调。协议见 [docs/usb_command_protocol.md](docs/usb_command_protocol.md)。

## 状态接收说明

The Linux workspace is `/home/badder/minipc/New_Custom_Controller`. The source
is also retained in this Windows directory. The `src` directory contains:

- `custom_controller_interfaces`: the `ControllerState` message definition.
- `custom_controller_usb`: the USB CDC receiver and launch file.

Build and start on Ubuntu 22.04 with ROS2 Humble:

```bash
cd /home/badder/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_controller_usb usb_receiver.launch.py port:=/dev/ttyACM0 forward:=false
```

Prefer a stable `/dev/serial/by-id/...` path when available. Pass its exact path
as `port:=...`. The node retries opening the port every second, resets partial
frames on disconnect, and reads without blocking the ROS2 executor. USB CDC
baudrate does not control the USB data transfer speed.

In another terminal, source both ROS environments and inspect the topics:

```bash
source /opt/ros/humble/setup.bash
source /home/badder/minipc/New_Custom_Controller/install/setup.bash
ros2 topic echo /custom_controller/state
ros2 topic hz /custom_controller/joint_states
```

`/custom_controller/state` (`custom_controller_interfaces/msg/ControllerState`)
contains sequence, device time, all six raw positions, buttons and the valid
motor bitmap in one message. `stamp` records the ROS receive time.
`/custom_controller/joint_states` (`sensor_msgs/msg/JointState`) uses motor names
`motor_1` through `motor_6`, and sets invalid positions to NaN. Consumers must
check the valid bitmap before mapping raw positions to the controlled arm.
Topics use reliable QoS with depth 10; the node publishes only valid CRC frames.
The separate command_bridge node handles arm mapping and command output.
No 500 ms motor shutdown behavior is added.
If reports stop, the receiver publishes no replacement measurements.

Node parameters are `port`, `baudrate` (115200), `poll_period_s` (0.01),
`reconnect_period_s` (1.0), and `joint_names` (six unique motor names).
To override parameters directly:

```bash
ros2 run custom_controller_usb usb_receiver --ros-args -p port:=/dev/ttyACM0
```

If a real USB device is unavailable, enable its connection to Ubuntu in VMware.
Check port permissions with `ls -l /dev/ttyACM0` and `groups`. Typical Ubuntu
devices need membership in `dialout`; adding it requires logout/login:

```bash
sudo usermod -aG dialout badder
```

Run automated tests without a motor board:

```bash
cd /home/badder/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover protocol bytes/CRC, arbitrary splitting, corrupt-frame recovery,
ROS2 publication, a nominal 100 Hz pseudoterminal stream, and unplug/reconnect.
They do not certify physical USB throughput or mechanical arm behavior.

The earlier receiver-only version was verified on the Ubuntu 22.04 / ROS2 Humble VM on 2026-10-01:
`colcon build --symlink-install` built both packages, and `colcon test-result
--verbose` reported 9 tests, 0 errors, 0 failures, 0 skipped. Physical USB
and motor-board testing is pending.

## USB state protocol

The controller sends a 42-byte little-endian binary frame every 10 ms.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | `A5 5A` start marker |
| 2 | 1 | version, `1` |
| 3 | 1 | type, `1` for state |
| 4 | 2 | frame length, `42` |
| 6 | 2 | sequence number |
| 8 | 4 | controller uptime in milliseconds |
| 12 | 24 | six little-endian IEEE-754 `float32` positions in radians |
| 36 | 2 | 16-bit button bitmap, bit 0 is button 0 |
| 38 | 2 | valid motor bitmap, bits 0-5 are motors 1-6 |
| 40 | 2 | CRC16, low byte first |

CRC uses initial value `0xffff` and reflected polynomial `0x8408`.
`StateFrameParser` accepts partial USB reads, multiple frames, and corrupted
frames. See `usb_report_protocol.py` for a dependency-free implementation.

CRC is CRC-16/MCRF4XX: reflected input/output, final xor zero, check value
`0x6f91` for ASCII `123456789`. It covers bytes 0-39 of the frame.

Valid motor bits require CAN feedback within 250 ms. A cleared bit means the
position cannot be used as current feedback. The timestamp is the controller's
position snapshot time; the button bitmap is read at report construction.
Sequence numbers increment on report attempts, so USB busy drops leave gaps.
Sequence and millisecond timestamp counters wrap modulo 2^16 and 2^32.

Install and run the USB receiver (replace COM8 with the actual controller port):

```powershell
python -m pip install -r requirements.txt
python receive_usb.py COM8
```

On Linux, use the relevant `/dev/ttyACM0` port instead. Each valid frame is
printed as one JSON object. This standalone script handles USB reports only;
use the ROS2 command_bridge for mapping and forwarding to the arm.

Run parser tests without serial hardware or pyserial:

```powershell
python -m unittest -v test_usb_report_protocol
```
