# Pinocchio 重力力矩回传

上位机从控制器的同一个 USB CDC 端口读取 25 通道 JustFloat 状态帧。每收到一帧有效的六轴位置，就按 `controller_mapping.yaml` 的方向和零点换算成 URDF 关节角，再调用 Pinocchio 的 `computeGeneralizedGravity` 计算重力力矩。结果按电机顺序打包成 42 字节二进制帧，写回同一个 USB 端口。

## 安装 Pinocchio

Ubuntu 22.04 的 ROS2 Humble 环境使用系统包：

```bash
sudo apt update
sudo apt install ros-humble-pinocchio
```

安装后检查：

```bash
source /opt/ros/humble/setup.bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

## 先做 dry-run

`gravity_dry_run` 默认是 `true`，只打印计算结果，不向控制器发送力矩。确认力矩数量级和方向后，再改成 `false`：

```bash
source /opt/ros/humble/setup.bash
source /home/badder/minipc/New_Custom_Controller/install/setup.bash
ros2 launch custom_controller_usb usb_receiver.launch.py \
  protocol:=justfloat \
  port:=/dev/ttyACM0 \
  gravity_feedforward:=true \
  gravity_dry_run:=true \
  urdf_path:=/home/badder/minipc/v2/urdf/v2.urdf \
  torque_scale:=0.2
```

确认方向后发送真实力矩：

```bash
ros2 launch custom_controller_usb usb_receiver.launch.py \
  protocol:=justfloat \
  port:=/dev/ttyACM0 \
  forward:=false \
  gravity_feedforward:=true \
  gravity_dry_run:=false \
  urdf_path:=/home/badder/minipc/v2/urdf/v2.urdf \
  torque_scale:=0.2
```

`torque_scale` 是 Pinocchio 输出的统一比例，建议从 `0.1` 或 `0.2` 开始。上位机没有发送或帧超过约 250 ms 时，控制器只清零外部力矩前馈，保持原有电机控制任务运行；采样状态仍然强制前馈为零。

## 二进制帧

帧长度为 42 字节，采用小端格式：`​A5 5A`、版本 1、类型 2、长度、序号、时间戳、六轴有效位、启用标志、六个 `float32` 力矩和 CRC16。CRC16 与控制器已有实现相同，初值 `0xffff`，多项式 `0x8408`。

控制器的二进制解析和 `GRAV_CMD,...` 文本命令共用输入缓冲，但会在一次 USB 读取中分别处理，因此采样命令仍可使用。
