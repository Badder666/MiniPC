# vision_test — 机械臂视觉抓取工程

## 环境要求

- **Ubuntu** 22.04
- **ROS 2** Humble
- **Python** 3.10

## 1. ROS 2 Humble 安装

```bash
# 添加 ROS 2 仓库
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 2. 依赖包安装

### 2.1 ROS 2 包

```bash
# MoveIt2 相关
sudo apt install -y ros-humble-moveit ros-humble-moveit-setup-assistant ros-humble-moveit-configs-utils

# ros2_control 相关
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager

# Joint State Broadcaster
sudo apt install -y ros-humble-joint-state-broadcaster

# TF2
sudo apt install -y ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# robot_state_publisher / joint_state_publisher
sudo apt install -y ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui

# RViz2
sudo apt install -y ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins

# URDF / Xacro
sudo apt install -y ros-humble-xacro ros-humble-urdf

# control_msgs (FollowJointTrajectory action)
sudo apt install -y ros-humble-control-msgs

# cv_bridge (OpenCV ↔ ROS 图像转换)
sudo apt install -y ros-humble-cv-bridge

# camera_info_manager
sudo apt install -y ros-humble-camera-info-manager

# warehouse (MoveIt 场景持久化)
sudo apt install -y ros-humble-warehouse-ros-mongo
```

### 2.2 Python (pip) 包

```bash
pip3 install --user pyserial opencv-python opencv-contrib-python numpy
```

### 2.3 系统包

```bash
sudo apt install -y python3-pip python3-colcon-common-extensions
```

## 3. 编译工程

```bash
cd /home/hanjiang/vision_test

# 安装依赖 (从源码包)
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

## 4. 硬件驱动

### 4.1 串口权限 (机械臂)

```bash
# 将用户加入 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录生效

# 或临时授权
sudo chmod 666 /dev/ttyACM0
```

### 4.2 MindVision 相机 (可选)

相机 SDK (`libMVSDK.so`) 位于 `src/` 目录，需设置环境变量：

```bash
export MVSDK_SO_DIR=/home/hanjiang/vision_test/src
export LD_LIBRARY_PATH=$MVSDK_SO_DIR:$LD_LIBRARY_PATH
```

## 5. 目录结构

```
vision_test/
├── src/
│   ├── serial_dm_bridge.py         # 串口桥接节点 (FollowJointTrajectory action server)
│   ├── serial_ramp_test.py         # 串口斜坡测试 (单关节匀速)
│   ├── serial_sine_test.py         # 串口正弦测试
│   ├── serial_send_monitor.py      # 串口发送曲线监测
│   ├── hand_eye_calibrate.py       # Eye-in-Hand 手眼标定
│   ├── capture_mindvision.py       # MindVision 相机抓图
│   ├── hj_arm/                     # URDF 机器人描述包
│   ├── hj_arm_moveit_config/       # MoveIt2 配置 (含 pick_ik)
│   ├── hjarm_moveit_config/        # MoveIt2 配置 (标准)
│   ├── vision_capture/             # MindVision 相机 ROS2 驱动包
│   └── energy_3d/                  # YOLO 关键点检测 + PnP 6D 位姿估计
│       ├── onnxruntime/            # ONNX Runtime 1.16.3 (随包分发)
│       ├── models/                 # YOLO 模型文件 (best.onnx / best.pt)
│       └── include/energy_3d/      # C++ 头文件
└── README.md
```

## 6. 运行

### 6.1 启动串口桥接节点

```bash
cd /home/hanjiang/vision_test
source install/setup.bash
python3 src/serial_dm_bridge.py
```

或带参数：

```bash
python3 src/serial_dm_bridge.py --ros-args \
    -p port:=/dev/ttyACM0 \
    -p baudrate:=921600 \
    -p send_rate:=10.0 \
    -p action_name:=/hjarm_group_controller/follow_joint_trajectory
```

### 6.2 启动 MoveIt2

```bash
source install/setup.bash
ros2 launch hjarm_moveit_config demo.launch.py
```

### 6.3 手眼标定

```bash
python3 src/hand_eye_calibrate.py --marker-size 0.05
```

### 6.4 相机抓图

```bash
python3 src/capture_mindvision.py              # 单帧
python3 src/capture_mindvision.py --mode stream # 实时预览
```

### 6.5 YOLO 位姿检测 (energy_3d)

启动前设置模型路径（包内默认 `models/best.onnx`，从包目录运行即可）：

```bash
cd /home/hanjiang/vision_test
source install/setup.bash

# 启动 YOLO + PnP 检测节点（无头模式）
ros2 run energy_3d image_sub

# 带 GUI 调试窗口（需要 X11 显示器）
ros2 run energy_3d image_sub --ros-args -p show_gui:=true

# 完整参数
ros2 run energy_3d image_sub --ros-args \
    -p topic_image:=/camera/image_raw \
    -p topic_pose:=detected_pose \
    -p frame_id:=camera_optical_frame \
    -p show_gui:=false
```

**发布话题：**
- `detected_pose` (`geometry_msgs/PoseStamped`) — 检测到的目标在相机坐标系下的 6D 位姿

**环境变量：**
- `YOLO_MODEL_PATH` — 覆盖默认模型路径

## 7. 串口协议

| 字节 | 内容 |
|------|------|
| 0 | `0xAA` 帧头 |
| 1–24 | 6 × float32 (IEEE 754, 小端) 关节角度 [rad] |
| 25 | 校验和 (sum of bytes 1–24) & `0xFF` |
| 26 | `0x55` 帧尾 |

共 27 字节。
