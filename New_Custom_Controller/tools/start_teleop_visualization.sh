#!/usr/bin/env bash
# 桌面启动入口：保持终端显示串口和遥操日志，Ctrl+C 停止整个 launch。
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/custom_controller_teleop_${UID}.lock"
if ! flock -n 9; then
    echo "可视化和遥操已在运行，无需重复启动。"
    exit 0
fi
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
if ! running_nodes="$(timeout 10 ros2 node list --no-daemon --spin-time 2)"; then
    echo "无法检查 ROS2 节点，请检查 ROS 环境后重试。"
    exit 1
fi
if printf '%s\n' "$running_nodes" | grep -Eq '^/(custom_controller/(usb_receiver|command_bridge|visualization)|teleop_arm/visualization|controller_rviz|arm_rviz)$'; then
    echo "已有控制器或机械臂节点在运行。请先在其终端按 Ctrl+C 停止，再使用本快捷方式。"
    exit 0
fi
echo "启动控制器 RViz、机械臂 RViz 和 USB 遥操转发；重力补偿由 MCU 运行。"
echo "轨迹与质心显示已关闭，按 Ctrl+C 停止。"
exec ros2 launch custom_controller_usb teleop_visualization.launch.py \
    controller_mapping_config:="$workspace_dir/src/custom_controller_usb/config/controller_mapping_urdf2.yaml" \
    mapping_config:="$workspace_dir/src/custom_controller_usb/config/command_bridge.yaml" "$@"
