#!/usr/bin/env python3
"""
arm_serial 包的 launch 文件，同时启动 cctrl_receiver 和 ccctrl_serial_bridge。

用法:
  ros2 launch arm_serial arm_serial.launch.py
  ros2 launch arm_serial arm_serial.launch.py receiver_port:=/dev/ttyUSB0
  ros2 launch arm_serial arm_serial.launch.py bridge_port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── cctrl_receiver 参数 ────────────────────────────────────
    receiver_port_arg = DeclareLaunchArgument(
        "receiver_port",
        default_value="/dev/ttyUSB0",
        description="CCtrl RS232 串口设备路径 (receiver)",
    )
    receiver_baudrate_arg = DeclareLaunchArgument(
        "receiver_baudrate",
        default_value="115200",
        description="receiver 波特率",
    )

    # ── ccctrl_serial_bridge 参数 ──────────────────────────────
    bridge_port_arg = DeclareLaunchArgument(
        "bridge_port",
        default_value="/dev/ttyACM0",
        description="串口桥接设备路径 (bridge)",
    )
    bridge_baudrate_arg = DeclareLaunchArgument(
        "bridge_baudrate",
        default_value="921600",
        description="bridge 波特率",
    )

    # ── cctrl_receiver 节点 ────────────────────────────────────
    ccctrl_receiver_node = Node(
        package="arm_serial",
        executable="ccctrl_receiver.py",
        name="ccctrl_receiver",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("receiver_port")},
            {"baudrate": LaunchConfiguration("receiver_baudrate")},
        ],
    )

    # ── ccctrl_joint_processor 节点 ────────────────────────────
    ccctrl_joint_processor_node = Node(
        package="arm_serial",
        executable="ccctrl_joint_processor.py",
        name="ccctrl_joint_processor",
        output="screen",
        parameters=[
            {"filter_alpha": 0.4},
            {"goto_duration": 3.0},
            {"goto_target_k1": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"goto_target_k2": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"goto_target_k3": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            {"goto_target_k4": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        ],
    )

    # ── ccctrl_serial_bridge 节点 ──────────────────────────────
    ccctrl_serial_bridge_node = Node(
        package="arm_serial",
        executable="ccctrl_serial_bridge.py",
        name="ccctrl_serial_bridge",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("bridge_port")},
            {"baudrate": LaunchConfiguration("bridge_baudrate")},
        ],
    )

    return LaunchDescription([
        receiver_port_arg,
        receiver_baudrate_arg,
        bridge_port_arg,
        bridge_baudrate_arg,
        ccctrl_receiver_node,
        ccctrl_joint_processor_node,
        ccctrl_serial_bridge_node,
    ])
