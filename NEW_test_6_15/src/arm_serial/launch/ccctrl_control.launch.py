#!/usr/bin/env python3
"""
仅启动 cctrl_receiver + ccctrl_joint_processor（不启动串口桥接）。

用法:
  ros2 launch arm_serial ccctrl_control.launch.py
  ros2 launch arm_serial ccctrl_control.launch.py receiver_port:=/dev/ttyUSB0
  ros2 launch arm_serial ccctrl_control.launch.py goto_duration:=2.0
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
        description="CCtrl 遥控器串口设备路径",
    )
    receiver_baudrate_arg = DeclareLaunchArgument(
        "receiver_baudrate",
        default_value="961200",
        description="receiver 波特率",
    )

    # ── ccctrl_joint_processor 参数 ────────────────────────────
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.4",
        description="关节角 EMA 低通滤波系数",
    )
    goto_duration_arg = DeclareLaunchArgument(
        "goto_duration",
        default_value="0.5",
        description="GOTO 每段轨迹时长 (秒)",
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
            {"filter_alpha": LaunchConfiguration("filter_alpha")},
            {"goto_duration": LaunchConfiguration("goto_duration")},
        ],
    )

    return LaunchDescription([
        receiver_port_arg,
        receiver_baudrate_arg,
        filter_alpha_arg,
        goto_duration_arg,
        ccctrl_receiver_node,
        ccctrl_joint_processor_node,
    ])
