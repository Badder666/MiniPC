#!/usr/bin/env python3
"""
一键启动 ccctrl 全部三个节点: receiver + joint_processor + serial_bridge。

用法:
  ros2 launch arm_serial ccctrl_all.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 cctrl_receiver、ccctrl_joint_processor、ccctrl_serial_bridge。"""

    ccctrl_receiver_node = Node(
        package="arm_serial",
        executable="ccctrl_receiver.py",
        name="ccctrl_receiver",
        output="screen",
    )

    ccctrl_joint_processor_node = Node(
        package="arm_serial",
        executable="ccctrl_joint_processor.py",
        name="ccctrl_joint_processor",
        output="screen",
    )

    ccctrl_serial_bridge_node = Node(
        package="arm_serial",
        executable="ccctrl_serial_bridge.py",
        name="ccctrl_serial_bridge",
        output="screen",
    )

    return LaunchDescription([
        ccctrl_receiver_node,
        ccctrl_joint_processor_node,
        ccctrl_serial_bridge_node,
    ])
