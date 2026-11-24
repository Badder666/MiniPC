"""
启动串口桥接 + real_positions → /joint_states 转发 + RViz 可视化。

用法:
  ros2 launch arm_driver serial_rviz.launch.py
  ros2 launch arm_driver serial_rviz.launch.py port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── 参数 ──────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyACM0")
    baudrate_arg = DeclareLaunchArgument("baudrate", default_value="921600")

    # ── MoveIt 配置 (用于 robot_state_publisher + RViz) ────────
    moveit_config = (
        MoveItConfigsBuilder("new_hj_arm2", package_name="new_hj_arm2_moveit_config")
        .to_moveit_configs()
    )

    # ── robot_state_publisher (订阅 /joint_states → TF) ───────
    rsp_params = {
        "robot_description": moveit_config.robot_description["robot_description"],
    }
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[rsp_params],
        output="screen",
    )

    # ── 串口桥接节点 ─────────────────────────────────────────
    bridge_node = Node(
        package="arm_driver",
        executable="serial_dm_feedforward_bridge.py",
        name="arm_serial_feedforward_node",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baudrate": LaunchConfiguration("baudrate")},
        ],
        output="screen",
    )

    # ── 转发 real_positions → /joint_states ──────────────────
    relay_node = Node(
        package="arm_driver",
        executable="joint_state_relay.py",
        name="joint_state_relay",
        output="screen",
    )

    # ── RViz2 (需要 URDF + SRDF + kinematics) ────────────────
    rviz_config = os.path.join(
        get_package_share_directory("new_hj_arm2_moveit_config"),
        "config",
        "ee.rviz",
    )
    rviz_params = {
        "robot_description": moveit_config.robot_description["robot_description"],
        "robot_description_semantic": moveit_config.robot_description_semantic[
            "robot_description_semantic"
        ],
    }
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[rviz_params],
        output="screen",
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        rsp_node,
        bridge_node,
        relay_node,
        rviz_node,
    ])