"""
启动 CCtrl 串口接收 + robot_state_publisher + RViz 可视化。

用法:
  ros2 launch arm_serial ccctrl_rviz.launch.py
  ros2 launch arm_serial ccctrl_rviz.launch.py port:=/dev/ttyUSB0
  ros2 launch arm_serial ccctrl_rviz.launch.py baudrate:=115200
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 参数 ──────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0")
    baudrate_arg = DeclareLaunchArgument("baudrate", default_value="961200")

    # ── 直接读纯 URDF (避免 xacro/MoveIt 处理失败导致空描述) ──
    urdf_path = os.path.join(
        get_package_share_directory("new_hj_arm2"),
        "urdf",
        "new_hj_arm2.urdf",
    )
    with open(urdf_path, "r") as f:
        robot_description = {"robot_description": f.read()}

    # ── static_transform_publisher: world → base_link ─────────
    # 保证 TF 树至少有一条根变换，即使 robot_state_publisher 异常也能定位问题
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    # ── robot_state_publisher (订阅 /joint_states → TF) ───────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ── CCtrl 串口接收节点 (发布 /joint_states) ───────────────
    ccctrl_node = Node(
        package="arm_serial",
        executable="ccctrl_receiver.py",
        name="ccctrl_receiver",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baudrate": LaunchConfiguration("baudrate")},
        ],
        output="screen",
    )

    # ── RViz2 ─────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("new_hj_arm2_moveit_config"),
        "config",
        "ee.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
        output="screen",
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        static_tf,
        rsp_node,
        ccctrl_node,
        rviz_node,
    ])
