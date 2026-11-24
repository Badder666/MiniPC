#!/usr/bin/env python3
"""
Gazebo 重力补偿仿真 — new_hj_arm2

启动 (先安装 Gazebo 相关包):
  sudo apt-get install -y ros-humble-ros-gz-sim ros-humble-ign-ros2-control \
                          ros-humble-forward-command-controller ros-humble-effort-controllers
  cd ~/NEW_test_6_15 && colcon build --symlink-install
  source install/setup.bash
  ros2 launch new_hj_arm2_moveit_config gazebo_gravity.launch.py

验证:
  # 在终端中启用重力补偿（机械臂应保持姿态不下坠）
  ros2 service call /gravity_compensation/enable std_srvs/srv/SetBool "{data: true}"

  # 禁用重力补偿（机械臂应在重力作用下下坠）
  ros2 service call /gravity_compensation/enable std_srvs/srv/SetBool "{data: false}"
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_moveit = FindPackageShare("new_hj_arm2_moveit_config")

    # ── 机械臂 URDF (Gazebo 模式 + effort 接口) ──
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_moveit, "config", "new_hj_arm2.urdf.xacro"]),
        " use_gazebo:=true",
        " initial_positions_file:=",
        PathJoinSubstitution([pkg_moveit, "config", "initial_positions.yaml"]),
    ])

    robot_description = {"robot_description": robot_description_content}

    # ── Gazebo (Ignition Fortress) ──
    world_path = PathJoinSubstitution([pkg_moveit, "worlds", "empty.world"])
    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_path],
        output="screen",
    )

    # ── 机械臂状态发布器 ──
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ── 在 Gazebo 中生成机械臂 ──
    # 使用自定义 spawn 脚本（避免 ros-humble-ros-gz-sim 的依赖冲突）
    spawn_robot = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([pkg_moveit, "scripts", "spawn_robot_gz.py"]),
            PathJoinSubstitution([pkg_moveit, "config", "new_hj_arm2.urdf.xacro"]),
            "use_gazebo:=true",
            "initial_positions_file:=",
            PathJoinSubstitution([pkg_moveit, "config", "initial_positions.yaml"]),
        ],
        output="screen",
    )

    # ── ros2_control 控制器管理器 ──
    controller_config = PathJoinSubstitution([pkg_moveit, "config", "ros2_controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # ── 按顺序加载控制器 ──
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_pos_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_pos_spawner",
        arguments=["hj_arm_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gravity_effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="gravity_effort_spawner",
        arguments=["gravity_compensation_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── 重力补偿控制节点 ──
    gravity_comp_node = Node(
        package="arm_driver",
        executable="gravity_compensation_controller.py",
        name="gravity_compensation_controller",
        output="screen",
        parameters=[{"model_package": "new_hj_arm2"}],
    )

    return LaunchDescription([
        ign_gazebo,
        robot_state_publisher,
        # 延迟启动: Gazebo → spawn → controller_manager → 各控制器 → 重力补偿
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[control_node]),
        TimerAction(period=7.0, actions=[joint_state_spawner]),
        TimerAction(period=8.0, actions=[arm_pos_spawner]),
        TimerAction(period=9.0, actions=[gravity_effort_spawner]),
        TimerAction(period=10.0, actions=[gravity_comp_node]),
    ])
