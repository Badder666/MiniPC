"""MCU 本地补偿：控制器显示、机械臂显示和 USB 遥操转发一起启动。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("custom_controller_usb"))
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_313F36753233-if00"),
        DeclareLaunchArgument("command_port", default_value="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_326433533034-if00"),
        DeclareLaunchArgument("mapping_config", default_value=str(share / "config/command_bridge.yaml")),
        DeclareLaunchArgument("controller_mapping_config", default_value=str(share / "config/controller_mapping_urdf2.yaml")),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/visualization.launch.py")),
            launch_arguments={
                "port": LaunchConfiguration("port"),
                "command_port": LaunchConfiguration("command_port"),
                "mapping_config": LaunchConfiguration("mapping_config"),
                "controller_mapping_config": LaunchConfiguration("controller_mapping_config"),
                "urdf_path": str(share / "config/controller_model_urdf2/controller_identified.urdf"),
                "forward": "true", "read_feedback": "true",
                "gravity_feedforward": "false",
                "show_trajectory": "false", "show_planned_trajectory": "false",
                "show_centers_of_mass": "false", "trajectory_output_csv": "",
            }.items())]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/arm_visualization.launch.py")),
            launch_arguments={"mapping_config": LaunchConfiguration("mapping_config")}.items())]),
    ])
