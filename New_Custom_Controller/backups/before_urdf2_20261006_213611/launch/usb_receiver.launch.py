from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mapping_config = os.path.join(get_package_share_directory("custom_controller_usb"),
                                  "config", "command_bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("protocol", default_value="justfloat"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("command_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("mapping_config", default_value=mapping_config),
        DeclareLaunchArgument("legacy_input", default_value="false"),
        DeclareLaunchArgument("forward", default_value="true"),
        DeclareLaunchArgument("read_feedback", default_value="false"),
        DeclareLaunchArgument("gravity_feedforward", default_value="false"),
        DeclareLaunchArgument("gravity_dry_run", default_value="true"),
        DeclareLaunchArgument("urdf_path", default_value=os.path.join(
            get_package_share_directory("custom_controller_usb"),
            "config", "controller_model", "controller.urdf")),
        DeclareLaunchArgument("torque_scale", default_value="0.3"),
        DeclareLaunchArgument("gravity_send_period_s", default_value="0.01"),
        Node(
            package="custom_controller_usb", executable="usb_receiver",
            namespace="custom_controller", output="screen",
            parameters=[{"port": LaunchConfiguration("port"),
                         "protocol": LaunchConfiguration("protocol"),
                         "gravity_feedforward": ParameterValue(LaunchConfiguration("gravity_feedforward"), value_type=bool),
                         "gravity_dry_run": ParameterValue(LaunchConfiguration("gravity_dry_run"), value_type=bool),
                         "urdf_path": LaunchConfiguration("urdf_path"),
                         "torque_scale": ParameterValue(LaunchConfiguration("torque_scale"), value_type=float),
                         "gravity_send_period_s": ParameterValue(LaunchConfiguration("gravity_send_period_s"), value_type=float)}],
        ),
        Node(
            package="custom_controller_usb", executable="command_bridge",
            namespace="custom_controller", output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("forward"), "'.lower() in ('true', '1') or '",
                LaunchConfiguration("read_feedback"), "'.lower() in ('true', '1')",
            ])),
            parameters=[LaunchConfiguration("mapping_config"),
                        {"port": LaunchConfiguration("command_port"),
                         "forward_enabled": ParameterValue(LaunchConfiguration("forward"), value_type=bool)}],
        ),
        Node(
            package="custom_controller_usb", executable="legacy_input_adapter",
            namespace="custom_controller", output="screen",
            condition=IfCondition(LaunchConfiguration("legacy_input")),
        ),
    ])
