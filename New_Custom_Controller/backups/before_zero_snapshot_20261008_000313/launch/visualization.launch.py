"""Controller USB + target/actual arm poses in RViz2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    share = Path(get_package_share_directory("custom_controller_usb"))
    urdf_path = Path(LaunchConfiguration("urdf_path").perform(context))
    text = urdf_path.read_text(encoding="utf-8")
    from custom_controller_usb.visualization_state import read_arm_description, target_robot_description
    description = read_arm_description(text)

    nodes = []
    for pose, xml in (("target", text), ("zero", target_robot_description(text))):
        namespace = f"custom_controller/{pose}"
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=namespace, name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": ParameterValue(xml, value_type=str),
                         "frame_prefix": f"{pose}/",
                         "publish_frequency": 30.0}],
        ))
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"world_to_{pose}", output="screen",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "world", "--child-frame-id", f"{pose}/{description.root_link}"],
        ))

    nodes.extend([
        Node(package="custom_controller_usb", executable="visualization",
             namespace="custom_controller", output="screen",
             parameters=[LaunchConfiguration("controller_mapping_config"),
                         {"urdf_path": str(urdf_path),
                          "show_status_text": ParameterValue(LaunchConfiguration("show_status_text"), value_type=bool),
                          "forward_enabled": ParameterValue(LaunchConfiguration("forward"), value_type=bool),
                          "show_trajectory": ParameterValue(LaunchConfiguration("show_trajectory"), value_type=bool),
                          "show_planned_trajectory": ParameterValue(LaunchConfiguration("show_planned_trajectory"), value_type=bool),
                          "planned_trajectory_csv": LaunchConfiguration("planned_trajectory_csv"),
                          "planned_trajectory_max_points": ParameterValue(LaunchConfiguration("planned_trajectory_max_points"), value_type=int),
                          "trajectory_max_points": ParameterValue(LaunchConfiguration("trajectory_max_points"), value_type=int),
                          "trajectory_output_csv": LaunchConfiguration("trajectory_output_csv")}]),
        Node(package="rviz2", executable="rviz2", name="controller_rviz", output="screen",
             arguments=["-d", str(share / "config" / "controller.rviz")]),
    ])
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("custom_controller_usb"))
    return LaunchDescription([
        DeclareLaunchArgument("controller_mapping_config", default_value=str(share / "config" / "controller_mapping_urdf2.yaml")),
        DeclareLaunchArgument("protocol", default_value="justfloat"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("command_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("forward", default_value="false"),
        DeclareLaunchArgument("read_feedback", default_value="false"),
        DeclareLaunchArgument("gravity_feedforward", default_value="false"),
        DeclareLaunchArgument("gravity_dry_run", default_value="true"),
        DeclareLaunchArgument("torque_scale", default_value="0.3"),
        DeclareLaunchArgument("gravity_send_period_s", default_value="0.01"),
        DeclareLaunchArgument("legacy_input", default_value="false"),
        DeclareLaunchArgument("show_status_text", default_value="false"),
        DeclareLaunchArgument("show_trajectory", default_value="true"),
        DeclareLaunchArgument("trajectory_max_points", default_value="3000"),
        DeclareLaunchArgument("show_planned_trajectory", default_value="true"),
        DeclareLaunchArgument("planned_trajectory_csv", default_value="/home/badder/minipc/trajectory/safe_sampling_conservative.csv"),
        DeclareLaunchArgument("planned_trajectory_max_points", default_value="3000"),
        DeclareLaunchArgument("trajectory_output_csv", default_value="/home/badder/minipc/trajectory/manual_controller.csv"),
        DeclareLaunchArgument("mapping_config", default_value=str(share / "config" / "command_bridge.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=str(share / "config" / "controller_model_urdf2" / "controller.urdf")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "usb_receiver.launch.py")),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ("controller_mapping_config", "port", "protocol", "command_port", "forward", "read_feedback", "legacy_input", "mapping_config", "gravity_feedforward", "gravity_dry_run", "urdf_path", "torque_scale", "gravity_send_period_s")}.items(),
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
