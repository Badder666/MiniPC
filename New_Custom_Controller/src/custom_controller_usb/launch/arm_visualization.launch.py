"""只订阅现有遥操话题，显示被控机械臂；不打开串口或修改零位。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from custom_controller_usb.visualization_state import read_arm_description, target_robot_description


def _launch_nodes(context):
    share = Path(get_package_share_directory("custom_controller_usb"))
    urdf_path = Path(LaunchConfiguration("urdf_path").perform(context))
    xml = urdf_path.read_text(encoding="utf-8")
    description = read_arm_description(xml)
    nodes = []
    for pose in ("actual", "target", "zero"):
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=f"teleop_arm/{pose}", name="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(
                target_robot_description(xml) if pose == "target" else xml, value_type=str),
                "frame_prefix": f"arm_{pose}/", "publish_frequency": 30.0}],
            output="screen"))
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"world_to_arm_{pose}",
            arguments=["--frame-id", "world", "--child-frame-id",
                       f"arm_{pose}/{description.root_link}"], output="screen"))
    nodes.append(Node(
        package="custom_controller_usb", executable="visualization",
        namespace="teleop_arm", name="visualization", output="screen",
        parameters=[LaunchConfiguration("mapping_config"), {
            "urdf_path": str(urdf_path), "enable_zero_calibration": False,
            "pose_frame_prefix": "arm_", "show_centers_of_mass": False,
            "show_trajectory": False, "show_planned_trajectory": False,
            "trajectory_output_csv": "", "zero_encoder_positions": [0.0] * 6,
            "reference_positions": [0.0] * 6,
        }],
        remappings=[("state", "/custom_controller/state"),
                    ("arm_joint_states", "/custom_controller/arm_joint_states")]))
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="arm_rviz", output="screen",
        arguments=["-d", str(share / "config" / "arm_teleop.rviz")]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("custom_controller_usb"))
    arm_share = Path(get_package_share_directory("new_hj_arm2"))
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=str(arm_share / "urdf/new_hj_arm2.urdf")),
        DeclareLaunchArgument("mapping_config", default_value=str(share / "config/command_bridge.yaml")),
        OpaqueFunction(function=_launch_nodes),
    ])
