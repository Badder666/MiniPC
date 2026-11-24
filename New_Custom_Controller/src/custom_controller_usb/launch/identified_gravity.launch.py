"""Use the identified static COM candidate for both RViz and Pinocchio."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("custom_controller_usb"))
    model_dir = share / "config/controller_model_urdf2"
    defaults = {
        "port": "/dev/ttyACM0",
        "torque_scale": "1.0",
        "gravity_dry_run": "true",
        "show_centers_of_mass": "false",
        "trajectory_output_csv": "",
        "controller_mapping_config": str(share / "config/controller_mapping_urdf2.yaml"),
        "urdf_path": str(model_dir / "controller_identified.urdf"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/visualization.launch.py")),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in defaults},
                "protocol": "justfloat", "forward": "false",
                "gravity_feedforward": "true", "show_planned_trajectory": "false",
                "com_reference_urdf_path": str(model_dir / "controller.urdf"),
            }.items()),
    ])
