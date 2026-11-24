from glob import glob
from setuptools import find_packages, setup

package_name = "custom_controller_usb"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        ("share/" + package_name + "/config/controller_model", ["config/controller_model/controller.urdf"]),
        ("share/" + package_name + "/config/controller_model/meshes",
         glob("config/controller_model/meshes/*.STL")),
    ],
    install_requires=["setuptools", "pyserial>=3.5,<4"],
    zip_safe=True,
    maintainer="badder",
    maintainer_email="Badder666@users.noreply.github.com",
    description="Receive Custom_Controller USB state frames and publish ROS2 data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "usb_receiver = custom_controller_usb.receiver_node:main",
        "command_bridge = custom_controller_usb.command_bridge:main",
        "legacy_input_adapter = custom_controller_usb.legacy_input_adapter:main",
        "visualization = custom_controller_usb.visualization_node:main",
        "gravity_torque = custom_controller_usb.gravity_model:main",
    ]},
)
