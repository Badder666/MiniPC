"""Nonblocking USB CDC reader with automatic reconnect and ROS2 publishing."""

import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from custom_controller_interfaces.msg import ControllerState
import serial
from ament_index_python.packages import (PackageNotFoundError,
                                          get_package_share_directory)

from .protocol import AXIS_COUNT, StateFrameParser
from .protocol import build_host_torque_frame
from .justfloat import JustFloatParser
from .pinocchio_gravity import PinocchioGravityModel


class ControllerUSBReceiver(Node):
    def __init__(self, **kwargs):
        kwargs.setdefault("namespace", "custom_controller")
        super().__init__("usb_receiver", **kwargs)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("protocol", "state")
        protocol = self.get_parameter("protocol").value
        if protocol not in ("state", "justfloat"):
            raise ValueError("protocol must be state or justfloat")
        self._parser_type = JustFloatParser if protocol == "justfloat" else StateFrameParser
        self.declare_parameter("poll_period_s", 0.01)
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("joint_names", [f"motor_{i + 1}" for i in range(AXIS_COUNT)])
        self.declare_parameter("gravity_feedforward", False)
        self.declare_parameter("gravity_dry_run", True)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("gravity", [0.0, 0.0, -9.81])
        self.declare_parameter("axis_order", [0, 1, 2, 3, 4, 5])
        self.declare_parameter("position_scales", [-1.0, -1.0, 1.0, 1.0, 1.0, -1.0])
        self.declare_parameter("position_offsets", [0.0] * AXIS_COUNT)
        # Share visualization's offsets; retain position_offsets as a legacy default.
        self.declare_parameter("offsets", self.get_parameter("position_offsets").value)
        self.declare_parameter("torque_signs", [1.0] * AXIS_COUNT)
        self.declare_parameter("torque_scale", 0.3)
        self.declare_parameter("gravity_send_period_s", 0.01)
        self._port_path = self.get_parameter("port").value
        self._baudrate = self.get_parameter("baudrate").value
        poll_period = self.get_parameter("poll_period_s").value
        self._reconnect_period = self.get_parameter("reconnect_period_s").value
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._gravity_enabled = bool(self.get_parameter("gravity_feedforward").value)
        self._gravity_dry_run = bool(self.get_parameter("gravity_dry_run").value)
        self._gravity_period = float(self.get_parameter("gravity_send_period_s").value)
        self._gravity_sequence = 0
        self._last_gravity_send = 0.0
        self._gravity_model = None
        self._axis_order = tuple(int(v) for v in self.get_parameter("axis_order").value)
        self._position_scales = tuple(float(v) for v in self.get_parameter("position_scales").value)
        self._position_offsets = tuple(float(v) for v in self.get_parameter("offsets").value)
        self._torque_signs = tuple(float(v) for v in self.get_parameter("torque_signs").value)
        self._torque_scale = float(self.get_parameter("torque_scale").value)
        if not self._port_path or self._baudrate <= 0:
            raise ValueError("port must be nonempty and baudrate must be positive")
        if (not math.isfinite(poll_period) or poll_period <= 0 or
                not math.isfinite(self._reconnect_period) or self._reconnect_period <= 0):
            raise ValueError("poll and reconnect periods must be finite and positive")
        if (not math.isfinite(self._gravity_period) or self._gravity_period <= 0 or
                len(self._axis_order) != AXIS_COUNT or
                len(self._position_scales) != AXIS_COUNT or
                len(self._position_offsets) != AXIS_COUNT or
                len(self._torque_signs) != AXIS_COUNT):
            raise ValueError("invalid gravity bridge parameters")
        if (len(self._joint_names) != AXIS_COUNT or
                len(set(self._joint_names)) != AXIS_COUNT or
                any(not name for name in self._joint_names)):
            raise ValueError("joint_names must contain six distinct nonempty names")

        qos = QoSProfile(depth=10)
        self._state_pub = self.create_publisher(ControllerState, "state", qos)
        self._joint_pub = self.create_publisher(JointState, "joint_states", qos)
        self._serial = None
        self._parser = self._parser_type()
        if self._gravity_enabled:
            urdf_path = str(self.get_parameter("urdf_path").value)
            if not urdf_path:
                try:
                    urdf_path = str(Path(get_package_share_directory("custom_controller_usb")) /
                                    "config" / "controller_model" / "controller.urdf")
                except (PackageNotFoundError, OSError):
                    urdf_path = str(Path(__file__).resolve().parents[1] /
                                    "config" / "controller_model" / "controller.urdf")
            try:
                self._gravity_model = PinocchioGravityModel(
                    urdf_path, self.get_parameter("gravity").value)
                self.get_logger().info(
                    f"Pinocchio gravity feedforward enabled: {urdf_path}, "
                    f"scale={self._torque_scale:g}, dry_run={self._gravity_dry_run}")
            except (OSError, RuntimeError, ValueError) as error:
                self._gravity_enabled = False
                self.get_logger().error(f"Gravity feedforward disabled: {error}")
        self._next_connect = 0.0
        self._timer = self.create_timer(poll_period, self._poll)

    def _connect(self):
        if time.monotonic() < self._next_connect:
            return
        self._next_connect = time.monotonic() + self._reconnect_period
        try:
            self._serial = serial.Serial(
                self._port_path, baudrate=self._baudrate,
                timeout=0, exclusive=True,
            )
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"Cannot open {self._port_path}: {error}")
            return
        self._parser = self._parser_type()
        self.get_logger().info(f"Connected to {self._port_path}")

    def _disconnect(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None
        # Never join a fragment from an old device session with a new frame.
        self._parser = self._parser_type()
        self._next_connect = time.monotonic() + self._reconnect_period

    def _send_gravity_torque(self, frame):
        if not self._gravity_enabled or self._gravity_model is None:
            return
        now = time.monotonic()
        if now - self._last_gravity_send < self._gravity_period:
            return
        self._last_gravity_send = now
        try:
            torques, valid_mask = self._gravity_model.torque_from_motor_positions(
                frame.positions, self._axis_order, self._position_scales,
                self._position_offsets, self._torque_signs, self._torque_scale,
                frame.valid_motor_mask)
            packet = build_host_torque_frame(
                int(now * 1000.0), torques, valid_mask,
                enabled=True, sequence=self._gravity_sequence)
            self._gravity_sequence = (self._gravity_sequence + 1) & 0xFFFF
            if self._gravity_dry_run:
                if self._gravity_sequence % 100 == 1:
                    self.get_logger().info("gravity dry-run torque[Nm]=" +
                                           ", ".join(f"{value:.4f}" for value in torques))
            else:
                self._serial.write(packet)
        except (OSError, serial.SerialException, ValueError) as error:
            self.get_logger().warning(f"Gravity torque send failed: {error}")
            self._disconnect()

    def _poll(self):
        if self._serial is None:
            self._connect()
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if not waiting:
                return
            chunk = self._serial.read(min(waiting, 4096))
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"Disconnected from {self._port_path}: {error}")
            self._disconnect()
            return
        for frame in self._parser.feed(chunk):
            stamp = self.get_clock().now().to_msg()
            state = ControllerState()
            state.stamp = stamp
            state.sequence = frame.sequence
            state.device_time_ms = frame.timestamp_ms
            state.positions = list(frame.positions)
            state.buttons = frame.buttons
            state.valid_motor_mask = frame.valid_motor_mask
            self._state_pub.publish(state)

            joints = JointState()
            joints.header.stamp = stamp
            joints.name = self._joint_names
            # Prevent generic JointState consumers from using stale positions.
            joints.position = [
                position if frame.valid_motor_mask & (1 << i) else math.nan
                for i, position in enumerate(frame.positions)
            ]
            self._joint_pub.publish(joints)
            self._send_gravity_torque(frame)

    def destroy_node(self):
        self._timer.cancel()
        self._disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControllerUSBReceiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
