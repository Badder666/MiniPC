"""把六轴控制器状态和操作输入发送到机械臂下位机。"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from custom_controller_interfaces.msg import ControllerState, OperatorInput
from sensor_msgs.msg import JointState
import serial

from .command_protocol import ArmFeedbackParser, OperatorState, CommandFrame, build_command_frame
from .bridge_state import CommandSelector, PendingCommandWriter
from .joint_mapping import JointMapper


class ControllerCommandBridge(Node):
    def __init__(self, **kwargs):
        kwargs.setdefault("namespace", "custom_controller")
        super().__init__("command_bridge", **kwargs)
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("poll_period_s", 0.01)
        self.declare_parameter("forward_enabled", True)
        self.declare_parameter("axis_order", [0, 1, 2, 3, 4, 5])
        self.declare_parameter("scales", [1.0] * 6)
        self.declare_parameter("offsets", [0.0] * 6)
        self.declare_parameter("main_button_bits", [-1] * 8)
        self.declare_parameter("handle_button_bits", [-1] * 8)
        self.declare_parameter("arm_joint_names", [f"joint_{i + 1}" for i in range(6)])
        self._port_path = self.get_parameter("port").value
        self._baudrate = self.get_parameter("baudrate").value
        self._reconnect_period = self.get_parameter("reconnect_period_s").value
        self._forward_enabled = self.get_parameter("forward_enabled").value
        poll_period = self.get_parameter("poll_period_s").value
        self._joint_names = list(self.get_parameter("arm_joint_names").value)
        if not self._port_path or self._baudrate <= 0:
            raise ValueError("port must be nonempty and baudrate must be positive")
        if any(not math.isfinite(v) or v <= 0 for v in (poll_period, self._reconnect_period)):
            raise ValueError("poll and reconnect periods must be finite and positive")
        if len(self._joint_names) != 6 or len(set(self._joint_names)) != 6 or not all(self._joint_names):
            raise ValueError("arm_joint_names must contain six unique names")
        mapper = JointMapper(
            self.get_parameter("axis_order").value,
            self.get_parameter("scales").value,
            self.get_parameter("offsets").value,
        )
        self._selector = CommandSelector(mapper,
                                        self.get_parameter("main_button_bits").value,
                                        self.get_parameter("handle_button_bits").value)
        self._serial = None
        self._next_connect = 0.0
        self._operator = OperatorState()
        self._writer = PendingCommandWriter()
        self._feedback_parser = ArmFeedbackParser()
        qos = QoSProfile(depth=1)
        self._state_sub = self.create_subscription(ControllerState, "state", self._state_cb, qos)
        self._operator_sub = self.create_subscription(OperatorInput, "operator_input", self._operator_cb, qos)
        self._feedback_pub = self.create_publisher(JointState, "arm_joint_states", qos)
        self._timer = self.create_timer(poll_period, self._poll)

    def _operator_cb(self, message: OperatorInput):
        operator = OperatorState(
            message.joystick_x, message.joystick_y,
            (message.mouse_x, message.mouse_y, message.mouse_z),
            (message.mouse_left, message.mouse_right, message.mouse_mid),
            message.keys, message.main_buttons, message.handle_buttons,
        )
        try:
            build_command_frame(CommandFrame(0, 0, (0.0,) * 6, operator=operator))
        except (ValueError, OverflowError) as error:
            self.get_logger().warning(f"忽略无效操作输入: {error}")
            return
        self._operator = operator

    def _connect(self):
        if not self._port_path or self._serial is not None or time.monotonic() < self._next_connect:
            return
        self._next_connect = time.monotonic() + self._reconnect_period
        try:
            self._serial = serial.Serial(self._port_path, baudrate=self._baudrate,
                                         timeout=0, write_timeout=0.02, exclusive=True)
            self._feedback_parser = ArmFeedbackParser()
            self.get_logger().info(f"已连接下位机串口 {self._port_path}")
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"无法打开下位机串口 {self._port_path}: {error}")

    def _state_cb(self, state: ControllerState):
        if not self._forward_enabled:
            return
        try:
            frame = self._selector.encode(state.positions, state.valid_motor_mask,
                                          state.device_time_ms, state.buttons, self._operator)
        except (ValueError, OverflowError) as error:
            self.get_logger().warning(f"忽略无效控制器状态: {error}")
            return
        # A disconnected session must not replay stored targets after reconnect.
        if self._serial is not None:
            self._writer.submit(frame)

    def _disconnect(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None
        self._writer.clear()
        self._feedback_parser = ArmFeedbackParser()
        self._next_connect = time.monotonic() + self._reconnect_period

    def _poll(self):
        self._connect()
        if self._serial is None:
            return
        try:
            waiting = self._serial.in_waiting
            if waiting:
                data = self._serial.read(min(waiting, 4096))
                for positions in self._feedback_parser.feed(data):
                    message = JointState()
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.name = self._joint_names
                    message.position = list(positions)
                    self._feedback_pub.publish(message)
            self._writer.write_once(self._serial)
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"下位机串口断开: {error}")
            self._disconnect()

    def destroy_node(self):
        self._timer.cancel()
        self._disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControllerCommandBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
