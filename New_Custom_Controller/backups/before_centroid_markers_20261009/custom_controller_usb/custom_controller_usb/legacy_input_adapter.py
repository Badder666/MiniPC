"""Adapt the existing arm_serial keyboard, mouse and joystick topics."""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from custom_controller_interfaces.msg import OperatorInput

from .command_protocol import KEY_NAMES


class LegacyInputAdapter(Node):
    def __init__(self):
        super().__init__("legacy_input_adapter", namespace="custom_controller")
        # Only this optional node requires the old workspace's message package.
        from arm_serial.msg import TcInfo, CCtrlJoint

        self._state = OperatorInput()
        self._state.joystick_x = 2048
        self._state.joystick_y = 2048
        self._pub = self.create_publisher(OperatorInput, "operator_input", 1)
        self._tc_sub = self.create_subscription(TcInfo, "/tc_info", self._tc_cb, 1)
        self._joystick_sub = self.create_subscription(CCtrlJoint, "/ccctrl_positions", self._joystick_cb, 1)

    def _tc_cb(self, message):
        for name in ("mouse_x", "mouse_y", "mouse_z", "mouse_left", "mouse_right", "mouse_mid"):
            setattr(self._state, name, getattr(message, name))
        self._state.keys = sum(int(bool(getattr(message, "key_" + name, 0))) << bit
                               for bit, name in enumerate(KEY_NAMES))
        self._pub.publish(self._state)

    def _joystick_cb(self, message):
        self._state.joystick_x = message.joystick_x
        self._state.joystick_y = message.joystick_y
        self._state.main_buttons = message.main_buttons
        self._state.handle_buttons = message.handle_buttons
        self._pub.publish(self._state)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LegacyInputAdapter()
        rclpy.spin(node)
    except ImportError as error:
        raise RuntimeError("先 source 旧 NEW_test_6_15 工作区的 install/setup.bash，以提供 arm_serial.msg") from error
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
