"""Run on ROS2/Linux: topics -> serial command -> arm feedback topic."""

import os
import pty
import struct
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from custom_controller_interfaces.msg import ControllerState, OperatorInput

from custom_controller_usb.command_bridge import ControllerCommandBridge
from custom_controller_usb.command_protocol import parse_command_frame


def test_ros_topics_to_command_and_arm_feedback(tmp_path):
    context = Context()
    rclpy.init(context=context)
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    bridge = ControllerCommandBridge(context=context, parameter_overrides=[
        Parameter("port", value=os.ttyname(slave)),
        Parameter("poll_period_s", value=0.005),
        Parameter("reconnect_period_s", value=0.02),
        Parameter("scales", value=[-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        Parameter("main_button_bits", value=list(range(8))),
        Parameter("handle_button_bits", value=list(range(8, 16))),
    ])
    probe = Node("command_bridge_probe", context=context)
    state_pub = probe.create_publisher(ControllerState, "/custom_controller/state", 1)
    input_pub = probe.create_publisher(OperatorInput, "/custom_controller/operator_input", 1)
    feedback = []
    probe.create_subscription(JointState, "/custom_controller/arm_joint_states", feedback.append, 1)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    executor.add_node(probe)

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            if predicate():
                return
        pytest.fail("Timed out waiting for command bridge")

    try:
        wait_for(lambda: bridge._serial is not None and state_pub.get_subscription_count() and
                 input_pub.get_subscription_count() and bridge._feedback_pub.get_subscription_count())
        op = OperatorInput()
        op.joystick_x, op.joystick_y = 2000, 2100
        op.mouse_x = -123
        op.keys = 2
        input_pub.publish(op)
        wait_for(lambda: bridge._operator.keys == 2)

        state = ControllerState()
        state.device_time_ms = 1234
        state.positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        state.buttons = 0x8001
        state.valid_motor_mask = 0x15
        state_pub.publish(state)
        received = bytearray()

        def command_arrived():
            try:
                received.extend(os.read(master, 4096))
            except BlockingIOError:
                pass
            return len(received) >= 64

        wait_for(command_arrived)
        command = parse_command_frame(bytes(received[:64]))
        assert command.positions == (-1.0, 0.0, 3.0, 0.0, 5.0, 0.0)
        assert command.buttons == 0x8001 and command.valid_joint_mask == 0x15
        assert command.operator.joystick_x == 2000 and command.operator.mouse[0] == -123
        assert command.operator.main_buttons == 1 and command.operator.handle_buttons == 0x80

        body = struct.pack("<6f", 1, 2, 3, 4, 5, 6)
        arm_frame = b"\xAA" + body + bytes((sum(body) & 0xFF, 0x55))
        os.write(master, arm_frame[:8])
        wait_for(lambda: bridge._serial.in_waiting == 0)
        assert not feedback
        os.write(master, arm_frame[8:])
        wait_for(lambda: len(feedback) == 1)
        assert list(feedback[0].position) == [1, 2, 3, 4, 5, 6]
        assert feedback[0].name == [f"joint_{i + 1}" for i in range(6)]
        # Observation mode still receives arm feedback but cannot emit commands.
        bridge._forward_enabled = False
        state.device_time_ms = 1244
        state_pub.publish(state)
        for _ in range(20):
            executor.spin_once(timeout_sec=0.005)
        with pytest.raises(BlockingIOError):
            os.read(master, 4096)
        os.write(master, arm_frame)
        wait_for(lambda: len(feedback) == 2)
    finally:
        executor.remove_node(bridge)
        executor.remove_node(probe)
        bridge.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        context.shutdown()
        os.close(master)
        os.close(slave)
