"""Exercise the receiver with real ROS2 topics and Linux pseudoterminals."""

import math
import os
import pty
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from custom_controller_interfaces.msg import ControllerState

from custom_controller_usb.protocol import build_state_frame
from custom_controller_usb.receiver_node import ControllerUSBReceiver


@pytest.fixture
def rig(tmp_path):
    context = Context()
    rclpy.init(context=context)
    master, slave = pty.openpty()
    port = tmp_path / "controller_usb"
    port.symlink_to(os.ttyname(slave))
    receiver = ControllerUSBReceiver(context=context, parameter_overrides=[
        Parameter("port", value=str(port)),
        Parameter("poll_period_s", value=0.005),
        Parameter("reconnect_period_s", value=0.02),
    ])
    probe = Node("controller_usb_test_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(receiver)
    executor.add_node(probe)
    states, joints = [], []
    state_sub = probe.create_subscription(
        ControllerState, "/custom_controller/state", states.append, 50)
    joint_sub = probe.create_subscription(
        JointState, "/custom_controller/joint_states", joints.append, 50)
    descriptors = [master, slave]

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        pytest.fail("Timed out waiting for USB/ROS2 state")

    wait_for(lambda: receiver._serial is not None and
             receiver._state_pub.get_subscription_count() > 0 and
             receiver._joint_pub.get_subscription_count() > 0)
    try:
        yield receiver, master, port, states, joints, wait_for, descriptors
    finally:
        probe.destroy_subscription(state_sub)
        probe.destroy_subscription(joint_sub)
        executor.remove_node(receiver)
        executor.remove_node(probe)
        receiver.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        context.shutdown()
        for descriptor in descriptors:
            if descriptor >= 0:
                os.close(descriptor)


def test_partial_corrupt_and_multiple_frames_publish(rig):
    receiver, master, _, states, joints, wait_for, _ = rig
    frame = build_state_frame(1234, [0, 1, -2, 3, 4, 5], 0x8001, 0x3F, 7)
    os.write(master, frame[:1])
    wait_for(lambda: receiver._serial.in_waiting == 0)
    assert not states
    os.write(master, frame[1:17])
    wait_for(lambda: receiver._serial.in_waiting == 0)
    assert not states
    os.write(master, frame[17:])
    wait_for(lambda: len(states) == 1 and len(joints) == 1)
    state = states[0]
    assert state.sequence == 7 and state.device_time_ms == 1234
    assert list(state.positions) == [0, 1, -2, 3, 4, 5]
    assert state.buttons == 0x8001 and state.valid_motor_mask == 0x3F
    assert joints[0].name == [f"motor_{i + 1}" for i in range(6)]
    assert list(joints[0].position) == [0, 1, -2, 3, 4, 5]
    assert state.stamp == joints[0].header.stamp

    corrupt = bytearray(frame)
    corrupt[15] ^= 1
    second = build_state_frame(1244, [1] * 6, 3, 0x01, 8)
    third = build_state_frame(1254, [2] * 6, 4, 0, 9)
    os.write(master, bytes(corrupt) + second + third)
    wait_for(lambda: len(states) == 3 and len(joints) == 3)
    assert [state.sequence for state in states] == [7, 8, 9]
    assert joints[1].position[0] == 1.0
    assert all(math.isnan(value) for value in joints[1].position[1:])
    assert all(math.isnan(value) for value in joints[2].position)


def test_100hz_stream_and_disconnect_reconnect(rig):
    receiver, master, port, states, joints, wait_for, descriptors = rig
    for sequence in range(20):
        frame = build_state_frame(sequence * 10, [sequence / 10] * 6,
                                  sequence, 0x3F, sequence)
        os.write(master, frame)
        wait_for(lambda: len(states) >= sequence + 1 and len(joints) >= sequence + 1)
        time.sleep(0.01)
    assert [state.sequence for state in states] == list(range(20))

    os.write(master, b"\xA5")
    wait_for(lambda: receiver._serial.in_waiting == 0)
    os.close(master)
    descriptors[0] = -1
    wait_for(lambda: receiver._serial is None)
    new_master, new_slave = pty.openpty()
    descriptors.extend([new_master, new_slave])
    port.unlink()
    port.symlink_to(os.ttyname(new_slave))
    wait_for(lambda: receiver._serial is not None)
    os.write(new_master, build_state_frame(0, [0.5] * 6, 0xFFFF, 0x3F, 0))
    wait_for(lambda: len(states) == 21 and len(joints) == 21)
    assert states[-1].sequence == 0 and states[-1].device_time_ms == 0
    assert states[-1].buttons == 0xFFFF
    assert list(states[-1].positions) == [0.5] * 6
