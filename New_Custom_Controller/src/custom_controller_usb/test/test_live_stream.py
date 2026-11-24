import math
import struct
from pathlib import Path

import pytest

from custom_controller_usb.justfloat import JustFloatParser
from custom_controller_usb.live_trajectory import LiveTrajectory

MODEL = Path(__file__).resolve().parents[1] / 'config/controller_model/controller.urdf'


def packet(q=(0, 0, 0, 0, 0, 0), mask=63, stamp=100, buttons=0):
    return struct.pack('<25f', *q, *([0]*12), buttons, mask, -1, 0,
                       stamp, 0, 0) + JustFloatParser.TAIL


def test_justfloat_split_noise_and_resync():
    frame = packet((1, 2, 3, 4, 5, 6), buttons=32769)
    for split in range(len(frame)+1):
        parser = JustFloatParser()
        result = parser.feed(frame[:split]) + parser.feed(frame[split:])
        assert len(result) == 1
        assert result[0].positions == (1, 2, 3, 4, 5, 6)
        assert result[0].buttons == 32769
    parser = JustFloatParser()
    assert len(parser.feed(b'noise'+frame+frame)) == 1
    assert not parser.feed(packet(mask=63.2))
    assert not parser.feed(packet(q=(math.nan, 0, 0, 0, 0, 0)))
    assert len(parser.feed(frame)) == 1


def test_live_range_gaps_and_capacity():
    path = LiveTrajectory(MODEL.read_text(encoding='utf-8-sig'), max_points=4)
    for i in range(10):
        assert path.update((0, i*.1, 0, 0, 0, 0), 63, i*.01, i*10)
    assert path.count <= 4
    assert path.minimum != path.maximum
    previous = len(path.line_points())
    assert not path.update((0, 1, 0, 0, 0, 0), 0, .1, 100)
    path.update((0, 2, 0, 0, 0, 0), 63, .11, 110)
    assert path.segments[-1] != path.segments[-2]
    assert len(path.line_points()) <= previous
    assert not path.update((0, 2, 0, 0, 0, 0), 63, .12, 110)


def test_fk_controller_joint2_matches_known_rigid_transform():
    # A quarter turn moves the next joint origin around J2, preserving its radius.
    from custom_controller_usb.trajectory import UrdfKinematics
    import xml.etree.ElementTree as ET
    root = ET.fromstring(MODEL.read_text(encoding='utf-8-sig'))
    joints = root.findall('joint')
    for j in joints:
        j.find('origin').set('xyz', '0 0 0')
        j.find('origin').set('rpy', '0 0 0')
    joints[2].find('origin').set('xyz', '1 0 0')
    fk = UrdfKinematics(ET.tostring(root, encoding='unicode'))
    assert fk.end_effector((0, math.pi/2, 0, 0, 0, 0)) == pytest.approx((0, -1, 0))


def test_serial_to_pose_and_live_markers(tmp_path):
    import os
    import pty
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import MarkerArray
    from custom_controller_usb.receiver_node import ControllerUSBReceiver
    from custom_controller_usb.visualization_node import ControllerVisualization

    context = Context()
    rclpy.init(context=context)
    master, slave = pty.openpty()
    receiver = ControllerUSBReceiver(context=context, parameter_overrides=[
        Parameter('port', value=os.ttyname(slave)),
        Parameter('protocol', value='justfloat')])
    viewer = ControllerVisualization(context=context, parameter_overrides=[
        Parameter('urdf_path', value=str(MODEL)),
        Parameter('scales', value=[-1., 1., 1., 1., -1., 1.]),
        Parameter('offsets', value=[0.]*6)])
    probe = Node('live_stream_probe', context=context)
    poses, markers = [], []
    probe.create_subscription(JointState, '/custom_controller/target/joint_states', poses.append, 10)
    probe.create_subscription(MarkerArray, '/custom_controller/trajectory_markers', markers.append, 10)
    executor = SingleThreadedExecutor(context=context)
    for node in (receiver, viewer, probe):
        executor.add_node(node)

    def wait(predicate):
        end = time.monotonic()+5
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('Serial -> pose/trajectory timeout')

    try:
        wait(lambda: receiver._serial is not None and receiver._state_pub.get_subscription_count()>0
             and viewer._target_pub.get_subscription_count()>0)
        os.write(master, packet(stamp=100))
        wait(lambda: viewer._trajectory.count == 1)
        os.write(master, packet((.2, .3, 0, 0, 0, 0), stamp=110))
        wait(lambda: poses and abs(poses[-1].position[0]+.2)<1e-5 and
             any(len(m.markers)>1 and len(m.markers[1].points)==2 for m in markers))
        assert poses[-1].name == ['jonit1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        assert viewer._trajectory.points[0] != viewer._trajectory.points[1]
    finally:
        for node in (receiver, viewer, probe):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        context.shutdown()
        os.close(master)
        os.close(slave)
