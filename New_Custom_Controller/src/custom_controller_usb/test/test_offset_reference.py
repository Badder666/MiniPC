"""Verify nonzero calibration offsets across reference, live pose and gravity."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from custom_controller_interfaces.msg import ControllerState
from custom_controller_usb.receiver_node import ControllerUSBReceiver
from custom_controller_usb.visualization_node import ControllerVisualization


class OffsetReferenceTests(unittest.TestCase):
    def test_reference_without_usb_and_common_gravity_offset(self):
        rclpy.init()
        visual = receiver = observer = None
        executor = SingleThreadedExecutor()
        try:
            offsets = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
            scales = [-1.0, -0.25, 1.0, -1.0, 1.0, -1.0]
            model = str(Path(get_package_share_directory('custom_controller_usb')) /
                        'config/controller_model_urdf2/controller.urdf')
            visual = ControllerVisualization(parameter_overrides=[
                Parameter('urdf_path', value=model), Parameter('offsets', value=offsets),
                Parameter('scales', value=scales), Parameter('show_planned_trajectory', value=False),
            ])
            receiver = ControllerUSBReceiver(parameter_overrides=[
                Parameter('urdf_path', value=model), Parameter('offsets', value=offsets),
                Parameter('position_offsets', value=[9.0] * 6),
                Parameter('position_scales', value=scales),
                Parameter('gravity_feedforward', value=True), Parameter('gravity_dry_run', value=True),
            ])
            self.assertEqual(receiver._position_offsets, tuple(offsets))
            self.assertIsNotNone(receiver._gravity_model)
            observer = rclpy.create_node('offset_reference_test')
            references = []
            geometry = []
            observer.create_subscription(JointState, '/custom_controller/zero/joint_states',
                                         references.append, 10)
            observer.create_subscription(MarkerArray, '/custom_controller/joint_geometry_markers',
                                         geometry.append, 10)
            executor.add_node(visual)
            executor.add_node(observer)
            deadline = time.monotonic() + 3.0
            while (not references or not geometry) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(references, 'Zero pose must publish without USB data')
            self.assertEqual(references[-1].position.tolist(), offsets)
            self.assertEqual(references[-1].name, [f'joint{i}' for i in range(1, 7)])
            self.assertTrue(geometry, 'Joint centers must publish without USB data')
            self.assertEqual(len(geometry[-1].markers), 36)
            for i, marker in enumerate(geometry[-1].markers):
                pose = 'target' if i < 18 else 'zero'
                joint = i % 18 // 3 + 1
                self.assertEqual(marker.header.frame_id, f'{pose}/link{joint}')
                self.assertTrue(marker.frame_locked)
                if marker.type == Marker.SPHERE:
                    self.assertEqual((marker.pose.position.x, marker.pose.position.y,
                                      marker.pose.position.z), (0., 0., 0.))
                elif marker.type == Marker.ARROW:
                    self.assertAlmostEqual(marker.points[1].z, -0.070)
            frame = ControllerState()
            frame.positions = [1.0] * 6
            frame.valid_motor_mask = 63
            frame.device_time_ms = 10
            visual._controller_cb(frame)
            expected = [a + b for a, b in zip(scales, offsets)]
            for actual, value in zip(visual._state.targets, expected):
                self.assertAlmostEqual(actual, value)
            self.assertEqual(visual._zero_positions, tuple(offsets))
            self.assertEqual(receiver._position_scales, tuple(scales))
            legacy = ControllerUSBReceiver(parameter_overrides=[
                Parameter('position_offsets', value=offsets),
            ])
            try:
                self.assertEqual(legacy._position_offsets, tuple(offsets))
            finally:
                legacy.destroy_node()
        finally:
            executor.shutdown()
            for node in (visual, receiver, observer):
                if node is not None:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
