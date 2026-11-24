"""Calibration records raw positions, persists them and updates gravity live."""

from pathlib import Path
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import yaml

from custom_controller_interfaces.msg import ControllerState
from custom_controller_usb.receiver_node import ControllerUSBReceiver
from custom_controller_usb.visualization_node import ControllerVisualization


class ZeroSnapshotTests(unittest.TestCase):
    def test_capture_persistence_live_update_and_invalid_feedback(self):
        rclpy.init()
        executor = SingleThreadedExecutor()
        visual = receiver = client_node = None
        try:
            share = Path(get_package_share_directory('custom_controller_usb'))
            model = str(share / 'config/controller_model_urdf2/controller.urdf')
            scales = [-1., -.25, 1., -1., 1., -1.]
            order = [1, 0, 2, 3, 4, 5]
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / 'mapping.yaml'
                config.write_text('# 保留注释\n/**:\n  ros__parameters:\n' +
                    '    axis_order: ' + str(order) + '\n' +
                    '    scales: ' + str(scales) + '\n' +
                    '    offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n', encoding='utf-8')
                receiver = ControllerUSBReceiver(parameter_overrides=[
                    Parameter('port', value='/tmp/nonexistent_snapshot_test_port'),
                    Parameter('reconnect_period_s', value=30.0),
                    Parameter('position_scales', value=scales), Parameter('axis_order', value=order),
                    Parameter('urdf_path', value=model),
                    Parameter('gravity_feedforward', value=True), Parameter('gravity_dry_run', value=True),
                ])
                visual = ControllerVisualization(parameter_overrides=[
                    Parameter('urdf_path', value=model), Parameter('scales', value=scales),
                    Parameter('axis_order', value=order),
                    Parameter('calibration_config_path', value=str(config)),
                    Parameter('display_stale_after_s', value=2.0),
                    Parameter('show_planned_trajectory', value=False),
                ])
                client_node = rclpy.create_node('snapshot_test_client')
                client = client_node.create_client(Trigger, '/custom_controller/capture_zero')
                for node in (visual, receiver, client_node):
                    executor.add_node(node)

                def spin_until(condition, seconds=3.0):
                    deadline = time.monotonic() + seconds
                    while not condition() and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.01)
                    self.assertTrue(condition(), 'ROS operation timed out')

                spin_until(lambda: client.service_is_ready() and
                           visual._calibration.receiver.service_is_ready())
                before = config.read_text()
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                self.assertFalse(future.result().success)
                self.assertEqual(config.read_text(), before)

                motors = [1., 2., 3., 4., 5., 6.]
                state = ControllerState()
                state.positions = motors
                state.valid_motor_mask = 63
                state.device_time_ms = 10
                visual._controller_cb(state)
                expected = [-scales[j] * motors[m] for j, m in enumerate(order)]
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                self.assertTrue(future.result().success, future.result().message)
                self.assertEqual(receiver._position_offsets, tuple(expected))
                self.assertEqual(visual._zero_positions, (0.,) * 6)
                self.assertEqual(visual._state.targets, [0.] * 6)
                self.assertEqual(visual._trajectory.count, 0)
                saved = yaml.safe_load(config.read_text())['/**']['ros__parameters']
                self.assertEqual(saved['offsets'], expected)
                self.assertEqual(saved['zero_encoder_positions'], motors)
                self.assertTrue(config.read_text().startswith('# 保留注释'))
                self.assertEqual(len(list((config.parent/'zero_snapshots').glob('*_before.yaml'))), 1)
                self.assertEqual(len(list((config.parent/'zero_snapshots').glob('*.json'))), 1)
                torque, mask = receiver._gravity_model.torque_from_motor_positions(
                    motors, order, scales, receiver._position_offsets)
                g = receiver._gravity_model.gravity_torques([0.] * 6)
                self.assertEqual(mask, 63)
                for j, m in enumerate(order):
                    self.assertAlmostEqual(torque[m], g[j])

                # Partial feedback must not replace a saved calibration.
                state.valid_motor_mask = 31
                visual._controller_cb(state)
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                self.assertFalse(future.result().success)
                self.assertEqual(yaml.safe_load(config.read_text())['/**']['ros__parameters'], saved)
                # Invalid dynamic offsets are rejected without corrupting gravity mapping.
                result = receiver.set_parameters_atomically([Parameter('offsets', value=[0.] * 5)])
                self.assertFalse(result.successful)
                self.assertEqual(receiver._position_offsets, tuple(expected))

                # A save failure rolls back gravity's live offsets.
                state.positions = [v + 0.2 for v in motors]
                state.valid_motor_mask = 63
                visual._controller_cb(state)
                valid_config = config.read_text()
                config.write_text(valid_config.replace(str(scales), str([1.] * 6)))
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                self.assertFalse(future.result().success)
                self.assertEqual(receiver._position_offsets, tuple(expected))
                self.assertEqual(visual._state.mapper.offsets, tuple(expected))
                config.write_text(valid_config)
                # Stale data cannot silently become a zero snapshot.
                visual._state.controller_arrival = time.monotonic() - 10.0
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                self.assertFalse(future.result().success)
                self.assertEqual(config.read_text(), valid_config)

                executor.remove_node(visual)
                visual.destroy_node()
                visual = ControllerVisualization(parameter_overrides=[
                    Parameter('urdf_path', value=model), Parameter('scales', value=scales),
                    Parameter('axis_order', value=order), Parameter('offsets', value=expected),
                    Parameter('zero_encoder_positions', value=motors),
                    Parameter('show_planned_trajectory', value=False),
                ])
                self.assertEqual(visual._zero_positions, (0.,) * 6)
        finally:
            executor.shutdown()
            for node in (visual, receiver, client_node):
                if node is not None:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
