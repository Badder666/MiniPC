"""COM coordinate/aggregation and ROS display checks for the static candidate."""

from pathlib import Path
import json
import math
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray

from custom_controller_usb.centers_of_mass import CenterOfMassModel
from custom_controller_usb.visualization_node import ControllerVisualization


class CenterOfMassTests(unittest.TestCase):
    def test_candidate_aggregation_and_published_coordinates(self):
        model_dir = Path(get_package_share_directory("custom_controller_usb")) / "config/controller_model_urdf2"
        candidate = model_dir / "controller_identified.urdf"
        original = model_dir / "controller.urdf"
        model = CenterOfMassModel(candidate.read_text())
        prior = CenterOfMassModel(original.read_text())
        fit = json.loads((model_dir / "controller_identified_fit.json").read_text())
        q = fit["rear_assembly"]["reference_model_angles_rad"]
        aggregate = model.aggregate(q, ("link3", "link4", "link5", "link6"), "link3")
        expected = fit["rear_assembly"]["fitted_equivalent_com_at_reference_m"]
        self.assertLess(math.dist(aggregate.xyz, expected), 1e-10)
        moved = model.aggregate([*q[:4], q[4] + 0.7, q[5]],
                                ("link3", "link4", "link5", "link6"), "link3")
        self.assertGreater(math.dist(moved.xyz, aggregate.xyz), 1e-3)
        self.assertEqual(set(model.links), {"base_link", *[f"link{i}" for i in range(1, 7)]})
        for name in model.links:
            self.assertEqual(model.links[name].mass, prior.links[name].mass)
            if name not in ("link2", "link3"):
                self.assertEqual(model.links[name].xyz, prior.links[name].xyz)
        rclpy.init()
        visual = observer = None
        executor = SingleThreadedExecutor()
        try:
            visual = ControllerVisualization(parameter_overrides=[
                Parameter("urdf_path", value=str(candidate)),
                Parameter("com_reference_urdf_path", value=str(original)),
                Parameter("show_planned_trajectory", value=False),
            ])
            observer = rclpy.create_node("com_marker_test")
            received = []
            observer.create_subscription(MarkerArray, "/custom_controller/center_of_mass_markers", received.append, 10)
            executor.add_node(visual)
            executor.add_node(observer)
            deadline = time.monotonic() + 3.0
            while not received and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(received, "COM markers must publish without USB")
            spheres = [m for m in received[-1].markers if m.type == Marker.SPHERE]
            active = [m for m in spheres if m.ns == "link_com_active"]
            self.assertEqual(len(active), 7)
            for marker in active:
                name = marker.header.frame_id.removeprefix("target/")
                p = marker.pose.position
                self.assertEqual((p.x, p.y, p.z), model.links[name].xyz)
                self.assertTrue(marker.frame_locked)
            self.assertEqual(len([m for m in spheres if m.ns == "link_com_original"]), 2)
        finally:
            executor.shutdown()
            for node in (visual, observer):
                if node is not None:
                    node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
