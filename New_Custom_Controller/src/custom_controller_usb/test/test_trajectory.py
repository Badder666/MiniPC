import csv
import tempfile
import unittest
from pathlib import Path

from custom_controller_usb.joint_mapping import JointMapper
from custom_controller_usb.trajectory import UrdfKinematics, load_trajectory


WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / "src/new_hj_arm2/urdf/new_hj_arm2.urdf"
MAPPER = JointMapper(range(6), [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0],
                     [0.0, 2.4, -2.4, 0.0, 0.0, 0.0])


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.urdf = MODEL.read_text(encoding="utf-8")

    def test_fk_has_finite_end_effector(self):
        point = UrdfKinematics(self.urdf).end_effector((0.0, 2.4, -2.4, 0.0, 0.0, 0.0))
        self.assertEqual(len(point), 3)
        self.assertTrue(all(abs(value) < 10.0 for value in point))

    def test_vofa_controller_columns_and_valid_mask(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", newline="", delete=False) as handle:
            writer = csv.writer(handle)
            writer.writerow([f"I{i}" for i in range(20)])
            writer.writerow([0, 0, 0, 0, 0, 0] + [0] * 13 + [63])
            writer.writerow([0, 0, 0, 0, 0, 0] + [0] * 13 + [1])
            path = Path(handle.name)
        try:
            data = load_trajectory(path, self.urdf, MAPPER)
        finally:
            path.unlink()
        self.assertEqual(data.count, 1)
        self.assertEqual(data.rows_read, 2)
        self.assertEqual(data.rows_skipped, 1)

    def test_collision_csv_j_columns_are_urdf_coordinates(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", newline="", delete=False) as handle:
            writer = csv.writer(handle)
            writer.writerow(["index", "J1", "J2", "J3", "J4", "J5", "J6", "collision_free"])
            writer.writerow([0, 0, 2.4, -2.4, 0, 0, 0, 1])
            writer.writerow([1, 0, 2.4, -2.4, 0, 0, 0, 0])
            path = Path(handle.name)
        try:
            data = load_trajectory(path, self.urdf, MAPPER)
        finally:
            path.unlink()
        self.assertEqual(data.count, 1)
        self.assertAlmostEqual(data.joint_positions[0][1], 2.4)


if __name__ == "__main__":
    unittest.main()
