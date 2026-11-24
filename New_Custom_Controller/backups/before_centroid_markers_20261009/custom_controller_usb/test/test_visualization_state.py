from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from custom_controller_usb.joint_mapping import JointMapper
from custom_controller_usb.visualization_state import (
    VisualizationState, read_arm_description, target_robot_description,
)


WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / "src/new_hj_arm2/urdf/new_hj_arm2.urdf"
CONTROLLER_MODEL = WORKSPACE / "src/custom_controller_usb/config/controller_model/controller.urdf"


class VisualizationStateTests(unittest.TestCase):
    def setUp(self):
        self.state = VisualizationState(JointMapper(range(6), [1.0] * 6, [0.0] * 6),
                                        [f"joint_{i + 1}" for i in range(6)],
                                        [0, 2.4, -2.4, 0, 0, 0])

    def test_model_and_mesh_assets(self):
        text = MODEL.read_text(encoding="utf-8")
        description = read_arm_description(text)
        self.assertEqual(description.root_link, "world")
        self.assertEqual(description.joint_names, ("J1", "J2", "J3", "J4", "J5", "J6"))
        self.assertEqual(description.limits[1], (0.0, 3.14))
        model_root = MODEL.parents[1]
        for mesh in ET.fromstring(text).iter("mesh"):
            path = model_root / mesh.attrib["filename"].removeprefix("package://new_hj_arm2/")
            self.assertTrue(path.is_file(), path)
            self.assertGreater(path.stat().st_size, 84)
        renamed = read_arm_description(text.replace('name="J6"', 'name="other"'))
        self.assertEqual(renamed.joint_names[-1], "other")

    def test_controller_model_uses_its_native_joint_names(self):
        text = CONTROLLER_MODEL.read_text(encoding="utf-8")
        description = read_arm_description(text)
        self.assertEqual(description.root_link, "base_link")
        self.assertEqual(description.joint_names,
                         ("jonit1", "joint2", "joint3", "joint4", "joint5", "joint6"))

    def test_target_material_preserves_kinematics_and_meshes(self):
        text = MODEL.read_text(encoding="utf-8")
        target = target_robot_description(text)
        self.assertEqual(read_arm_description(text), read_arm_description(target))
        original, tinted = ET.fromstring(text), ET.fromstring(target)
        self.assertEqual([m.attrib for m in original.iter("mesh")],
                         [m.attrib for m in tinted.iter("mesh")])
        self.assertEqual([ET.tostring(j) for j in original.findall("joint")],
                         [ET.tostring(j) for j in tinted.findall("joint")])
        for visual in tinted.findall("./link/visual"):
            self.assertEqual(visual.find("material/color").attrib["rgba"], "0.2 0.85 0.45 1")

    def test_partial_validity_holds_previous_targets(self):
        self.state.update_controller([1, 2, 3, 4, 5, 6], 0x15, 0x8001, 7, 1234, 1.0)
        self.assertEqual(self.state.effective_valid_mask(1.2), 0x15)
        self.assertEqual(self.state.targets, [1, 2.4, 3, 0, 5, 0])
        self.assertEqual(self.state.buttons, 0x8001)
        self.state.update_controller([9] * 6, 0, 0, 8, 1244, 1.01)
        self.assertEqual(self.state.targets, [1, 2.4, 3, 0, 5, 0])
        self.assertEqual(self.state.effective_valid_mask(1.02), 0)

    def test_expiry_rate_and_joint_error(self):
        self.state.update_controller([1] * 6, 0x3F, 0, 0, 0, 1.0)
        self.state.update_controller([1] * 6, 0x3F, 0, 1, 10, 1.01)
        self.state.update_feedback([f"joint_{i + 1}" for i in range(6)], [0.5] * 6, 1.01)
        self.assertAlmostEqual(self.state.joint_error(0, 1.02), 0.5)
        self.assertAlmostEqual(self.state.receive_rate(1.02), 100)
        self.assertEqual(self.state.effective_valid_mask(1.3), 0)
        self.assertIsNone(self.state.joint_error(0, 1.3))
        self.assertEqual(self.state.targets, [1] * 6)

    def test_feedback_names_and_reordering(self):
        names = [f"joint_{i + 1}" for i in range(6)]
        self.state.update_feedback(names[::-1], list(range(6))[::-1], 1.0)
        self.assertEqual(self.state.actual, list(range(6)))
        with self.assertRaises(ValueError):
            self.state.update_feedback(["bad"] * 6, [0] * 6, 2.0)
        with self.assertRaises(ValueError):
            self.state.update_feedback(names, [float("nan")] * 6, 2.0)
        self.assertEqual(self.state.actual, list(range(6)))

    def test_controller_zero_maps_to_folded_pose_with_inverted_axes(self):
        mapper = JointMapper(range(6), [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0],
                             [0.0, 2.4, -2.4, 0.0, 0.0, 0.0])
        targets, mask = mapper.map([0.0] * 6, 0x3F)
        self.assertEqual(targets, (0.0, 2.4, -2.4, 0.0, 0.0, 0.0))
        self.assertEqual(mask, 0x3F)
        targets, mask = mapper.map([0.1] * 6, 0x3F)
        self.assertEqual(targets, (-0.1, 2.3, -2.3, -0.1, -0.1, -0.1))
        self.assertEqual(mask, 0x3F)


if __name__ == "__main__":
    unittest.main()
