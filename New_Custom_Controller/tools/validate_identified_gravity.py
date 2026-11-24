"""独立验证导出URDF的重力、能量梯度和拟合预测一致性。"""

import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from custom_controller_usb.pinocchio_gravity import PinocchioGravityModel
from custom_controller_usb.centers_of_mass import CenterOfMassModel


def validate(candidate, original, fit_path, comparison, output):
    fit = json.loads(Path(fit_path).read_text(encoding="utf-8-sig"))
    model = PinocchioGravityModel(candidate, fit["mapping"]["gravity"])
    with Path(comparison).open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    predicted, measured, expected = [], [], []
    for row in rows:
        q = [float(row[f"q{i}_model_rad"]) for i in range(1, 7)]
        gravity = model.gravity_torques(q)
        predicted.append([gravity[i-1] for i in (2, 3)])
        measured.append([float(row[f"J{i}_measured_model_nm"]) for i in (2, 3)])
        expected.append([float(row[f"J{i}_fit_model_nm"]) - fit["bias_model_nm"][i-2]
                         for i in (2, 3)])
    predicted, measured, expected = map(np.asarray, (predicted, measured, expected))
    fit_error = float(np.max(np.abs(predicted - expected)))
    # This check uses independent forward kinematics/COMs, rather than RNEA.
    geometry = CenterOfMassModel(Path(candidate).read_text())
    gravity_vector = fit["mapping"]["gravity"]

    def potential(q):
        poses = geometry.kinematics.link_transforms(q)
        energy = 0.0
        for name, link in geometry.links.items():
            t = poses[name]
            xyz = [t[i][3] + sum(t[i][j] * link.xyz[j] for j in range(3)) for i in range(3)]
            energy -= link.mass * sum(gravity_vector[i] * xyz[i] for i in range(3))
        return energy

    derivative_error = 0.0
    for row in rows:
        q = np.array([float(row[f"q{i}_model_rad"]) for i in range(1, 7)])
        gradient = []
        for i in range(6):
            delta = np.zeros(6); delta[i] = 1e-6
            gradient.append((potential(q + delta) - potential(q - delta)) / 2e-6)
        derivative_error = max(derivative_error, float(np.max(np.abs(
            np.asarray(model.gravity_torques(q)) - gradient))))
    # Only two inertial origins may change; joint frames, mass and all meshes stay identical.
    old_root, new_root = ET.parse(original).getroot(), ET.parse(candidate).getroot()
    for root in (old_root, new_root):
        for link in root.findall("link"):
            if link.get("name") in ("link2", "link3"):
                link.find("inertial/origin").set("xyz", "COM_CORRECTION")
    assert ET.tostring(old_root) == ET.tostring(new_root)
    assert fit_error < 1e-10, fit_error
    assert derivative_error < 1e-7, derivative_error
    report = {
        "pose_count": len(rows),
        "fit_prediction_max_error_nm": fit_error,
        "energy_gradient_max_error_nm": derivative_error,
        "pure_gravity_rmse_J2_J3_nm": np.sqrt(np.mean((predicted-measured)**2, axis=0)).tolist(),
        "gravity_plus_diagnostic_bias_rmse_J2_J3_nm": np.sqrt(np.mean(
            (predicted + fit["bias_model_nm"] - measured)**2, axis=0)).tolist(),
        "bias_applied_to_runtime": False,
        "link_com_m": {name: list(link.xyz) for name, link in geometry.links.items()},
    }
    Path(output).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("candidate", "original", "fit", "comparison", "output"):
        parser.add_argument(f"--{name}", required=True)
    args = parser.parse_args()
    validate(args.candidate, args.original, args.fit, args.comparison, args.output)
