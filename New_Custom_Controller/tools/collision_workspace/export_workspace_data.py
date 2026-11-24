"""Attach distances and export witness configurations for offline inspection."""

import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

from run_workspace import CollisionScene


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    args = parser.parse_args()
    summary_path = args.scene / "workspace_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    scene = CollisionScene(args.urdf, args.scene, summary["external_clearance_threshold_m"])
    if summary.get("collision_policy", {}).get("chassis_contact_exempt_links") != scene.chassis_exempt_links:
        raise RuntimeError("Result collision policy differs from this scene; recompute before exporting distances")
    archive = np.load(args.scene / "workspace_voxels.npz")
    values = {name: archive[name] for name in archive.files}
    q = values["q"]
    clearance = []
    nearest = []
    chassis_clearance = []
    external_pairs = [i for i, pair in enumerate(scene.geometry.collisionPairs) if pair.second == scene.chassis_index]
    for index, angles in enumerate(q):
        pin.forwardKinematics(scene.model, scene.data, angles)
        pin.updateGeometryPlacements(scene.model, scene.data, scene.geometry, scene.gdata)
        external = [(float(pin.computeDistance(scene.geometry, scene.gdata, i).min_distance), scene.pair_labels[i])
                    for i in external_pairs]
        chassis_clearance.append([distance for distance, _ in external])
        if scene.ground_z is not None:
            for i in range(1, 7):
                placement = scene.gdata.oMg[i]
                distance = float(np.min(scene.ground_support[i] @ placement.rotation[2]) + placement.translation[2] - scene.ground_z)
                external.append((distance, scene.geometry.geometryObjects[i].name + "/ground"))
        distance, label = min(external)
        if distance < summary["external_clearance_threshold_m"] - 1e-5:
            raise RuntimeError(f"Witness {index} violates external clearance: {distance} m at {label}")
        clearance.append(distance)
        nearest.append(label)
        if (index + 1) % 2000 == 0:
            print("Distance annotation", index + 1, "/", len(q), flush=True)
    values["external_clearance_m"] = np.asarray(clearance)
    values["nearest_external_pair"] = np.asarray(nearest)
    link_names = summary["collision_policy"]["chassis_checked_links"]
    values["chassis_clearance_by_link_m"] = np.asarray(chassis_clearance)
    values["chassis_clearance_link_names"] = np.asarray(link_names)
    np.savez_compressed(args.scene / "workspace_voxels.npz", **values)
    columns = np.column_stack([values["index"], values["tcp"], np.rad2deg(q), values["quaternion_xyzw"], clearance,
                               values["chassis_clearance_by_link_m"]])
    header = "voxel_ix,voxel_iy,voxel_iz,x_m,y_m,z_m,J1_deg,J2_deg,J3_deg,J4_deg,J5_deg,J6_deg,quat_x,quat_y,quat_z,quat_w,external_clearance_m"
    header += "," + ",".join(name + "_chassis_clearance_m" for name in link_names)
    np.savetxt(args.scene / "workspace_voxels.csv", columns, delimiter=",", header=header, comments="", fmt="%.8g")
    summary["witness_external_clearance_m"] = {"minimum": float(np.min(clearance)), "median": float(np.median(clearance)),
                                               "maximum": float(np.max(clearance))}
    summary["witness_chassis_minimum_clearance_by_link_m"] = {
        name: float(np.min(values["chassis_clearance_by_link_m"][:, i])) for i, name in enumerate(link_names)}
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("Witness CSV exported", len(q), "rows", summary["witness_external_clearance_m"], flush=True)


if __name__ == "__main__":
    main()
