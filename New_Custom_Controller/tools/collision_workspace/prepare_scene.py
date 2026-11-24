"""Assemble CAD obstacles in the existing arm URDF coordinate frame."""

import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def read_stl(path):
    with Path(path).open("rb") as stream:
        stream.seek(80)
        count = struct.unpack("<I", stream.read(4))[0]
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        return np.fromfile(stream, dtype=dtype, count=count)["vertices"].reshape(-1, 3).astype(float)


def apply(vertices, transform):
    return vertices @ transform[:3, :3].T + transform[:3, 3]


def kinematic_chain(urdf):
    root = ET.parse(urdf).getroot()
    joints = []
    for element in root.findall("joint"):
        if element.attrib["type"] == "fixed":
            continue
        origin = element.find("origin")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.attrib["rpy"], sep=" ")).as_matrix()
        matrix[:3, 3] = np.fromstring(origin.attrib["xyz"], sep=" ")
        joints.append((matrix, np.fromstring(element.find("axis").attrib["xyz"], sep=" ")))

    def fk(q):
        matrices = [np.eye(4)]
        for angle, (origin, axis) in zip(q, joints):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            matrices.append(matrices[-1] @ origin @ rotation)
        return matrices

    return fk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.cad / "assembly_manifest.json").read_text(encoding="utf-8"))
    records = manifest["components"]
    arm_root = next(r for r in records if "/" not in r["name"] and "新国赛机械臂ZP" in r["name"] and r["suppression"] != 0)
    root_to_sw = np.asarray(arm_root["transform"])
    base_to_root = np.array([[1., 0., 0., 0.], [0., 0., 1., .0235], [0., -1., 0., -.14], [0., 0., 0., 1.]])
    base_to_sw = root_to_sw @ base_to_root
    sw_to_base = np.linalg.inv(base_to_sw)
    cache = {}

    def vertices(record):
        filename = record["mesh"]
        if filename not in cache:
            cache[filename] = np.load(args.cad / filename)["vertices"]
        return apply(cache[filename], sw_to_base @ np.asarray(record["transform"]))

    nonphysical = ["装甲板不可遮挡区域", "初始尺寸限制区域", "1500限位圆", "干涉范围", "球形干涉", "LINK2范围"]
    obstacles, arm_parts, excluded, missing = [], [], [], []
    obstacle_records = []
    ground_z = None
    for record in records:
        if record.get("suppression") == 0 or record.get("doc_type") != 1:
            continue
        if "mesh" not in record:
            missing.append({key: record.get(key) for key in ["name", "box", "error"]})
            continue
        name = record["name"]
        if any(word in name for word in nonphysical):
            excluded.append(name)
            continue
        v = vertices(record)
        if "1200-1200地板" in name:
            ground_z = float(v[:, 2].max())
            continue
        if name.startswith(arm_root["name"] + "/"):
            arm_parts.append(v)
        else:
            obstacles.append(v)
            obstacle_records.append({"name": name, "triangles": len(v) // 3,
                                     "bounds": [v.min(0).tolist(), v.max(0).tolist()]})
    environment = np.concatenate(obstacles).astype(np.float32)
    cad_arm = np.concatenate(arm_parts).astype(np.float32)
    np.savez_compressed(args.output / "scene_meshes.npz", environment=environment, cad_arm=cad_arm)
    tree = cKDTree(cad_arm[::3])
    meshes = args.urdf.parent.parent / "meshes"
    link_vertices = [read_stl(meshes / (name + ".STL")) for name in ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]]
    fk = kinematic_chain(args.urdf)

    def root_point(record_name):
        r = next(r for r in records if r["name"].startswith(arm_root["name"] + "/") and r["name"].split("/")[-1] == record_name)
        return (sw_to_base @ np.asarray(r["transform"]))[:3, 3]

    elbow_target = root_point("V3-J3电机输出轴大齿轮侧-机加工-1")
    wrist_target = root_point("V3-J5输出轴-机加工-1")
    # These component origins lie at different points along the same transverse
    # axes. Match the coordinates normal to those axes, not their axial offsets.
    # In the base frame the shoulder/elbow axes are parallel to X. Their X
    # origins need not equal the CAD component origin along that line.
    def position_error(q23):
        matrices = fk([0., q23[0], q23[1], 0., 0., 0.])
        return np.r_[matrices[3][1:3, 3] - elbow_target[1:3], matrices[4][1:3, 3] - wrist_target[1:3]]

    fit23 = least_squares(position_error, [.97, -2.], bounds=([0., -3.14], [3.14, 0.]))
    cad_q = np.r_[0., fit23.x, 0., 0., 0.]
    rng = np.random.default_rng(20261114)
    clouds = [v[rng.choice(len(v), min(len(v), 1500), replace=False)] for v in link_vertices]

    def wrist_error(angles):
        q = cad_q.copy()
        q[3:] = angles
        matrices = fk(q)
        return np.concatenate([np.minimum(tree.query(apply(clouds[i], matrices[i]))[0], .025) for i in [4, 5, 6]])

    best = None
    for initial in [[0., 0., 0.], [0., 1., 0.], [0., -1., 0.], [3.0, 0., 0.]]:
        fit = least_squares(wrist_error, initial, diff_step=1e-4, bounds=([-3.14, -2., -3.14], [3.14, 2., 3.14]),
                            loss="soft_l1", f_scale=.002, max_nfev=55)
        if best is None or fit.cost < best.cost:
            best = fit
    cad_q[3:] = best.x
    matrices = fk(cad_q)
    alignment = {}
    for i, cloud in enumerate(clouds):
        distances = tree.query(apply(cloud, matrices[i]))[0] * 1000
        alignment["base_link" if i == 0 else f"Link{i}"] = dict(zip(["median_mm", "p90_mm", "p95_mm", "max_mm"], np.percentile(distances, [50, 90, 95, 100]).tolist()))
    config = {
        "schema": "arm-collision-workspace-scene-v1",
        "scope": "URDF arm with full CAD chassis in current assembly configuration; not a certified physical workspace",
        "source_assembly": manifest["assembly"], "arm_assembly": arm_root["name"],
        "base_to_solidworks": base_to_sw.tolist(),
        "mount_derivation": "J1 output-axis datum (0,0.0235,-0.14) in arm assembly; R=Rx(-pi/2); checked against CAD mesh and joint-axis positions",
        "coordinate_frame": "URDF base_link / J1 output axis, metres, Z up",
        "ground_z_m": ground_z, "tcp_translation_m": [0., 0., .192],
        "tcp_definition": "Geometric reference at old Link6 mesh tip plane; actual grasp TCP is not measured",
        "cad_pose_joint_rad": cad_q.tolist(), "cad_pose_joint_deg": np.rad2deg(cad_q).tolist(),
        "joint_axis_fit_max_error_mm": float(np.max(np.abs(position_error(cad_q[1:3]))) * 1000),
        "urdf_to_cad_alignment": alignment,
        "environment_instances": len(obstacles), "environment_triangles": len(environment) // 3,
        "cad_arm_instances": len(arm_parts), "cad_arm_triangles": len(cad_arm) // 3,
        "excluded_nonphysical": excluded, "missing_tessellation": missing,
        "obstacles": obstacle_records,
        "limitations": ["URDF link geometry and joint limits must be checked against current physical arm",
                        "CAD-only moving attachments and passive rod kinematics are not yet added to the URDF",
                        "Gripper geometry is fixed in the URDF opening state; no carried object",
                        "Chassis and step-climbing mechanisms remain in the CAD assembly pose",
                        "Joint limits come from the existing URDF and are not newly measured",
                        "TCP is a geometry-based reference, not a calibrated grasp frame"],
    }
    (args.output / "scene_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: config[key] for key in ["environment_instances", "environment_triangles", "cad_arm_instances", "cad_arm_triangles", "ground_z_m", "cad_pose_joint_deg", "joint_axis_fit_max_error_mm", "urdf_to_cad_alignment", "missing_tessellation"]}, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
