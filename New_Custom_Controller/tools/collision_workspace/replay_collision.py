"""Audit recorded arm motion against the full collision scene; no hardware I/O."""

import argparse
from collections import Counter
import csv
import hashlib
import html
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import plotly.graph_objects as go

from prepare_scene import apply, kinematic_chain, read_stl
from run_workspace import CollisionScene
from visualize_workspace import mesh_trace, surface_subset


# Same motor -> URDF mapping as Robotic_Arm_Operation and the USB recorder.
SCALES = np.array([1., 1., 1. / 1.65, -1., 1., -1.])
OFFSETS = np.array([0., np.pi - .07, -np.pi - .05, 0., 0., 0.])
TRACKS = {
    "actual": ("motor_q_rad", "joint_q_rad"),
    "ramped": ("motor_ramped_target_rad", "joint_ramped_target_rad"),
    "target": ("motor_target_rad", "joint_target_rad"),
}


def external_distances(scene, q):
    pin.forwardKinematics(scene.model, scene.data, q)
    pin.updateGeometryPlacements(scene.model, scene.data, scene.geometry, scene.gdata)
    values = {scene.pair_labels[i]: float(pin.computeDistance(scene.geometry, scene.gdata, i).min_distance)
              for i, pair in enumerate(scene.geometry.collisionPairs) if pair.second == scene.chassis_index}
    if scene.ground_z is not None:
        for i in range(1, 7):
            pose = scene.gdata.oMg[i]
            values[scene.geometry.geometryObjects[i].name + "/ground"] = float(
                np.min(scene.ground_support[i] @ pose.rotation[2]) + pose.translation[2] - scene.ground_z)
    return values


def contact_details(scene, q):
    data = pin.GeometryData(scene.geometry)
    for request in data.collisionRequests:
        request.security_margin = 0.
        request.enable_contact = True
        request.num_max_contacts = 8
    pin.computeCollisions(scene.model, scene.data, scene.geometry, data, q, False)
    obstacles = scene.config["obstacles"]
    ends = np.cumsum([part["triangles"] for part in obstacles])
    contacts = []
    for i, result in enumerate(data.collisionResults):
        for contact in result.getContacts():
            value = {"pair": scene.pair_labels[i], "position_m": np.asarray(contact.pos).tolist(),
                     "penetration_depth_m": float(contact.penetration_depth)}
            if scene.geometry.collisionPairs[i].second == scene.chassis_index:
                triangle = int(contact.b2)
                part = int(np.searchsorted(ends, triangle, side="right"))
                if triangle >= 0 and part < len(obstacles):
                    value["chassis_part"] = obstacles[part]["name"]
                    value["chassis_triangle"] = triangle
            contacts.append(value)
    return contacts


def diagnose(scene, q):
    violations = scene.valid(q, explain=True)
    distances = external_distances(scene, q)
    return {"q_rad": q.tolist(), "q_deg": np.rad2deg(q).tolist(),
            "margin_violations": violations, "external_distances_m": distances,
            "contacts": contact_details(scene, q)}


def replay(scene, path, output, tracks, distance_period, examples):
    sidecar = path.with_suffix(".session.json")
    if sidecar.exists():
        metadata = json.loads(sidecar.read_text(encoding="utf-8-sig"))
        for key, expected in [("joint_scales", SCALES), ("joint_offsets_rad", OFFSETS)]:
            if key in metadata and not np.allclose(metadata[key], expected, rtol=0., atol=1e-7):
                raise ValueError(f"{path}: recorded {key} differs from the firmware mapping")
    stats = {name: {"poses": 0, "margin_violations": 0, "mesh_contacts": 0,
                    "joint_limit_violations": 0, "invalid_feedback": 0,
                    "checked_edges": 0, "blocked_edges": 0, "unknown_edges": 0,
                    "pair_counts": Counter(), "contact_pair_counts": Counter()}
             for name in tracks}
    zero_data = pin.GeometryData(scene.geometry)
    for request in zero_data.collisionRequests:
        request.security_margin = 0.
        request.num_max_contacts = 1
    previous = {}
    last_distance = {name: -np.inf for name in tracks}
    plot_values = {name: [] for name in tracks}
    mapping_error = 0.
    elapsed = 0.
    last_tick = None
    last_segment = None
    fields = ["source_row", "time_s", "track", "outputs_enabled", "feedback_valid",
              "margin_violations", "mesh_contacts", "joint_limit_joints", "path_status",
              "external_clearance_m", "nearest_external_pair"] + [f"J{i}_rad" for i in range(1, 7)]
    output.mkdir(parents=True, exist_ok=True)
    with path.open(encoding="utf-8-sig", newline="") as source, (output / "collision_replay.csv").open("w", newline="", encoding="utf-8-sig") as destination:
        reader = csv.DictReader(source)
        writer = csv.DictWriter(destination, fieldnames=fields)
        writer.writeheader()
        for row_number, row in enumerate(reader, 1):
            tick = int(row["device_tick_ms"])
            segment = (row.get("recording_segment", "0"), row.get("device_segment", "0"))
            delta_ms = ((tick - last_tick) & 0xFFFFFFFF) if last_tick is not None else 0
            contiguous = last_tick is not None and segment == last_segment and 0 < delta_ms <= 50
            elapsed += delta_ms / 1000. if last_tick is not None and segment == last_segment and delta_ms < 10000 else 0.01
            last_tick, last_segment = tick, segment
            feedback_valid = (int(row["feedback_mask"]) == 63 and
                              max(float(row[f"can_age_ms_{i}"]) for i in range(1, 7)) <= 30.)
            enabled = int(row["outputs_enabled"])
            for name in tracks:
                motor_prefix, joint_prefix = TRACKS[name]
                q = np.array([float(row[f"{motor_prefix}_{i}"]) for i in range(1, 7)]) * SCALES + OFFSETS
                recorded = np.array([float(row[f"{joint_prefix}_{i}"]) for i in range(1, 7)])
                finite = bool(np.isfinite(q).all() and np.isfinite(recorded).all())
                if finite:
                    error = float(np.max(np.abs(q - recorded)))
                    mapping_error = max(mapping_error, error)
                    if error > 1e-6:
                        raise ValueError(f"{path}:{row_number}: {name} motor/joint mapping differs by {error} rad")
                values = stats[name]
                values["poses"] += 1
                values["invalid_feedback"] += int(not feedback_valid or not finite)
                violations, mesh_contacts, limits = [], [], []
                distance, nearest = np.nan, ""
                if finite:
                    limits = [f"J{i+1}" for i in range(6) if q[i] < scene.model.lowerPositionLimit[i] or q[i] > scene.model.upperPositionLimit[i]]
                    violations = scene.valid(q, explain=True)
                    if violations:
                        pin.computeCollisions(scene.model, scene.data, scene.geometry, zero_data, q, False)
                        mesh_contacts = [scene.pair_labels[i] for i, result in enumerate(zero_data.collisionResults) if result.isCollision()]
                        if scene.ground_z is not None:
                            for i in range(1, 7):
                                pose = scene.gdata.oMg[i]
                                if np.min(scene.ground_support[i] @ pose.rotation[2]) + pose.translation[2] <= scene.ground_z:
                                    mesh_contacts.append(scene.geometry.geometryObjects[i].name + "/ground")
                    if elapsed - last_distance[name] >= distance_period:
                        distances = external_distances(scene, q)
                        nearest = min(distances, key=distances.get)
                        distance = distances[nearest]
                        last_distance[name] = elapsed
                    if name == "actual":
                        examples.setdefault("first_recorded_actual", q.copy())
                        if mesh_contacts:
                            examples.setdefault("first_recorded_contact", q.copy())
                valid = finite and feedback_valid and not limits and not violations
                old = previous.get(name)
                if old is None or not contiguous:
                    path_status = "unknown_gap"
                    values["unknown_edges"] += 1
                elif not valid or not old[1]:
                    path_status = "blocked_endpoint"
                    values["blocked_edges"] += 1
                else:
                    values["checked_edges"] += 1
                    path_status = "checked_clear" if scene.edge(old[0], q) else "blocked_intermediate"
                    if path_status == "blocked_intermediate":
                        values["blocked_edges"] += 1
                previous[name] = q.copy(), valid
                values["margin_violations"] += int(bool(violations))
                values["mesh_contacts"] += int(bool(mesh_contacts))
                values["joint_limit_violations"] += int(bool(limits))
                values["pair_counts"].update(violations)
                values["contact_pair_counts"].update(mesh_contacts)
                writer.writerow(dict(source_row=row_number, time_s=elapsed, track=name, outputs_enabled=enabled,
                                     feedback_valid=int(feedback_valid and finite), margin_violations=";".join(violations),
                                     mesh_contacts=";".join(mesh_contacts), joint_limit_joints=";".join(limits),
                                     path_status=path_status, external_clearance_m=distance,
                                     nearest_external_pair=nearest, **{f"J{i+1}_rad": float(q[i]) for i in range(6)}))
                plot_values[name].append((elapsed, distance, bool(violations)))
            if row_number % 2000 == 0:
                print(path.name, "rows", row_number, "actual violations", stats.get("actual", {}).get("margin_violations"), flush=True)
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for name, rows in plot_values.items():
        data = np.asarray(rows)
        if len(data):
            sampled = np.isfinite(data[:, 1])
            axes[0].plot(data[sampled, 0], data[sampled, 1] * 1000., label=name, linewidth=1)
            axes[1].plot(data[:, 0], data[:, 2], label=name, linewidth=1, alpha=.65)
    axes[0].axhline(scene.margin * 1000., color="red", linestyle="--", label="external margin")
    axes[0].set_ylabel("Sampled external clearance / mm")
    axes[1].set(ylabel="Full-arm margin violation", xlabel="Recording time / s", yticks=[0, 1])
    for axis in axes:
        axis.grid(alpha=.25)
        axis.legend()
    fig.suptitle("Offline collision replay: " + path.name)
    fig.tight_layout()
    fig.savefig(output / "collision_replay.png", dpi=140)
    plt.close(fig)
    return {"input": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "motor_joint_mapping_max_error_rad": mapping_error, "tracks": stats,
            "distance_sampling_period_s": distance_period,
            "path_check": "all finite healthy recorded endpoints; same-segment gaps <=50 ms; full-mesh finite subdivisions"}


def publish_diagnostics(scene, scene_dir, urdf, output, diagnostics):
    meshes = np.load(scene_dir / "scene_meshes.npz")
    fig = go.Figure()
    fig.add_trace(mesh_trace(surface_subset(meshes["environment"], 40000, 1), "Chassis", "#8293a5", .7))
    chain = kinematic_chain(urdf)
    links = ["base_link"] + [f"Link{i}" for i in range(1, 7)]
    vertices = [read_stl(urdf.parent.parent / "meshes" / (name + ".STL")) for name in links]
    groups = []
    tables = []
    for index, (name, diagnostic) in enumerate(diagnostics.items()):
        first = len(fig.data)
        transforms = chain(diagnostic["q_rad"])
        for i, link in enumerate(links):
            color = "#e3b34d" if i < 2 else "#a4daf4"
            if any(link + "_0/" in pair or "/" + link + "_0" in pair for pair in diagnostic["margin_violations"]):
                color = "#ef5350"
            triangles = surface_subset(apply(vertices[i], transforms[i]), 3000, i + 10)
            fig.add_trace(mesh_trace(triangles, name + ": " + link, color, .95, index == 0))
        points = np.asarray([contact["position_m"] for contact in diagnostic["contacts"]]).reshape(-1, 3)
        fig.add_trace(go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers",
                                  name="Mesh contact", marker=dict(color="#ff1111", size=6), visible=index == 0))
        groups.append(list(range(first, len(fig.data))))
        rows = "".join("<tr><td>" + html.escape(c["pair"]) + "</td><td>" + html.escape(c.get("chassis_part", "arm link")) + "</td></tr>"
                       for c in diagnostic["contacts"])
        tables.append(f"<h3>{html.escape(name)}</h3><p>关节角 / °：{np.round(diagnostic['q_deg'], 2).tolist()}；"
                      f"间隙违规：{html.escape(', '.join(diagnostic['margin_violations']) or '无')}</p>"
                      f"<table border='1' cellpadding='5'><tr><th>碰撞对</th><th>车体部件</th></tr>{rows}</table>")
    buttons = []
    for name, group in zip(diagnostics, groups):
        visible = [i == 0 or i in group for i in range(len(fig.data))]
        buttons.append(dict(label=name, method="update", args=[{"visible": visible}, {"title": "Collision diagnosis: " + name}]))
    fig.update_layout(title="Collision diagnosis: " + next(iter(diagnostics)), template="plotly_dark", height=800,
                      scene=dict(aspectmode="data", xaxis_title="base X / m", yaxis_title="base Y / m", zaxis_title="base Z / m"),
                      updatemenus=[dict(buttons=buttons, x=.02, y=1.08)], margin=dict(l=0, r=0, b=0, t=80))
    heading = ("<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>碰撞限位离线核对</title>"
               "<body style='background:#111827;color:#e5e7eb;font:16px sans-serif;padding:20px'>"
               "<h2>碰撞限位：坐标与轨迹离线核对</h2><p>金色为基座/YAW，浅蓝为其余连杆，红色为违反间隙的连杆。"
               "红点表示零间隙网格接触；10 mm 间隙违规与实际网格接触分别记录。该工具不连接或控制硬件。</p>")
    (output / "collision_diagnostics.html").write_text(heading + fig.to_html(full_html=False, include_plotlyjs=True) + "".join(tables) + "</body></html>", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--input", type=Path, nargs="*", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tracks", choices=TRACKS, nargs="+", default=["actual", "ramped"])
    parser.add_argument("--distance-period", type=float, default=.25)
    args = parser.parse_args()
    if not np.isfinite(args.distance_period) or args.distance_period <= 0:
        parser.error("--distance-period must be finite and positive")
    if args.output.exists():
        raise FileExistsError(f"Output already exists; choose a new directory: {args.output}")
    args.output.mkdir(parents=True)
    started = time.monotonic()
    scene = CollisionScene(args.urdf, args.scene, .01)
    examples = {"firmware_home": np.array([0., 2.44, -2.44, 0., 0., 0.]),
                "cad_assembly_pose": np.array(scene.config["cad_pose_joint_rad"])}
    diagnostics = {name: diagnose(scene, q) for name, q in examples.items()}
    (args.output / "pose_diagnostics.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")
    summaries = []
    for index, path in enumerate(args.input):
        summaries.append(replay(scene, path, args.output / (str(index + 1) + "_" + path.stem), args.tracks, args.distance_period, examples))
    diagnostics = {name: diagnose(scene, q) for name, q in examples.items()}
    (args.output / "pose_diagnostics.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")
    publish_diagnostics(scene, args.scene, args.urdf, args.output, diagnostics)
    summary = {"status": "offline_completed", "hardware_io": False, "deployment_status": "not_validated_for_hardware",
               "urdf": str(args.urdf), "urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
               "scene": str(args.scene), "motor_joint_scales": SCALES.tolist(), "motor_joint_offsets_rad": OFFSETS.tolist(),
               "chassis_exempt_links": scene.chassis_exempt_links,
               "chassis_checked_links": [f"Link{i}" for i in scene.chassis_link_indices],
               "external_margin_m": scene.margin, "self_margin_m": .002, "inputs": summaries,
               "collision_queries": scene.calls, "elapsed_seconds": time.monotonic() - started,
               "limitations": ["Geometry and mount alignment are evaluated, not corrected from assumed collision-free recordings",
                               "All checks are model-based; finite path subdivisions do not guarantee continuous collision freedom",
                               "Exact external distances are sampled; Boolean pose checks use every recorded row",
                               "Zero-margin triangle mesh contact does not establish physical contact or penetration depth"]}
    (args.output / "collision_replay_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
