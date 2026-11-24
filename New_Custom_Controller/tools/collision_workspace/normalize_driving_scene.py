"""Build the confirmed four-wheel driving pose from the mixed-pose CAD scene."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np


def family(record):
    stem = Path(record["path"]).stem.replace("Mirror", "")
    return re.sub(r"5\.06\.\d+$", "5.06", stem)


def origin(record):
    return np.asarray(record["transform"])[:3, 3]


def movement_groups(records):
    active = [r for r in records if r.get("suppression") != 0]
    roots = [r for r in active if "/" not in r["name"] and "上台阶机构" in r["name"]]
    if len(roots) != 4:
        raise ValueError(f"Expected four active lift assemblies, got {len(roots)}")
    roots.sort(key=lambda r: (r["box"][0] + r["box"][3]) / 2)
    rear = sorted(roots[:2], key=lambda r: r["box"][2])
    front = sorted(roots[2:], key=lambda r: r["box"][2])
    groups, pairings = {}, []
    wheel_hardware = {"轮组加强件-铜柱-M3-65和垫片-0.2", "M5-6-8塞打",
                      "F6803法兰轴承", "轮组外固定-机加工", "轮组轴承内固定-TPU"}
    for back, reference in zip(rear, front):
        children = [r for r in active if r.get("parent") == back["name"]]
        candidates = [r for r in active if r.get("parent") == reference["name"]]
        tube = next(r for r in children if "30-30-2.5铝方管" in r["name"])
        reference_tube = next(r for r in candidates if "30-30-2.5铝方管" in r["name"])
        travel = float(origin(reference_tube)[1] - origin(tube)[1])
        mirror_x_sum = float(origin(reference_tube)[0] + origin(tube)[0])
        if not np.isclose(travel, .197, atol=1e-7):
            raise ValueError(f"Unexpected lift travel {travel}; inspect this CAD revision")
        classifications = Counter()
        for part in children:
            xyz = origin(part)
            matches = []
            for candidate in candidates:
                if family(candidate) != family(part):
                    continue
                for dy in [0., travel]:
                    expected = np.array([mirror_x_sum - xyz[0], xyz[1] + dy, xyz[2]])
                    matches.append((float(np.linalg.norm(origin(candidate) - expected)),
                                    dy, candidate["name"]))
            matches.sort()
            if matches and matches[0][0] <= .0031:
                error, dy, match = matches[0]
                evidence = {"method": "mirrored counterpart origin", "reference_component": match,
                            "origin_residual_mm": error * 1000}
            elif family(part).removeprefix("V3-") in wheel_hardware:
                dy = travel
                evidence = {"method": "wheel axle/bracing hardware at moving wheel-arm datum"}
                if not -.27 < xyz[1] < -.17:
                    raise ValueError(f"Unexpected wheel hardware location: {part['name']}")
            elif "存矿机构机加工-6061" in part["name"]:
                dy = travel
                evidence = {"method": "rail-top fixture; origin shares the moving SGR rail upper end"}
                if not np.isclose(xyz[1], origin(tube)[1] - .01, atol=1e-6):
                    raise ValueError("Rail-top fixture no longer matches the tube datum")
            else:
                raise ValueError(f"Unclassified lift component: {part['name']}")
            classifications["moving" if dy else "fixed"] += 1
            groups[part["name"]] = {"translation_solidworks_m": [0., dy, 0.], **evidence}
        pairings.append({"adjusted_assembly": back["name"], "reference_assembly": reference["name"],
                         "travel_m": travel, "classification_counts": dict(classifications)})
    travel = pairings[0]["travel_m"]
    if not np.isclose(travel, pairings[1]["travel_m"], atol=1e-7):
        raise ValueError("The rear lift displacements disagree")
    rfid_attachments = {"V3-26官方-场地交互模块FI0213.0-2.0-3.0-5.06.0-1",
                        "V3-底盘-RFID连接板-2.0-3.0-5.06.0-1"}
    rfid_attachments.update(f"V3-M3-30铝柱-2.0-3.0-5.06.0-{i}" for i in [1, 2, 7, 8])
    for part in active:
        if "/" in part["name"] or origin(part)[0] >= .765:
            continue
        if "后轮拖链固定件" in part["name"]:
            groups[part["name"]] = {"translation_solidworks_m": [0., travel, 0.],
                                    "method": "moving rear cable-carrier end bracket"}
        elif "rfid超级钣金" in part["name"]:
            groups[part["name"]] = {"translation_solidworks_m": [0., travel, 0.],
                                    "method": "cross-bracket attached to the two moving rear wheel arms"}
        elif part["name"] in rfid_attachments:
            groups[part["name"]] = {"translation_solidworks_m": [0., travel, 0.],
                                    "method": "RFID board and four pillars attached below the moving RFID cross-bracket"}
        elif "坦克拖链" in part["name"] and not part["name"].endswith("-5"):
            groups[part["name"]] = {"translation_solidworks_m": [0., travel, 0.],
                                    "method": "nominal rigid carrier proxy follows its moving end bracket",
                                    "requires_shape_check": True}
    return groups, pairings, travel


def surface(vertices, count, seed):
    triangles = vertices.reshape(-1, 3, 3)
    if len(triangles) <= count:
        return triangles
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1)
    score = np.maximum(area, 1e-18) / np.random.default_rng(seed).exponential(size=len(area))
    return triangles[np.argpartition(score, -count)[-count:]]


def preview(output, before, after, config, moved_names):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    chunks, start = {}, 0
    for part in config["obstacles"]:
        end = start + part["triangles"] * 3
        chunks[part["name"]] = after[start:end]
        start = end
    carriers = {name: v for name, v in chunks.items() if "坦克拖链" in name}
    fixed = np.concatenate([v for name, v in chunks.items() if name not in moved_names and name not in carriers])
    moved = np.concatenate([v for name, v in chunks.items() if name in moved_names and name not in carriers])
    arm = np.load(output / "scene_meshes.npz")["cad_arm"]
    fig = plt.figure(figsize=(16, 8))
    axes = [fig.add_subplot(1, 3, 1), fig.add_subplot(1, 3, 2), fig.add_subplot(1, 3, 3, projection="3d")]
    sets = [(surface(before, 35000, 1), "#8899aa"),
            (surface(arm, 15000, 2), "#d49b50")]
    for triangles, color in sets:
        axes[0].add_collection(PolyCollection(triangles[:, :, [1, 2]], facecolors=color,
                                              edgecolors="none", rasterized=True))
    layers = [(surface(fixed, 28000, 3), "#8899aa"),
              (surface(moved, 14000, 4), "#219aa4"),
              (surface(arm, 15000, 2), "#d49b50")]
    for index, (name, v) in enumerate(sorted(carriers.items())):
        layers.append((v.reshape(-1, 3, 3), "#b854bb"))
        center = (v.min(0) + v.max(0)) / 2
        axes[2].text(*center, name.rsplit("-", 1)[-1], fontsize=12, color="#741d79")
    for triangles, color in layers:
        axes[1].add_collection(PolyCollection(triangles[:, :, [1, 2]], facecolors=color,
                                              edgecolors="none", rasterized=True))
        axes[2].add_collection3d(Poly3DCollection(triangles, facecolors=color, edgecolors="none",
                                                rasterized=True, zsort="average"))
    axes[0].set_title("原始装配：两侧升降高度混合")
    axes[1].set_title("正常行驶零位：四轮高度统一")
    combined = np.concatenate([before[::30], after[::30], arm[::20]])
    lower, upper = combined.min(0), combined.max(0)
    for index, ax in enumerate(axes[:2]):
        ax.set_xlim(lower[1] - .04, upper[1] + .04)
        ax.set_ylim(lower[2] - .04, upper[2] + .04)
        ground = config["ground_z_m"] - (config["driving_zero"]["rear_lift_translation_m"] if index == 0 else 0.)
        ax.axhline(ground, color="#555555", linestyle="--", linewidth=.8)
        ax.set_aspect("equal"); ax.set_xlabel("base Y / m"); ax.set_ylabel("base Z / m")
        ax.grid(alpha=.15)
    ax = axes[2]
    ax.set_title("零位三维预览：紫色为四条拖链")
    ax.set_xlim(lower[0] - .03, upper[0] + .03)
    ax.set_ylim(lower[1] - .03, upper[1] + .03)
    ax.set_zlim(lower[2] - .03, upper[2] + .03)
    ax.set_box_aspect(upper - lower)
    ax.view_init(elev=20, azim=25)
    ax.set_xlabel("base X / m"); ax.set_ylabel("base Y / m"); ax.set_zlabel("base Z / m")
    fig.suptitle("整车正常行驶零位场景核对（机械臂保持 CAD 装配姿态）", fontsize=15)
    fig.text(.5, .035, "蓝绿色：平移后的后侧可动件    灰色：其余车体    金色：机械臂    紫色：拖链\n"
             "后侧拖链使用随端点平移的名义几何；实际弯曲形状尚需核对。", ha="center", fontsize=10)
    fig.tight_layout(rect=[0, .08, 1, .95])
    fig.savefig(output / "driving_zero_preview.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Output already exists: {args.output}")
    manifest_path = args.cad / "assembly_manifest.json"
    records = json.loads(manifest_path.read_text(encoding="utf-8"))["components"]
    groups, pairings, travel = movement_groups(records)
    config_path = args.scene / "scene_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    rotation = np.linalg.inv(np.asarray(config["base_to_solidworks"]))[:3, :3]
    with np.load(args.scene / "scene_meshes.npz") as source:
        meshes = {name: source[name] for name in source.files}
    before = meshes["environment"]
    if sum(part["triangles"] for part in config["obstacles"]) * 3 != len(before):
        raise ValueError("Obstacle metadata and mesh triangle counts disagree")
    chunks, obstacles, movements, references = [], [], [], []
    start = 0
    for part in config["obstacles"]:
        end = start + part["triangles"] * 3
        vertices = before[start:end]
        start = end
        name = part["name"]
        if "/" not in name and "坦克拖链" in name and name.endswith("-5"):
            references.append(dict(part, reason="Alternate-state carrier reference; this scene contains four physical carriers"))
            continue
        branches = [branch for branch in groups if name == branch or name.startswith(branch + "/")]
        if len(branches) > 1:
            raise ValueError(f"Overlapping movement classifications for {name}")
        if branches:
            group = groups[branches[0]]
            delta = rotation @ np.asarray(group["translation_solidworks_m"])
            if np.linalg.norm(delta) > 0.:
                vertices = vertices + delta
                movements.append({"component": name, "branch": branches[0],
                                  "translation_base_m": delta.tolist(), **group})
        chunks.append(vertices)
        obstacles.append(dict(part, bounds=[vertices.min(0).tolist(), vertices.max(0).tolist()]))
    carriers = [part for part in obstacles if "坦克拖链" in part["name"]]
    if len(carriers) != 4 or len(references) != 1:
        raise ValueError("Expected four carriers and one alternate-state reference")
    meshes["environment"] = np.concatenate(chunks).astype(np.float32)
    config.update(obstacles=obstacles, environment_instances=len(obstacles),
                  environment_triangles=len(meshes["environment"]) // 3,
                  scope="Four-wheel normal-driving zero pose derived from the confirmed right-side CAD pose; offline scene under review")
    original_ground = config.get("ground_z_m")
    if original_ground is not None:
        config["ground_z_m"] = original_ground + float((rotation @ [0., travel, 0.])[2])
    component_meshes = {part["name"]: vertices for part, vertices in zip(obstacles, chunks)}
    wheel_bottoms = []
    for record in records:
        if (record.get("suppression") != 0 and record.get("doc_type") == 2
                and "装配联轴器26" in Path(record["path"]).stem):
            wheel_parts = [v for name, v in component_meshes.items() if name.startswith(record["name"] + "/")]
            wheel_bottoms.append({"assembly": record["name"],
                                  "bottom_z_m": min(float(v[:, 2].min()) for v in wheel_parts)})
    if len(wheel_bottoms) != 4 or np.ptp([wheel["bottom_z_m"] for wheel in wheel_bottoms]) > 1e-5:
        raise ValueError("The four wheel contact heights are not aligned")
    if original_ground is not None and meshes["environment"][:, 2].min() < config["ground_z_m"] - .002:
        raise ValueError("A chassis component remains below the normalized driving ground plane")
    config["driving_zero"] = {
        "reference": "User-confirmed right-side wheel group in codex-clipboard-d855a2b1-02a5-44c5-a7a1-b70ae2908933.png",
        "rear_lift_translation_m": travel, "paired_assemblies": pairings,
        "classified_branches": groups, "transformed_components": movements,
        "alternate_state_references": references,
        "wheel_bottoms": wheel_bottoms,
        "original_ground_z_m": original_ground,
        "ground_derivation": "Preserve original ground-to-wheel gap after aligning all four wheel contact heights",
        "carrier_geometry": "Rigid nominal CAD carrier geometry follows rear moving end brackets by 197 mm; real articulation not reconstructed",
        "ready_for_hardware_limit": False,
        "remaining_geometry_checks": ["Rear cable-carrier bending shape and mounting details",
                                      "Current CAD arm versus old URDF Link2 geometry discrepancy"],
    }
    config["limitations"] = [entry for entry in config.get("limitations", [])
                             if entry != "Chassis and step-climbing mechanisms remain in the CAD assembly pose"]
    config["limitations"].extend(config["driving_zero"]["remaining_geometry_checks"])
    config.setdefault("scene_revision_history", []).append({
        "source_scene": str(args.scene.resolve()),
        "source_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "source_meshes_sha256": hashlib.sha256((args.scene / "scene_meshes.npz").read_bytes()).hexdigest(),
        "cad_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "operation": "Normalize four lift assemblies to the user-confirmed normal-driving zero pose",
    })
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output / "scene_meshes.npz", **meshes)
    (args.output / "scene_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    preview(args.output, before, meshes["environment"], config, {item["component"] for item in movements})
    print(json.dumps({"output": str(args.output), "rear_travel_mm": travel * 1000,
                      "transformed_components": len(movements), "environment_instances": len(obstacles),
                      "environment_triangles": config["environment_triangles"],
                      "ground_z_m": config["ground_z_m"], "carrier_instances": len(carriers),
                      "pairings": pairings}, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
