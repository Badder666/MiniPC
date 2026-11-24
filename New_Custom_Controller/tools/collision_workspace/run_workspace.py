"""Offline sampled workspace and home-connected roadmap; no hardware I/O."""

import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import time

import coal
import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull, cKDTree
from scipy.stats import qmc


def make_bvh(vertices):
    vertices = np.asarray(vertices, dtype=np.float64)
    points = coal.StdVec_Vec3s()
    points.extend(vertices)
    triangles = coal.StdVec_Triangle()
    triangles.extend(coal.Triangle(i, i + 1, i + 2) for i in range(0, len(vertices), 3))
    geometry = coal.BVHModelOBBRSS()
    geometry.beginModel(len(vertices) // 3, len(vertices))
    geometry.addSubModel(points, triangles)
    geometry.endModel()
    return geometry


class CollisionScene:
    def __init__(self, urdf, scene_dir, margin):
        self.config = json.loads((scene_dir / "scene_config.json").read_text(encoding="utf-8"))
        self.model, self.geometry, _ = pin.buildModelsFromUrdf(str(urdf), package_dirs=[str(urdf.parent.parent.parent)])
        environment = np.load(scene_dir / "scene_meshes.npz")["environment"]
        print("Building chassis collision tree", environment.shape, flush=True)
        chassis = make_bvh(environment)
        self.chassis_index = self.geometry.addGeometryObject(pin.GeometryObject("chassis", 0, pin.SE3.Identity(), chassis))
        self.ground_z = self.config.get("ground_z_m")
        self.ground_support = []
        for name in ["base_link"] + [f"Link{i}" for i in range(1, 7)]:
            with (urdf.parent.parent / "meshes" / (name + ".STL")).open("rb") as stream:
                stream.seek(80)
                count = struct.unpack("<I", stream.read(4))[0]
                dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attribute", "<u2")])
                vertices = np.unique(np.fromfile(stream, dtype=dtype, count=count)["v"].reshape(-1, 3), axis=0).astype(float)
            self.ground_support.append(vertices[ConvexHull(vertices).vertices])
        self.geometry.removeAllCollisionPairs()
        self.pair_labels = []
        # The fixed base and J1/YAW form the original chassis mounting interface.
        # This exemption applies only to chassis contact, never to other links.
        self.chassis_exempt_links = ["base_link", "Link1"]
        self.chassis_link_indices = list(range(2, 7))
        for i, name in enumerate(["base_link"] + [f"Link{j}" for j in range(1, 7)]):
            if self.geometry.geometryObjects[i].name != name + "_0":
                raise RuntimeError("Unexpected URDF geometry order; inspect link collision mapping")
        # Only directly connected links are excluded; no copied 'Never' rules.
        for i in range(7):
            for j in range(i + 2, 7):
                self.add_pair(i, j)
        for i in self.chassis_link_indices:
            self.add_pair(i, self.chassis_index)
        self.data = self.model.createData()
        self.gdata = pin.GeometryData(self.geometry)
        for pair, request in zip(self.geometry.collisionPairs, self.gdata.collisionRequests):
            request.security_margin = min(margin, .002) if pair.second < 7 else margin
            request.num_max_contacts = 1
        self.frame = self.model.getFrameId("Link6")
        self.margin = margin
        self.calls = 0

    def add_pair(self, i, j):
        self.geometry.addCollisionPair(pin.CollisionPair(i, j))
        self.pair_labels.append(self.geometry.geometryObjects[i].name + "/" + self.geometry.geometryObjects[j].name)

    def valid(self, q, explain=False):
        self.calls += 1
        collided = pin.computeCollisions(self.model, self.data, self.geometry, self.gdata, q, not explain)
        ground_pairs = []
        if self.ground_z is not None and (not collided or explain):
            for i in range(1, 7):
                placement = self.gdata.oMg[i]
                # Linear support over a triangle mesh equals support over its
                # convex hull: this ground check is exact for the supplied mesh.
                minimum_z = np.min(self.ground_support[i] @ placement.rotation[2]) + placement.translation[2]
                if minimum_z - self.ground_z <= self.margin:
                    ground_pairs.append(self.geometry.geometryObjects[i].name + "/ground")
                    if not explain:
                        break
        if explain:
            return [self.pair_labels[i] for i, result in enumerate(self.gdata.collisionResults) if result.isCollision()] + ground_pairs
        return not collided and not ground_pairs

    def poses(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)
        placement = self.data.oMf[self.frame]
        return (placement.translation.copy(), placement.act(np.asarray(self.config["tcp_translation_m"])),
                pin.Quaternion(placement.rotation).coeffs().copy())

    def clearance(self, q):
        pin.computeDistances(self.model, self.data, self.geometry, self.gdata, q)
        distances = np.asarray([result.min_distance for result in self.gdata.distanceResults])
        closest = int(np.argmin(distances))
        result = float(distances[closest]), self.pair_labels[closest]
        if self.ground_z is not None:
            for i in range(1, 7):
                placement = self.gdata.oMg[i]
                distance = float(np.min(self.ground_support[i] @ placement.rotation[2]) + placement.translation[2] - self.ground_z)
                if distance < result[0]:
                    result = distance, self.geometry.geometryObjects[i].name + "/ground"
        return result

    def edge(self, start, end):
        delta = end - start
        # Finite edge sampling, explicitly recorded in output. The weighted
        # angular bound also limits coarse TCP/geometry translation between samples.
        steps = max(1, int(np.ceil(np.max(np.abs(delta)) / np.deg2rad(1.))),
                    int(np.ceil(np.dot(np.abs(delta), [1.1, 1.1, .8, .4, .4, .3]) / .005)))
        for index in range(1, steps + 1):
            if not self.valid(start + delta * (index / steps)):
                return False
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--samples-power", type=int, default=15)
    parser.add_argument("--margin", type=float, default=.01)
    parser.add_argument("--voxel", type=float, default=.02)
    parser.add_argument("--roadmap-nodes", type=int, default=1500)
    parser.add_argument("--diagnose", action="store_true")
    parser.add_argument("--reuse-samples", action="store_true")
    args = parser.parse_args()
    started = time.monotonic()
    scene = CollisionScene(args.urdf, args.scene, args.margin)
    home = np.array([0., 2.44, -2.44, 0., 0., 0.])
    cad_pose = np.array(scene.config["cad_pose_joint_rad"])
    diagnostics = {name: {"q_deg": np.rad2deg(q).tolist(), "colliding_pairs": scene.valid(q, explain=True),
                           "minimum_clearance": scene.clearance(q)}
                   for name, q in [("firmware_home", home), ("cad_pose", cad_pose)]}
    print(json.dumps({"diagnostics": diagnostics, "build_seconds": time.monotonic() - started}, indent=2), flush=True)
    if args.diagnose:
        engine = qmc.Sobol(6, scramble=True, seed=20261114)
        candidates = qmc.scale(engine.random_base2(7), scene.model.lowerPositionLimit, scene.model.upperPositionLimit)
        tick = time.monotonic()
        labels = [scene.valid(q, explain=True) for q in candidates[:16]]
        tick = time.monotonic()
        flags = [scene.valid(q) for q in candidates]
        count = sum(flags)
        print("Benchmark 128 states", count, "valid seconds", time.monotonic() - tick,
              "pair_rejections", dict(Counter(label for row in labels for label in row)), flush=True)
        return
    seed_name = "firmware_home"
    seed = home
    if not scene.valid(seed):
        seed_name = "cad_assembly_pose"
        seed = cad_pose
    if not scene.valid(seed):
        raise RuntimeError("Neither firmware home nor CAD assembly pose is valid; inspect the model before sampling paths")
    candidates = (np.load(args.scene / "free_configurations.npz")["q"] if args.reuse_samples else
                  qmc.scale(qmc.Sobol(6, scramble=True, seed=20261114).random_base2(args.samples_power),
                            scene.model.lowerPositionLimit, scene.model.upperPositionLimit))
    accepted, xyz, wrist, quat = [], [], [], []
    rejected = 0
    for i, q in enumerate(candidates):
        if scene.valid(q):
            p, t, rotation = scene.poses(q)
            accepted.append(q)
            wrist.append(p)
            xyz.append(t)
            quat.append(rotation)
        else:
            rejected += 1
        if (i + 1) % 2048 == 0:
            print(f"Sampling {i+1}/{len(candidates)} accepted={len(accepted)} elapsed={time.monotonic()-started:.1f}s", flush=True)
    accepted = np.asarray(accepted)
    xyz = np.asarray(xyz)
    wrist = np.asarray(wrist)
    quat = np.asarray(quat)
    if len(accepted) == 0:
        raise RuntimeError("No valid configurations")
    np.savez_compressed(args.scene / "free_configurations.npz", q=accepted, tcp=xyz, wrist=wrist, quaternion_xyzw=quat)
    nodes = [seed.copy()]
    parents = [-1]
    rng = np.random.default_rng(20261114)
    failures = 0
    attempts = 0
    while len(nodes) < args.roadmap_nodes and attempts < args.roadmap_nodes * 12:
        attempts += 1
        if len(nodes) > 1 and rng.random() < .65:
            parent = int(rng.integers(len(nodes)))
            target = nodes[parent] + rng.normal(0., .18, 6)
            target = np.clip(target, scene.model.lowerPositionLimit, scene.model.upperPositionLimit)
        else:
            target = accepted[int(rng.integers(len(accepted)))]
            parent = int(cKDTree(nodes).query(target)[1])
        delta = target - nodes[parent]
        next_q = nodes[parent] + delta * min(1., .14 / max(np.max(np.abs(delta)), 1e-9))
        if np.max(np.abs(next_q - nodes[parent])) < 1e-5:
            continue
        if scene.valid(next_q) and scene.edge(nodes[parent], next_q):
            nodes.append(next_q)
            parents.append(parent)
        else:
            failures += 1
        if len(nodes) % 200 == 0 and len(nodes) > 1:
            print(f"Connected roadmap nodes={len(nodes)} attempts={attempts} calls={scene.calls} elapsed={time.monotonic()-started:.1f}s", flush=True)
    nodes = np.asarray(nodes)
    node_xyz, node_quat = [], []
    for q in nodes:
        _, tcp, rotation = scene.poses(q)
        node_xyz.append(tcp)
        node_quat.append(rotation)
    node_xyz = np.asarray(node_xyz)
    np.savez_compressed(args.scene / "seed_connected.npz", q=nodes, parent=np.asarray(parents), tcp=node_xyz,
                        quaternion_xyzw=np.asarray(node_quat), seed=seed)
    voxel_indices, witness = np.unique(np.floor(xyz / args.voxel).astype(np.int32), axis=0, return_index=True)
    connected_voxels = np.unique(np.floor(node_xyz / args.voxel).astype(np.int32), axis=0)
    summary = {"status": "preliminary_model_based_workspace", "samples": len(candidates), "accepted": len(accepted),
               "rejected": rejected, "voxel_size_m": args.voxel, "occupied_voxels": len(witness),
               "occupied_voxel_volume_m3": len(witness) * args.voxel ** 3,
               "tcp_bounds_m": [xyz.min(0).tolist(), xyz.max(0).tolist()],
               "external_clearance_threshold_m": args.margin, "self_clearance_threshold_m": min(args.margin, .002),
               "connected_seed": seed_name, "seed_joint_rad": seed.tolist(), "seed_connected_nodes": len(nodes),
               "seed_connected_voxels": len(connected_voxels), "roadmap_attempts": attempts,
               "roadmap_rejected_edges": failures, "collision_queries": scene.calls,
               "edge_check": {"maximum_joint_step_deg": 1., "weighted_motion_step_m": .005,
                              "method": "finite subdivision; no continuous collision guarantee"},
               "home_joint_rad": home.tolist(), "diagnostics": diagnostics,
               "elapsed_seconds": time.monotonic() - started,
               "pinocchio_version": pin.__version__, "coal_version": coal.__version__,
               "chassis_collision_geometry": "original CAD surface mesh",
               "collision_policy": {
                   "chassis_contact_exempt_links": scene.chassis_exempt_links,
                   "chassis_checked_links": [f"Link{i}" for i in scene.chassis_link_indices],
                   "checked_chassis_pairs": [scene.pair_labels[i] for i, pair in enumerate(scene.geometry.collisionPairs)
                                             if pair.second == scene.chassis_index],
                   "self_collision": "all non-adjacent arm links, including base and YAW",
                   "motion_check": "full link meshes at every sampled path configuration",
                   "point_cloud": "TCP projection of valid full-arm configurations; not a TCP-only collision check"},
               "limitations": scene.config["limitations"] + ["Finite sampling approximates the configuration space",
                    "Voxel volume counts observed occupied cells; not exact workspace volume",
                    "A voxel contains witness configurations; it does not imply all orientations are valid",
                    "Only roadmap nodes have checked paths from the named seed; other accepted samples are pose-only",
                    "Firmware home is not assumed collision-free; CAD assembly pose is used when home is invalid",
                    "Self-collision clearance is 2mm to retain the wrist's existing 6mm assembly gap; external clearance is 10mm by default"]}
    (args.scene / "workspace_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    np.savez_compressed(args.scene / "workspace_voxels.npz", index=voxel_indices, q=accepted[witness], tcp=xyz[witness],
                        quaternion_xyzw=quat[witness], voxel_size=args.voxel)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
