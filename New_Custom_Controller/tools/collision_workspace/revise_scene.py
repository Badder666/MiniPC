"""Apply confirmed physical component exclusions to a separate collision scene."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude-component", action="append", required=True)
    parser.add_argument("--reason", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Output already exists: {args.output}")
    if not args.reason.strip():
        parser.error("--reason must describe the confirmed physical discrepancy")
    exclusions = set(args.exclude_component)
    config_path = args.scene / "scene_config.json"
    meshes_path = args.scene / "scene_meshes.npz"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    obstacles = config["obstacles"]
    for name in exclusions:
        count = sum(part["name"] == name for part in obstacles)
        if count != 1:
            raise ValueError(f"Expected one exact component match for {name!r}; got {count}")
    with np.load(meshes_path) as source:
        meshes = {name: source[name] for name in source.files}
    environment = meshes["environment"]
    if environment.ndim != 2 or environment.shape[1] != 3:
        raise ValueError("Expected an environment triangle soup with shape (N, 3)")
    if any(int(part["triangles"]) <= 0 for part in obstacles):
        raise ValueError("Obstacle triangle counts must be positive")
    if sum(int(part["triangles"]) for part in obstacles) * 3 != len(environment):
        raise ValueError("Obstacle triangle counts do not match the environment mesh")
    kept, removed, chunks = [], [], []
    start = 0
    for part in obstacles:
        end = start + int(part["triangles"]) * 3
        if part["name"] in exclusions:
            removed.append(part)
        else:
            kept.append(part)
            chunks.append(environment[start:end])
        start = end
    if not kept:
        raise ValueError("The revised scene must retain chassis obstacles")
    meshes["environment"] = np.concatenate(chunks)
    config["obstacles"] = kept
    config["environment_instances"] = len(kept)
    config["environment_triangles"] = len(meshes["environment"]) // 3
    config.setdefault("physical_component_exclusions", []).extend(
        dict(part, reason=args.reason) for part in removed)
    config.setdefault("scene_revision_history", []).append({
        "source_scene": str(args.scene.resolve()),
        "source_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "source_meshes_sha256": hashlib.sha256(meshes_path.read_bytes()).hexdigest(),
        "excluded_components": sorted(exclusions),
        "reason": args.reason,
    })
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output / "scene_meshes.npz", **meshes)
    (args.output / "scene_config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({
        "output": str(args.output), "excluded_components": sorted(exclusions),
        "remaining_instances": len(kept),
        "remaining_triangles": config["environment_triangles"],
        "removed_triangles": sum(part["triangles"] for part in removed),
        "reason": args.reason,
    }, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
