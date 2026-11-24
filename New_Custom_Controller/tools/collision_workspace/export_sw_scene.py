"""Read-only SolidWorks scene extraction for offline collision computation."""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import pythoncom
import win32com.client


def value(obj, name, *args):
    result = getattr(obj, name)
    return result(*args) if callable(result) and not hasattr(result, "_oleobj_") else result


def transform(component):
    tx = component.Transform2
    if tx is None:
        return np.eye(4).tolist()
    data = np.asarray(tx.ArrayData, dtype=float)
    result = np.eye(4)
    result[:3, :3] = data[:9].reshape(3, 3).T * data[12]
    result[:3, 3] = data[9:12]
    return result.tolist()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--export-meshes", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sw = win32com.client.Dispatch("SldWorks.Application")
    error = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warning = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    print("Opening read-only:", args.assembly, flush=True)
    doc = sw.OpenDoc6(str(args.assembly.resolve()), 2, 3, "", error, warning)
    if doc is None:
        raise RuntimeError(f"SolidWorks open failed: error={error.value}, warning={warning.value}")
    print("Resolving lightweight components (in memory only)", flush=True)
    resolved = value(doc, "ResolveAllLightWeightComponents", True)
    print("Resolve status:", resolved, flush=True)
    components = value(doc, "GetComponents", False) or []
    records = []
    cache = {}
    for index, component in enumerate(components):
        name = component.Name2
        record = {"name": name, "path": value(component, "GetPathName"),
                  "configuration": component.ReferencedConfiguration}
        try:
            parent = value(component, "GetParent")
            record["parent"] = parent.Name2 if parent else None
            record["suppression"] = value(component, "GetSuppression")
            record["hidden"] = bool(value(component, "IsHidden", False))
            record["transform"] = transform(component)
            record["box"] = value(component, "GetBox", False, False)
            part = value(component, "GetModelDoc2")
            record["loaded"] = part is not None
            record["doc_type"] = int(value(part, "GetType")) if part is not None else (1 if record["path"].lower().endswith(".sldprt") else 2)
            if args.export_meshes and record["suppression"] != 0 and record["doc_type"] == 1:
                key = (record["path"], record["configuration"])
                if key not in cache:
                    triangles = value(component, "GetTessTriangles", True)
                    if not triangles and part is not None:
                        triangles = value(part, "GetTessTriangles", True)
                    if not triangles:
                        value(component, "SetSuppression2", 2)
                        part = value(component, "GetModelDoc2")
                        if part is not None:
                            triangles = value(part, "GetTessTriangles", True)
                    if not triangles:
                        raise RuntimeError("No tessellated geometry available")
                    vertices = np.asarray(triangles, dtype=np.float32).reshape(-1, 3)
                    mesh_name = f"part_{len(cache):04d}.npz"
                    np.savez_compressed(args.output / mesh_name, vertices=vertices)
                    cache[key] = mesh_name
                record["mesh"] = cache[key]
        except Exception as exc:
            record["error"] = str(exc)
        records.append(record)
        if index % 100 == 0:
            print(f"Components {index}/{len(components)}, meshes {len(cache)}", flush=True)
    result = {"assembly": str(args.assembly.resolve()), "solidworks_revision": value(sw, "RevisionNumber"),
              "open_error": error.value, "open_warning": warning.value,
              "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "components": records}
    path = args.output / "assembly_manifest.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Saved", path, flush=True)
    print("Components", len(records), "unloaded", sum(not r.get("loaded", False) for r in records),
          "errors", sum("error" in r for r in records), "meshes", len(cache), flush=True)
    for record in records:
        if "/" not in record["name"] and (record.get("doc_type") == 2 or "机械臂" in record["name"]):
            print(json.dumps(record, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
