"""将已合并的编码器姿态按更新后的 offset 重新表达；不更改测得力矩。"""

import argparse
import csv
from pathlib import Path
import yaml


def refresh(grouped, old_mapping, new_mapping, output):
    old = yaml.safe_load(Path(old_mapping).read_text())["/**"]["ros__parameters"]
    new = yaml.safe_load(Path(new_mapping).read_text())["/**"]["ros__parameters"]
    for key in ("axis_order", "scales", "torque_signs"):
        if old[key] != new[key]:
            raise ValueError(f"{key} 已改变，需要从原始窗口重新分组")
    with Path(grouped).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields, rows = reader.fieldnames, list(reader)
    for row in rows:
        for i in range(6):
            key = f"q{i + 1}_model_rad"
            row[key] = float(row[key]) + new["offsets"][i] - old["offsets"][i]
    with Path(output).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grouped", required=True)
    parser.add_argument("--old-mapping", required=True)
    parser.add_argument("--new-mapping", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    refresh(args.grouped, args.old_mapping, args.new_mapping, args.output)
