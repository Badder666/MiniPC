"""Offline trajectory loading and URDF forward kinematics for RViz."""

from __future__ import annotations

import csv
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .protocol import AXIS_COUNT


_IDENTIFIER = re.compile(r"[^a-z0-9]+")


def _key(value: str) -> str:
    return _IDENTIFIER.sub("", value.strip().lower())


def _number(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        result = float(value.strip())
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _vector(text: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not text:
        return default
    values = tuple(float(value) for value in text.replace(",", " ").split())
    if len(values) != 3:
        raise ValueError(f"需要三个数值，得到 {text!r}")
    return values


def _matmul(left, right):
    return tuple(tuple(sum(left[i][k] * right[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def _identity():
    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = ((1.0, 0.0, 0.0, 0.0), (0.0, cr, -sr, 0.0),
          (0.0, sr, cr, 0.0), (0.0, 0.0, 0.0, 1.0))
    ry = ((cp, 0.0, sp, 0.0), (0.0, 1.0, 0.0, 0.0),
          (-sp, 0.0, cp, 0.0), (0.0, 0.0, 0.0, 1.0))
    rz = ((cy, -sy, 0.0, 0.0), (sy, cy, 0.0, 0.0),
          (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    return _matmul(_matmul(rz, ry), rx)


def _rigid(xyz, rpy):
    matrix = [list(row) for row in _rpy_matrix(rpy)]
    matrix[0][3], matrix[1][3], matrix[2][3] = xyz
    return tuple(tuple(row) for row in matrix)


def _axis_rotation(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    if length <= 1.0e-9:
        raise ValueError("URDF 关节轴长度不能为零")
    x, y, z = (value / length for value in axis)
    sine, cosine = math.sin(angle), math.cos(angle)
    one_minus = 1.0 - cosine
    return ((x * x * one_minus + cosine, x * y * one_minus - z * sine,
             x * z * one_minus + y * sine, 0.0),
            (y * x * one_minus + z * sine, y * y * one_minus + cosine,
             y * z * one_minus - x * sine, 0.0),
            (z * x * one_minus - y * sine, z * y * one_minus + x * sine,
             z * z * one_minus + cosine, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def _point(matrix):
    return (matrix[0][3], matrix[1][3], matrix[2][3])


@dataclass(frozen=True)
class TrajectoryData:
    """Trajectory represented by URDF joint positions and end-effector points."""

    joint_positions: tuple[tuple[float, ...], ...]
    points: tuple[tuple[float, float, float], ...]
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    source: str
    rows_read: int
    rows_skipped: int

    @property
    def count(self) -> int:
        return len(self.points)


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    child: str
    joint_type: str
    origin: tuple
    axis: tuple[float, float, float]


class UrdfKinematics:
    """Minimal serial-chain FK implementation matching robot_state_publisher."""

    def __init__(self, xml_text: str):
        root = ET.fromstring(xml_text)
        links = {element.attrib["name"] for element in root.findall("link")}
        child_links = set()
        self._children: dict[str, list[_Joint]] = {name: [] for name in links}
        revolute = []
        for element in root.findall("joint"):
            origin = element.find("origin")
            axis = element.find("axis")
            joint = _Joint(
                element.attrib["name"],
                element.find("parent").attrib["link"],
                element.find("child").attrib["link"],
                element.attrib.get("type", "fixed"),
                _rigid(_vector(origin.attrib.get("xyz") if origin is not None else None),
                       _vector(origin.attrib.get("rpy") if origin is not None else None)),
                _vector(axis.attrib.get("xyz") if axis is not None else None,
                        (0.0, 0.0, 1.0)),
            )
            self._children[joint.parent].append(joint)
            child_links.add(joint.child)
            if joint.joint_type in ("revolute", "continuous"):
                revolute.append(joint)
        roots = links - child_links
        if len(roots) != 1 or len(revolute) != AXIS_COUNT:
            raise ValueError("URDF 必须包含一个根链接和六个旋转关节")
        self._root = next(iter(roots))
        self._revolute = tuple(revolute)

    def link_transforms(self, positions):
        """Return each link transform in the URDF root frame."""
        if len(positions) != AXIS_COUNT or not all(math.isfinite(v) for v in positions):
            raise ValueError("需要六个有限关节位置")
        values = {joint.name: float(positions[index])
                  for index, joint in enumerate(self._revolute)}
        result = {self._root: _identity()}

        def visit(parent: str, parent_pose):
            for joint in self._children.get(parent, ()):
                transform = joint.origin
                if joint.joint_type in ("revolute", "continuous"):
                    transform = _matmul(transform, _axis_rotation(joint.axis, values[joint.name]))
                pose = _matmul(parent_pose, transform)
                result[joint.child] = pose
                visit(joint.child, pose)

        visit(self._root, _identity())
        return result

    def end_effector(self, positions: tuple[float, ...] | list[float]) -> tuple[float, float, float]:
        transforms = self.link_transforms(positions)
        child = self._revolute[-1].child
        if child not in transforms:
            raise ValueError("URDF 六轴链未连接到末端")
        return _point(transforms[child])


def _columns(fieldnames: list[str], mode: str) -> tuple[str, tuple[str, ...]]:
    by_key = {_key(name): name for name in fieldnames if name}
    controller = tuple(by_key.get(f"i{index}") for index in range(AXIS_COUNT))
    joint = tuple(by_key.get(f"j{index + 1}") for index in range(AXIS_COUNT))
    q_rad = tuple(by_key.get(f"q{index + 1}rad") for index in range(AXIS_COUNT))
    if mode == "controller" and all(controller):
        return "controller", controller
    if mode == "urdf" and (all(q_rad) or all(joint)):
        return "urdf", q_rad if all(q_rad) else joint
    if all(controller):
        return "controller", controller
    if all(q_rad):
        return "urdf", q_rad
    if all(joint):
        # Collision checker output uses J1..J6 as URDF joint coordinates.
        return "urdf", joint
    raise ValueError("CSV 未找到六轴列：需要 I0..I5、J1..J6 或 q1_rad..q6_rad")


def load_trajectory(path: str | Path, xml_text: str, mapper,
                    *, mode="auto", max_points=3000, require_valid_mask=True,
                    require_collision_free=True) -> TrajectoryData:
    """Read VOFA+/collision CSV and convert valid rows to end-effector points."""
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise FileNotFoundError(f"轨迹 CSV 不存在: {file_path}")
    if max_points < 2:
        raise ValueError("max_points 必须至少为 2")
    kinematics = UrdfKinematics(xml_text)
    selected: list[tuple[float, ...]] = []
    rows_read = rows_skipped = 0
    with file_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("CSV 缺少表头")
        input_mode, names = _columns(reader.fieldnames, mode)
        keys = {_key(name): name for name in reader.fieldnames if name}
        valid_column = keys.get("i19") or keys.get("validmotormask") or keys.get("validmask")
        collision_column = keys.get("collisionfree")
        for row in reader:
            rows_read += 1
            values = tuple(_number(row.get(name)) for name in names)
            if any(value is None for value in values):
                rows_skipped += 1
                continue
            if valid_column and require_valid_mask:
                valid_mask = _number(row.get(valid_column))
                if valid_mask is not None and int(round(valid_mask)) != 0x3F:
                    rows_skipped += 1
                    continue
            if collision_column and require_collision_free:
                safe = _number(row.get(collision_column))
                if safe is not None and int(round(safe)) == 0:
                    rows_skipped += 1
                    continue
            if input_mode == "controller":
                joints, valid_mask = mapper.map(values, 0x3F)
                if valid_mask != 0x3F:
                    rows_skipped += 1
                    continue
            else:
                joints = values
            try:
                point = kinematics.end_effector(joints)
            except (ValueError, OverflowError):
                rows_skipped += 1
                continue
            selected.append(tuple(joints))

    if not selected:
        raise ValueError(f"CSV 没有可显示的有效轨迹点（读取 {rows_read} 行，跳过 {rows_skipped} 行）")
    if len(selected) > max_points:
        # Uniform decimation preserves the whole forward/reverse path.
        indices = [round(index * (len(selected) - 1) / (max_points - 1))
                   for index in range(max_points)]
        selected = [selected[index] for index in indices]
    points = tuple(kinematics.end_effector(joints) for joints in selected)
    minimum = tuple(min(point[index] for point in points) for index in range(3))
    maximum = tuple(max(point[index] for point in points) for index in range(3))
    return TrajectoryData(tuple(selected), points, minimum, maximum,
                          str(file_path), rows_read, rows_skipped)
