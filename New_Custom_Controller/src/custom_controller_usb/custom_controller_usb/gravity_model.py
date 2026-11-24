"""Static gravity torque estimation from a serial-chain URDF.

The model intentionally uses only the URDF mass and center-of-mass fields.
It estimates the torque required to hold a pose against gravity; velocity,
acceleration, friction, gearbox losses, and motor inertia are outside this
first approximation.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

Vector = tuple[float, float, float]
Matrix = tuple[tuple[float, float, float],
               tuple[float, float, float],
               tuple[float, float, float]]


def _vec_add(a: Vector, b: Vector) -> Vector:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _vec_sub(a: Vector, b: Vector) -> Vector:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _vec_scale(a: Vector, scale: float) -> Vector:
    return a[0] * scale, a[1] * scale, a[2] * scale


def _dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vector, b: Vector) -> Vector:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a: Vector) -> float:
    return math.sqrt(_dot(a, a))


def _normalize(a: Vector) -> Vector:
    length = _norm(a)
    if length <= 1.0e-12:
        raise ValueError("joint axis must be non-zero")
    return _vec_scale(a, 1.0 / length)


def _mat_mul(a: Matrix, b: Matrix) -> Matrix:
    return tuple(tuple(sum(a[row][k] * b[k][col] for k in range(3))
                       for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _mat_vec(a: Matrix, v: Vector) -> Vector:
    return tuple(sum(a[row][k] * v[k] for k in range(3))
                 for row in range(3))  # type: ignore[return-value]


def _identity() -> Matrix:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """URDF fixed-axis RPY rotation, equivalent to Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_matrix(axis: Vector, angle: float) -> Matrix:
    x, y, z = _normalize(axis)
    c, s, one_minus_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (c + x * x * one_minus_c,
         x * y * one_minus_c - z * s,
         x * z * one_minus_c + y * s),
        (y * x * one_minus_c + z * s,
         c + y * y * one_minus_c,
         y * z * one_minus_c - x * s),
        (z * x * one_minus_c - y * s,
         z * y * one_minus_c + x * s,
         c + z * z * one_minus_c),
    )


@dataclass(frozen=True)
class _Link:
    name: str
    mass: float
    center_of_mass: Vector


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    child: str
    origin_xyz: Vector
    origin_rpy: Vector
    axis: Vector


@dataclass(frozen=True)
class _Pose:
    rotation: Matrix
    position: Vector


class SerialUrdfGravityModel:
    """Gravity model for a tree containing one six-joint serial chain."""

    def __init__(self, links: dict[str, _Link], joints: tuple[_Joint, ...], gravity: Vector):
        if len(joints) != 6:
            raise ValueError(f"expected six movable joints, found {len(joints)}")
        self._links = links
        self.joints = joints
        self.gravity = gravity
        self._joint_index = {joint.name: index for index, joint in enumerate(joints)}
        self._ancestors: dict[str, tuple[int, ...]] = {}
        for link in links:
            self._ancestors[link] = self._find_ancestors(link, joints)

    @classmethod
    def from_urdf(cls, path: str | Path, gravity: Vector = (0.0, 0.0, -9.81)) -> "SerialUrdfGravityModel":
        root = ET.parse(path).getroot()
        if root.tag != "robot":
            raise ValueError("URDF root element must be <robot>")

        links: dict[str, _Link] = {}
        for element in root.findall("link"):
            name = element.attrib.get("name")
            if not name:
                raise ValueError("link without a name")
            inertial = element.find("inertial")
            if inertial is None:
                # Fixed geometry with no mass contributes no gravity torque.
                links[name] = _Link(name, 0.0, (0.0, 0.0, 0.0))
                continue
            mass_element = inertial.find("mass")
            if mass_element is None:
                raise ValueError(f"link {name} has inertial data without mass")
            mass = float(mass_element.attrib["value"])
            origin = inertial.find("origin")
            center = _parse_xyz(origin, "xyz") if origin is not None else (0.0, 0.0, 0.0)
            if not math.isfinite(mass) or mass < 0.0:
                raise ValueError(f"invalid mass on link {name}")
            links[name] = _Link(name, mass, center)

        joints: list[_Joint] = []
        for element in root.findall("joint"):
            if element.attrib.get("type") not in ("revolute", "continuous"):
                continue
            parent_element, child_element = element.find("parent"), element.find("child")
            axis_element = element.find("axis")
            if parent_element is None or child_element is None or axis_element is None:
                raise ValueError(f"joint {element.attrib.get('name', '?')} is incomplete")
            origin = element.find("origin")
            xyz = _parse_xyz(origin, "xyz") if origin is not None else (0.0, 0.0, 0.0)
            rpy = _parse_xyz(origin, "rpy") if origin is not None else (0.0, 0.0, 0.0)
            joints.append(_Joint(
                element.attrib.get("name", f"joint_{len(joints) + 1}"),
                parent_element.attrib["link"], child_element.attrib["link"],
                xyz, rpy, _parse_xyz(axis_element, "xyz")))

        ordered = _order_serial_chain(links, joints)
        return cls(links, tuple(ordered), gravity)

    def _find_ancestors(self, link_name: str, joints: tuple[_Joint, ...]) -> tuple[int, ...]:
        ancestors: list[int] = []
        current = link_name
        while True:
            parent = next((index for index, joint in enumerate(joints) if joint.child == current), None)
            if parent is None:
                break
            ancestors.append(parent)
            current = joints[parent].parent
        return tuple(reversed(ancestors))

    def gravity_torques(self, joint_positions: Iterable[float]) -> tuple[float, ...]:
        q = tuple(float(value) for value in joint_positions)
        if len(q) != len(self.joints) or not all(math.isfinite(value) for value in q):
            raise ValueError("joint_positions must contain six finite radians")

        poses: dict[str, _Pose] = {}
        joint_positions_world: list[Vector] = []
        joint_axes_world: list[Vector] = []
        root_candidates = set(self._links) - {joint.child for joint in self.joints}
        if len(root_candidates) != 1:
            raise ValueError("URDF must contain one root link for gravity calculation")
        root = next(iter(root_candidates))
        poses[root] = _Pose(_identity(), (0.0, 0.0, 0.0))

        for index, joint in enumerate(self.joints):
            parent_pose = poses.get(joint.parent)
            if parent_pose is None:
                raise ValueError(f"joint chain is disconnected at {joint.name}")
            origin_rotation = _rpy_matrix(*joint.origin_rpy)
            joint_rotation = _mat_mul(parent_pose.rotation, origin_rotation)
            joint_position = _vec_add(parent_pose.position,
                                      _mat_vec(parent_pose.rotation, joint.origin_xyz))
            axis_world = _normalize(_mat_vec(joint_rotation, joint.axis))
            child_rotation = _mat_mul(joint_rotation, _axis_matrix(joint.axis, q[index]))
            poses[joint.child] = _Pose(child_rotation, joint_position)
            joint_positions_world.append(joint_position)
            joint_axes_world.append(axis_world)

        torques = [0.0] * len(self.joints)
        for link_name, link in self._links.items():
            if link.mass <= 0.0 or link_name not in poses:
                continue
            com_world = _vec_add(poses[link_name].position,
                                 _mat_vec(poses[link_name].rotation, link.center_of_mass))
            force = _vec_scale(self.gravity, link.mass)
            for joint_index in self._ancestors[link_name]:
                lever = _vec_sub(com_world, joint_positions_world[joint_index])
                torques[joint_index] += _dot(joint_axes_world[joint_index], _cross(lever, force))
        return tuple(torques)

    def gravity_torques_for_motor_positions(self, motor_positions: Iterable[float],
                                            scales: Iterable[float] = (1.0,) * 6,
                                            offsets: Iterable[float] = (0.0,) * 6) -> tuple[float, ...]:
        """Apply the same joint = scale * motor + offset mapping as the bridge."""
        motors = tuple(float(value) for value in motor_positions)
        scale_values = tuple(float(value) for value in scales)
        offset_values = tuple(float(value) for value in offsets)
        if len(motors) != 6 or len(scale_values) != 6 or len(offset_values) != 6:
            raise ValueError("motor, scale, and offset arrays must contain six values")
        joints = tuple(scale_values[i] * motors[i] + offset_values[i] for i in range(6))
        return self.gravity_torques(joints)


def _parse_xyz(element: ET.Element | None, attribute: str) -> Vector:
    if element is None:
        return (0.0, 0.0, 0.0)
    values = tuple(float(value) for value in element.attrib.get(attribute, "0 0 0").split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{attribute} must contain three finite numbers")
    return values  # type: ignore[return-value]


def _order_serial_chain(links: dict[str, _Link], joints: list[_Joint]) -> list[_Joint]:
    roots = set(links) - {joint.child for joint in joints}
    if len(roots) != 1:
        raise ValueError("URDF must contain one root link")
    ordered: list[_Joint] = []
    current = next(iter(roots))
    while True:
        children = [joint for joint in joints if joint.parent == current]
        if not children:
            break
        if len(children) != 1:
            raise ValueError("gravity model requires one serial child at every link")
        ordered.append(children[0])
        current = children[0].child
    if len(ordered) != len(joints) or {joint.child for joint in ordered} != {joint.child for joint in joints}:
        raise ValueError("URDF contains a disconnected joint or branch")
    return ordered


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Estimate six-axis static gravity torques from URDF")
    parser.add_argument("--urdf", required=True, help="path to the serial-chain URDF")
    parser.add_argument("--q", nargs=6, type=float, required=True,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="six joint positions in radians")
    parser.add_argument("--gravity", nargs=3, type=float, default=(0.0, 0.0, -9.81),
                        metavar=("GX", "GY", "GZ"))
    args = parser.parse_args(argv)
    model = SerialUrdfGravityModel.from_urdf(args.urdf, tuple(args.gravity))
    torques = model.gravity_torques(args.q)
    print(json.dumps({"joints": [joint.name for joint in model.joints],
                      "q_rad": args.q, "gravity_torque_nm": torques},
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
