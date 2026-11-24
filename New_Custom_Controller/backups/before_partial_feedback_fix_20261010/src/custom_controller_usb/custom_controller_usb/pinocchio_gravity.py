"""Pinocchio gravity torque calculation for the controller USB bridge."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable


class PinocchioGravityModel:
    """Compute gravity torques for a six joint URDF.

    Pinocchio is imported lazily so state visualization and protocol tests still
    work on machines where the optional dynamics package is not installed.
    """

    def __init__(self, urdf_path: str | Path, gravity: Iterable[float] = (0.0, 0.0, -9.81)):
        try:
            import pinocchio as pin
            import numpy as np
        except ImportError as error:  # pragma: no cover - depends on host image
            raise RuntimeError(
                "Pinocchio 未安装，请在 ROS2 虚拟机安装 python3-pinocchio 或 pinocchio"
            ) from error
        gravity_values = tuple(float(value) for value in gravity)
        if len(gravity_values) != 3 or any(not math.isfinite(value) for value in gravity_values):
            raise ValueError("gravity must contain three finite values")
        self._pin = pin
        self._np = np
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError(f"expected six one-DOF joints, got nq={self.model.nq}, nv={self.model.nv}")
        self.model.gravity.linear = np.array(gravity_values, dtype=float)
        self.data = self.model.createData()

    def gravity_torques(self, joint_positions: Iterable[float]) -> tuple[float, ...]:
        values = tuple(float(value) for value in joint_positions)
        if len(values) != 6 or any(not math.isfinite(value) for value in values):
            raise ValueError("joint_positions must contain six finite values")
        result = self._pin.computeGeneralizedGravity(
            self.model, self.data, self._np.asarray(values, dtype=float))
        torques = tuple(float(value) for value in result)
        if any(not math.isfinite(value) for value in torques):
            raise ValueError("Pinocchio returned a non-finite gravity torque")
        return torques

    def torque_from_motor_positions(
        self,
        motor_positions: Iterable[float],
        axis_order: Iterable[int],
        scales: Iterable[float],
        offsets: Iterable[float],
        torque_signs: Iterable[float] = (1.0,) * 6,
        torque_scale: float = 1.0,
        valid_motor_mask: int = 0x3F,
    ) -> tuple[tuple[float, ...], int]:
        motors = tuple(float(value) for value in motor_positions)
        order = tuple(int(value) for value in axis_order)
        scale_values = tuple(float(value) for value in scales)
        offset_values = tuple(float(value) for value in offsets)
        signs = tuple(float(value) for value in torque_signs)
        if (len(motors), len(order), len(scale_values), len(offset_values), len(signs)) != (6, 6, 6, 6, 6):
            raise ValueError("all mappings must contain six values")
        if sorted(order) != list(range(6)) or any(value == 0 for value in scale_values):
            raise ValueError("axis_order must be a permutation and scales must be nonzero")
        if not math.isfinite(torque_scale):
            raise ValueError("torque_scale must be finite")
        joints = [0.0] * 6
        joint_mask = 0
        for joint, motor in enumerate(order):
            if valid_motor_mask & (1 << motor):
                joints[joint] = scale_values[joint] * motors[motor] + offset_values[joint]
                if math.isfinite(joints[joint]):
                    joint_mask |= 1 << joint
        gravity = self.gravity_torques(joints)
        output = [0.0] * 6
        output_mask = 0
        for joint, motor in enumerate(order):
            if joint_mask & (1 << joint):
                output[motor] = gravity[joint] * signs[joint] * torque_scale
                if math.isfinite(output[motor]):
                    output_mask |= 1 << motor
        return tuple(output), output_mask
