"""Map controller motor radians into commanded arm joint radians."""

import math

from .protocol import AXIS_COUNT


class JointMapper:
    def __init__(self, axis_order, scales, offsets):
        if len(axis_order) != AXIS_COUNT or sorted(axis_order) != list(range(AXIS_COUNT)):
            raise ValueError("axis_order must be a permutation of 0..5")
        if len(scales) != AXIS_COUNT or len(offsets) != AXIS_COUNT:
            raise ValueError("scales and offsets must each contain six values")
        if any(not math.isfinite(v) or v == 0 for v in scales):
            raise ValueError("scales must be finite and nonzero")
        if any(not math.isfinite(v) for v in offsets):
            raise ValueError("offsets must be finite radians")
        self.axis_order = tuple(axis_order)
        self.scales = tuple(scales)
        self.offsets = tuple(offsets)

    def map(self, positions, valid_motor_mask):
        if len(positions) != AXIS_COUNT or valid_motor_mask & ~0x3F:
            raise ValueError("invalid controller snapshot")
        targets = [0.0] * AXIS_COUNT
        valid_mask = 0
        for joint, motor in enumerate(self.axis_order):
            if valid_motor_mask & (1 << motor) and math.isfinite(positions[motor]):
                target = self.scales[joint] * positions[motor] + self.offsets[joint]
                if math.isfinite(target) and abs(target) <= 3.4028234663852886e38:
                    targets[joint] = target
                    valid_mask |= 1 << joint
        return tuple(targets), valid_mask
