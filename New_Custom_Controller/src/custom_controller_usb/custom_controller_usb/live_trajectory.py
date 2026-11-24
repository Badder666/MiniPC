"""Accumulate observed poses only; gaps never become interpolated paths."""

import math

from .trajectory import UrdfKinematics


class LiveTrajectory:
    def __init__(self, xml_text, max_points=3000, gap_s=0.25):
        if max_points < 4:
            raise ValueError('trajectory_max_points must be at least 4')
        self.model = UrdfKinematics(xml_text)
        self.max_points = max_points
        self.gap_s = gap_s
        self.points = []
        self.joint_positions = []
        self.device_times = []
        self.segments = []
        self.minimum = self.maximum = None
        self._segment = 0
        self._last_time = self._last_device_time = self._last_q = None
        self._broken = True

    @property
    def count(self):
        return len(self.points)

    def update(self, q, mask, now, device_time_ms):
        if mask != 63 or not all(math.isfinite(v) for v in q):
            self._broken = True
            return False
        gap = (self._broken or self._last_time is None or
               now - self._last_time > self.gap_s or
               device_time_ms < self._last_device_time)
        duplicate = not gap and device_time_ms == self._last_device_time
        self._last_time, self._last_device_time = now, device_time_ms
        if duplicate:
            return False
        point = self.model.end_effector(q)
        if self.minimum is None:
            self.minimum = self.maximum = point
        self.minimum = tuple(min(a, b) for a, b in zip(self.minimum, point))
        self.maximum = tuple(max(a, b) for a, b in zip(self.maximum, point))
        if (not gap and max(abs(a-b) for a, b in zip(q, self._last_q)) < 0.01
                and math.dist(point, self.points[-1]) < 0.002):
            return False
        if gap:
            self._segment += 1
        self._broken = False
        self._last_q = tuple(q)
        self.points.append(point)
        self.joint_positions.append(tuple(q))
        self.device_times.append(int(device_time_ms))
        self.segments.append(self._segment)
        if len(self.points) > self.max_points:
            # Retain session-wide coverage instead of a rolling tail only.
            indices = list(range(0, len(self.points)-1, 2)) + [len(self.points)-1]
            self.points = [self.points[i] for i in indices]
            self.joint_positions = [self.joint_positions[i] for i in indices]
            self.device_times = [self.device_times[i] for i in indices]
            self.segments = [self.segments[i] for i in indices]
        return True

    def line_points(self):
        return [p for i in range(1, self.count)
                if self.segments[i-1] == self.segments[i]
                for p in (self.points[i-1], self.points[i])]
