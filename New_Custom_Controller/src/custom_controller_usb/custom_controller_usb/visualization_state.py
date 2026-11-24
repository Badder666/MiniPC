"""Read-only target/feedback tracking for RViz; independent of ROS2."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from .protocol import AXIS_COUNT


@dataclass(frozen=True)
class ArmDescription:
    root_link: str
    joint_names: tuple[str, ...]
    limits: tuple[tuple[float, float], ...]


def read_arm_description(xml_text: str) -> ArmDescription:
    robot = ET.fromstring(xml_text)
    if robot.tag != "robot":
        raise ValueError("description must be a URDF robot")
    links = {link.attrib["name"] for link in robot.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in robot.findall("joint")}
    roots = links - children
    if len(roots) != 1:
        raise ValueError("URDF must have exactly one root link")
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")
              if joint.attrib["type"] in ("revolute", "continuous")}
    if len(joints) != AXIS_COUNT:
        raise ValueError("arm URDF must contain exactly six revolute joints")
    # Prefer the historical J1..J6 names, but also accept the controller URDF
    # names (jonit1, joint2 ... joint6) and preserve their chain order.
    preferred = tuple(f"J{i + 1}" for i in range(AXIS_COUNT))
    names = preferred if set(preferred).issubset(joints) else tuple(
        joint.attrib["name"] for joint in robot.findall("joint")
        if joint.attrib["type"] in ("revolute", "continuous")
    )
    limits = []
    for name in names:
        joint = joints[name]
        limit = joint.find("limit")
        if joint.attrib["type"] == "continuous":
            limits.append((-math.inf, math.inf))
        else:
            if limit is None:
                raise ValueError(f"missing joint limits: {name}")
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"invalid joint limits: {name}")
            limits.append((lower, upper))
    return ArmDescription(next(iter(roots)), names, tuple(limits))


def target_robot_description(xml_text: str) -> str:
    robot = ET.fromstring(xml_text)
    for visual in robot.findall("./link/visual"):
        material = visual.find("material")
        if material is not None:
            visual.remove(material)
        material = ET.SubElement(visual, "material", {"name": "target_green"})
        ET.SubElement(material, "color", {"rgba": "0.2 0.85 0.45 1"})
    return ET.tostring(robot, encoding="unicode")


class VisualizationState:
    def __init__(self, mapper, feedback_names, reference_positions, stale_after_s=0.25):
        if (len(feedback_names) != AXIS_COUNT or len(set(feedback_names)) != AXIS_COUNT or
                not all(feedback_names)):
            raise ValueError("feedback names must contain six unique names")
        if len(reference_positions) != AXIS_COUNT or not all(math.isfinite(v) for v in reference_positions):
            raise ValueError("reference pose must contain six finite radians")
        if not math.isfinite(stale_after_s) or stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive and finite")
        self.mapper = mapper
        self.feedback_names = tuple(feedback_names)
        self.stale_after_s = stale_after_s
        self.raw_positions = [math.nan] * AXIS_COUNT
        self.targets = list(reference_positions)
        self.actual = list(reference_positions)
        self.target_seen_mask = 0
        self.actual_seen_mask = 0
        self.valid_joint_mask = 0
        self.buttons = 0
        self.sequence = 0
        self.device_time_ms = 0
        self.controller_arrival = None
        self.feedback_arrival = None
        self._controller_times = []

    def update_controller(self, positions, valid_motor_mask, buttons, sequence, device_time_ms, now):
        targets, mask = self.mapper.map(positions, valid_motor_mask)
        if not 0 <= buttons <= 0xFFFF:
            raise ValueError("invalid controller button bitmap")
        self.raw_positions = list(positions)
        for i in range(AXIS_COUNT):
            if mask & (1 << i):
                self.targets[i] = targets[i]
        self.valid_joint_mask = mask
        self.target_seen_mask |= mask
        self.buttons = buttons
        self.sequence = sequence
        self.device_time_ms = device_time_ms
        self.controller_arrival = now
        self._controller_times.append(now)
        # Bounded two-second rolling rate estimate, including startup.
        self._controller_times = [t for t in self._controller_times[-512:] if now - t <= 2.0]

    def update_feedback(self, names, positions, now):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("feedback joint names and positions must match")
        incoming = dict(zip(names, positions))
        if set(incoming) != set(self.feedback_names):
            raise ValueError("feedback must contain exactly the configured six joint names")
        ordered = [incoming[name] for name in self.feedback_names]
        if not all(math.isfinite(v) for v in ordered):
            raise ValueError("non-finite actual joint position")
        self.actual = ordered
        self.actual_seen_mask = 0x3F
        self.feedback_arrival = now

    @staticmethod
    def age(arrival, now):
        return None if arrival is None else max(0.0, now - arrival)

    def controller_online(self, now):
        age = self.age(self.controller_arrival, now)
        return age is not None and age <= self.stale_after_s

    def feedback_online(self, now):
        age = self.age(self.feedback_arrival, now)
        return age is not None and age <= self.stale_after_s

    def effective_valid_mask(self, now):
        return self.valid_joint_mask if self.controller_online(now) else 0

    def receive_rate(self, now):
        times = [t for t in self._controller_times if now - t <= 2.0]
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) >= 2 and times[-1] > times[0] else 0.0

    def joint_error(self, joint, now):
        bit = 1 << joint
        if (self.effective_valid_mask(now) & bit and self.actual_seen_mask & bit and
                self.feedback_online(now)):
            return self.targets[joint] - self.actual[joint]
        return None
