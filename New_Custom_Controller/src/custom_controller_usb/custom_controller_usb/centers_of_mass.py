"""Read link COMs and combine them at the current pose, without Pinocchio."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from .trajectory import UrdfKinematics


@dataclass(frozen=True)
class LinkMass:
    name: str
    mass: float
    xyz: tuple[float, float, float]


def read_link_masses(xml_text):
    result = {}
    for link in ET.fromstring(xml_text).findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        xyz = tuple(float(v) for v in (
            origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
        if len(xyz) != 3 or not all(math.isfinite(v) for v in (mass, *xyz)) or mass < 0:
            raise ValueError(f"invalid mass/COM: {link.get('name')}")
        if mass > 0:
            name = link.get("name")
            result[name] = LinkMass(name, mass, xyz)
    return result


class CenterOfMassModel:
    def __init__(self, xml_text):
        self.links = read_link_masses(xml_text)
        self.kinematics = UrdfKinematics(xml_text)

    def aggregate(self, positions, names, frame):
        """Mass-weighted COM expressed in frame, recomputed as joints move."""
        transforms = self.kinematics.link_transforms(positions)
        mass = sum(self.links[name].mass for name in names)
        if mass <= 0:
            raise ValueError("aggregate mass must be positive")
        weighted = [0.0] * 3
        for name in names:
            link = self.links[name]
            transform = transforms[name]
            for i in range(3):
                weighted[i] += link.mass * (transform[i][3] + sum(
                    transform[i][j] * link.xyz[j] for j in range(3)))
        reference = transforms[frame]
        difference = [weighted[i] / mass - reference[i][3] for i in range(3)]
        # R transpose maps from root coordinates back into the chosen link.
        xyz = tuple(sum(reference[j][i] * difference[j] for j in range(3))
                    for i in range(3))
        return LinkMass("rear_assembly", mass, xyz)
