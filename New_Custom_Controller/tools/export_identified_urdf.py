"""把静态等效质量一阶矩修正导出为独立 URDF，不改变原模型。"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def export(original, fit_path, output):
    fit = json.loads(Path(fit_path).read_text(encoding="utf-8-sig"))
    tree = ET.parse(original)
    links = {link.get("name"): link for link in tree.getroot().findall("link")}
    delta = fit["delta_first_moments_kg_m"]
    if len(delta) != 4 or not all(math.isfinite(v) for v in delta):
        raise ValueError("需要四个有限质量一阶矩修正值")
    for name, shift in (("link2", delta[:2]), ("link3", delta[2:])):
        inertial = links[name].find("inertial")
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        com = [float(v) for v in origin.get("xyz").split()]
        if mass <= 0:
            raise ValueError(f"{name} 质量必须大于零")
        if name == "link2":
            prior = fit["upper_arm"]
            if abs(mass - prior["mass_kg"]) > 1e-10 or math.dist(com, prior["prior_com_m"]) > 1e-10:
                raise ValueError("URDF 与辨识使用的上臂质量/质心不一致")
        for i in range(2):
            com[i] += shift[i] / mass
        origin.set("xyz", " ".join(f"{v:.17g}" for v in com))
    tree.getroot().insert(0, ET.Comment(
        " Static gravity candidate: fixed masses, link2 xy COM correction, "
        "rear-assembly equivalent first-moment correction assigned to link3. "
        "z and inertia tensors retain URDF priors. Torque biases are NOT applied. "))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--fit", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(export(args.urdf, args.fit, args.output))
