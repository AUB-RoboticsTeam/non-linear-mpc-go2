#!/usr/bin/env python3
"""Convert static SDF box/cylinder models into a MuJoCo scene.

This intentionally handles the primitive subset used by elevation.sdf. The
robot remains an MJCF include; only the static environment is converted.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def values(text, count, default):
    if not text:
        return list(default)
    result = [float(value) for value in text.split()]
    if len(result) != count:
        raise ValueError(f"expected {count} values, got {result}")
    return result


def number(value):
    if abs(value) < 1e-12:
        value = 0.0
    return f"{value:.9g}"


def vector(items):
    return " ".join(number(value) for value in items)


def matching_visual(link, collision_name):
    candidates = []
    if collision_name.endswith("_c"):
        candidates.append(collision_name[:-2] + "_v")
    if collision_name.endswith("c"):
        candidates.append(collision_name[:-1] + "v")
    if collision_name == "collision":
        candidates.append("visual")
    if collision_name == "c":
        candidates.append("v")
    candidates.append(collision_name)
    visuals = {visual.get("name", ""): visual for visual in link.findall("visual")}
    for candidate in candidates:
        if candidate in visuals:
            return visuals[candidate]
    return None


def rgba_for(link, collision_name):
    visual = matching_visual(link, collision_name)
    if visual is not None:
        diffuse = visual.findtext("material/diffuse")
        if diffuse:
            return vector(values(diffuse, 4, [0.65, 0.65, 0.65, 1.0]))
    return "0.55 0.55 0.55 1"


def convert(sdf_path, output_path, robot_include):
    sdf = ET.parse(sdf_path).getroot()
    world = sdf.find("world")
    if world is None:
        raise ValueError("SDF contains no <world>")

    root = ET.Element("mujoco", {"model": world.get("name", "converted world")})
    ET.SubElement(root, "include", {"file": robot_include})
    ET.SubElement(root, "statistic", {"center": "0 0 0.8", "extent": "12"})

    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        {
            "diffuse": "0.6 0.6 0.6",
            "ambient": "0.45 0.45 0.45",
            "specular": "0.2 0.2 0.2",
        },
    )
    ET.SubElement(visual, "rgba", {"haze": "0.75 0.82 0.90 1"})
    ET.SubElement(visual, "global", {"azimuth": "-130", "elevation": "-20"})

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "sun",
            "pos": "0 0 10",
            "dir": "-0.4 0.2 -1",
            "directional": "true",
            "castshadow": "true",
            "diffuse": "0.9 0.9 0.9",
            "specular": "0.2 0.2 0.2",
        },
    )

    for model in world.findall("model"):
        if model.findtext("static", "false").strip().lower() != "true":
            continue
        model_pose = values(model.findtext("pose"), 6, [0, 0, 0, 0, 0, 0])
        body = ET.SubElement(
            worldbody,
            "body",
            {
                "name": model.get("name", "static_model"),
                "pos": vector(model_pose[:3]),
                "euler": vector(model_pose[3:]),
            },
        )
        for link in model.findall("link"):
            link_pose = values(link.findtext("pose"), 6, [0, 0, 0, 0, 0, 0])
            link_body = body
            if any(abs(value) > 1e-12 for value in link_pose):
                link_body = ET.SubElement(
                    body,
                    "body",
                    {
                        "name": f"{model.get('name')}_{link.get('name')}",
                        "pos": vector(link_pose[:3]),
                        "euler": vector(link_pose[3:]),
                    },
                )

            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                pose = values(collision.findtext("pose"), 6, [0, 0, 0, 0, 0, 0])
                attributes = {
                    "name": f"{model.get('name')}_{collision.get('name')}",
                    "pos": vector(pose[:3]),
                    "euler": vector(pose[3:]),
                    "rgba": rgba_for(link, collision.get("name", "")),
                    "friction": "0.8 0.02 0.01",
                    "condim": "3",
                }
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None:
                    full_size = values(box.findtext("size"), 3, [1, 1, 1])
                    attributes["type"] = "box"
                    attributes["size"] = vector(value / 2.0 for value in full_size)
                elif cylinder is not None:
                    radius = float(cylinder.findtext("radius"))
                    length = float(cylinder.findtext("length"))
                    attributes["type"] = "cylinder"
                    attributes["size"] = vector([radius, length / 2.0])
                else:
                    raise ValueError(
                        f"unsupported geometry in {model.get('name')}/{collision.get('name')}"
                    )
                ET.SubElement(link_body, "geom", attributes)

    ET.indent(root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="unicode", xml_declaration=False)
    with output_path.open("a", encoding="utf-8") as output:
        output.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--robot-include", default="go2.xml")
    args = parser.parse_args()
    convert(args.sdf, args.output, args.robot_include)


if __name__ == "__main__":
    main()
