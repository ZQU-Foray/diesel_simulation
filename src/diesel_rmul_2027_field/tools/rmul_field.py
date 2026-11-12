"""Build the RMUL 2027 3v3 scenery from the supplied manual's nominal dimensions.

All dimensions are metres. Appearance uses actual pixels from Fig 3-1;
unpublished measurements are explicitly listed in parameters.json.
No SLAM, navigation stack, robot dynamics or referee logic is implemented here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src/guodong_sentry_sim"
MODEL = PKG / "models/rmul_2027"
MAP = PKG / "maps/rmul_2027"
PARAMS = json.loads((MAP / "parameters.json").read_text(encoding="utf-8"))
MANUAL = PARAMS.get("manual_reference", PARAMS)
S = MANUAL["specified"]
D = MANUAL["derived"]
A = MANUAL["inferred"]
NS = "http://www.collada.org/2005/11/COLLADASchema"


def rect(x, y, w, h):
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]


def parts():
    """Legacy manual-only primitives, kept for provenance and regression comparison.

    If step_geometry.json.gz exists, generate() uses full STEP geometry instead.
    These primitives must not be used as current scan or collision ground truth.
    """
    output = []

    def add(name, poly, z0, z1, *, texture=True, paired=False, top_z=None):
        item = {"name": name, "polygon": poly, "z0": z0, "z1": z1, "textured": texture}
        if top_z is not None:
            item["top_z"] = top_z
        if paired:
            output.append(dict(item, name="red_" + name))
            blue = dict(
                item, name="blue_" + name, polygon=[(12 - x, 8 - y) for x, y in poly]
            )
            output.append(blue)
        else:
            output.append(item)

    fw, fh = S["field_size"]
    pvc = S["pvc_thickness"]
    add(
        "floor_wood",
        rect(0, 0, fw, fh),
        -A["floor_substructure_thickness"],
        -pvc,
        texture=False,
    )
    add("floor_pvc_2mm", rect(0, 0, fw, fh), -pvc, 0)
    wt, wh = A["perimeter_thickness"], A["perimeter_height"]
    add("perimeter_south", rect(-wt, -wt, fw + 2 * wt, wt), -0.1, wh, texture=False)
    add("perimeter_north", rect(-wt, fh, fw + 2 * wt, wt), -0.1, wh, texture=False)
    add("perimeter_west", rect(-wt, 0, wt, fh), -0.1, wh, texture=False)
    add("perimeter_east", rect(fw, 0, wt, fh), -0.1, wh, texture=False)
    add("supply_extra_pvc", rect(0, 6, *S["supply_size"]), 0, pvc, paired=True)
    # Workbench 750x450mm, straight edges 650x350mm -> 50mm chamfers.
    c = D["workbench_corner_chamfer"]
    bw, bh, bz = S["workbench_size"]
    poly = [
        (0, 8 - bh + c),
        (c, 8 - bh),
        (bw - c, 8 - bh),
        (bw, 8 - bh + c),
        (bw, 8 - c),
        (bw - c, 8),
        (c, 8),
        (0, 8 - c),
    ]
    add("workbench", poly, pvc, bz, paired=True)
    x, y = S["highland_southwest"]
    w, h, z = S["highland_size"]
    c = A["highland_north_chamfer"]
    poly = [
        (x, y),
        (x + w, y),
        (x + w, y + h - c),
        (x + w - c, y + h),
        (x + c, y + h),
        (x, y + h - c),
    ]
    add("highland_wood", poly, 0, z - pvc, paired=True)
    add("highland_pvc_2mm", poly, z - pvc, z, paired=True)
    add(
        "parapet",
        rect(
            x + w - S["highland_parapet_width"],
            y + S["highland_parapet_south_offset"],
            S["highland_parapet_width"],
            S["highland_parapet_length"],
        ),
        z,
        z + S["highland_parapet_height_above_deck"],
        paired=True,
    )
    dx, dy = S["highland_control_southwest_offset"]
    add(
        "highland_control_extra_pvc",
        rect(x + dx, y + dy, *S["highland_control_size"]),
        z,
        z + pvc,
        paired=True,
    )
    # Far support occupies only the last 140mm. The 800mm tunnel stays open.
    north = D["highland_north_y"]
    far = north + S["tunnel_clear_span"]
    add(
        "tunnel_far_support",
        rect(x, far, w, 8 - far),
        0,
        S["tunnel_clear_height"],
        paired=True,
    )
    sx = x + S["metal_strip_west_offset"]
    sw = S["metal_strip_width"]
    add(
        "tunnel_metal_roof",
        rect(sx, north, sw, 8 - north),
        S["tunnel_clear_height"],
        S["tunnel_roof_top"],
        paired=True,
    )
    # The drawing shows a metal transition onto the deck, but no length callout.
    length = A["metal_transition_length"]
    add(
        "metal_transition",
        rect(sx, north - length, sw, length),
        z - 0.02,
        S["tunnel_roof_top"],
        paired=True,
        top_z=[z, z, S["tunnel_roof_top"], S["tunnel_roof_top"]],
    )
    add(
        "center_control_extra_pvc",
        rect(*S["center_control_southwest"], *S["center_control_size"]),
        0,
        pvc,
    )
    # Two tunnel gain patches per team (the supply area is the other patch).
    add(
        "tunnel_gain_extra_pvc",
        rect(
            x + S["metal_strip_west_offset"] + sw,
            north,
            w - S["metal_strip_west_offset"] - sw,
            far - north,
        ),
        0,
        pvc,
        paired=True,
    )
    return output


def collada(part):
    """Closed triangle mesh; top uses world-aligned UVs, sides use dark material."""
    ET.register_namespace("", NS)

    def sub(parent, tag, text=None, **attrs):
        node = ET.SubElement(parent, "{" + NS + "}" + tag, attrs)
        if text is not None:
            node.text = str(text)
        return node

    root = ET.Element("{" + NS + "}COLLADA", version="1.4.1")
    asset = sub(root, "asset")
    sub(asset, "created", "2026-10-16T00:00:00Z")
    sub(asset, "modified", "2026-10-16T00:00:00Z")
    sub(asset, "unit", name="meter", meter="1")
    sub(asset, "up_axis", "Z_UP")
    imgs = sub(root, "library_images")
    sub(
        sub(imgs, "image", id="atlas"),
        "init_from",
        "../materials/textures/official_top.png",
    )
    effects = sub(root, "library_effects")
    for name in ("top", "side"):
        eff = sub(effects, "effect", id=name + "-fx")
        profile = sub(eff, "profile_COMMON")
        if name == "top":
            surface = sub(sub(profile, "newparam", sid="surface"), "surface", type="2D")
            sub(surface, "init_from", "atlas")
            sampler = sub(sub(profile, "newparam", sid="sampler"), "sampler2D")
            sub(sampler, "source", "surface")
        tech = sub(profile, "technique", sid="common")
        shader = sub(tech, "lambert")
        diffuse = sub(shader, "diffuse")
        if name == "top" and part["textured"]:
            sub(diffuse, "texture", texture="sampler", texcoord="UVSET0")
        else:
            sub(diffuse, "color", "0.11 0.11 0.11 1")
    mats = sub(root, "library_materials")
    for name in ("top", "side"):
        sub(
            sub(mats, "material", id=name + "-mat", name=name),
            "instance_effect",
            url="#" + name + "-fx",
        )

    poly = part["polygon"]
    topz = part.get("top_z", [part["z1"]] * len(poly))
    top = [(x, y, z) for (x, y), z in zip(poly, topz)]
    bottom = [(x, y, part["z0"]) for x, y in poly]
    faces = {"top": [], "side": []}
    for i in range(1, len(poly) - 1):
        faces["top"].append((top[0], top[i], top[i + 1]))
        faces["side"].append((bottom[0], bottom[i + 1], bottom[i]))
    for i in range(len(poly)):
        j = (i + 1) % len(poly)
        faces["side"].extend(
            [(bottom[i], bottom[j], top[j]), (bottom[i], top[j], top[i])]
        )
    points, normals, uvs, indexes = [], [], [], {"top": [], "side": []}
    import math

    for material, triangles in faces.items():
        for a, b, c in triangles:
            ab = [b[k] - a[k] for k in range(3)]
            ac = [c[k] - a[k] for k in range(3)]
            normal = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ]
            norm = math.sqrt(sum(n * n for n in normal))
            normal = [n / norm for n in normal]
            for vertex in (a, b, c):
                indexes[material].append(len(points) // 3)
                points.extend(vertex)
                normals.extend(normal)
                uvs.extend((vertex[0] / 12, vertex[1] / 8))
    mesh = sub(sub(sub(root, "library_geometries"), "geometry", id="geometry"), "mesh")
    for name, values, stride, axes in [
        ("pos", points, 3, "XYZ"),
        ("normal", normals, 3, "XYZ"),
        ("uv", uvs, 2, "ST"),
    ]:
        source = sub(mesh, "source", id=name)
        sub(
            source,
            "float_array",
            " ".join(f"{n:.8f}" for n in values),
            id=name + "-array",
            count=str(len(values)),
        )
        accessor = sub(
            sub(source, "technique_common"),
            "accessor",
            source="#" + name + "-array",
            count=str(len(values) // stride),
            stride=str(stride),
        )
        for axis in axes:
            sub(accessor, "param", name=axis, type="float")
    sub(
        sub(mesh, "vertices", id="vertices"),
        "input",
        semantic="POSITION",
        source="#pos",
    )
    for material, indices in indexes.items():
        tri = sub(mesh, "triangles", count=str(len(indices) // 3), material=material)
        sub(tri, "input", semantic="VERTEX", source="#vertices", offset="0")
        sub(tri, "input", semantic="NORMAL", source="#normal", offset="1")
        sub(tri, "input", semantic="TEXCOORD", source="#uv", offset="2", set="0")
        sub(tri, "p", " ".join(str(i) for i in indices for _ in range(3)))
    scene = sub(sub(root, "library_visual_scenes"), "visual_scene", id="scene")
    instance = sub(
        sub(scene, "node", id="object"), "instance_geometry", url="#geometry"
    )
    common = sub(sub(instance, "bind_material"), "technique_common")
    for name in ("top", "side"):
        mat = sub(common, "instance_material", symbol=name, target="#" + name + "-mat")
        sub(
            mat,
            "bind_vertex_input",
            semantic="UVSET0",
            input_semantic="TEXCOORD",
            input_set="0",
        )
    sub(sub(root, "scene"), "instance_visual_scene", url="#scene")
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def model_xml(items):
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name="rmul_2027")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="arena")
    for part in items:
        for kind in ("collision", "visual"):
            node = ET.SubElement(link, kind, name=part["name"] + "_" + kind)
            mesh = ET.SubElement(ET.SubElement(node, "geometry"), "mesh")
            ET.SubElement(mesh, "uri").text = (
                f"model://rmul_2027/meshes/{part['name']}.dae"
            )
            if kind == "collision":
                friction = ET.SubElement(ET.SubElement(node, "surface"), "friction")
                ode = ET.SubElement(friction, "ode")
                for name in ("mu", "mu2"):
                    ET.SubElement(ode, name).text = str(A["floor_friction_mu"])
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def world_xml(preview=False):
    root = ET.Element("sdf", version="1.9")
    # Retain the original world name for bridge.yaml and Gazebo service compatibility.
    world = ET.SubElement(root, "world", name="sentry_lab")
    physics = ET.SubElement(world, "physics", name="physics", type="ignored")
    ET.SubElement(physics, "max_step_size").text = ".002"
    ET.SubElement(physics, "real_time_factor").text = "1"
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    for system, filename in [
        ("Physics", "physics"),
        ("UserCommands", "user-commands"),
        ("SceneBroadcaster", "scene-broadcaster"),
        ("Sensors", "sensors"),
    ]:
        p = ET.SubElement(
            world,
            "plugin",
            filename="gz-sim-" + filename + "-system",
            name="gz::sim::systems::" + system,
        )
        if system == "Sensors":
            ET.SubElement(p, "render_engine").text = "ogre2"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = ".75 .75 .75 1"
    ET.SubElement(scene, "background").text = ".16 .17 .19 1"
    ET.SubElement(scene, "shadows").text = "false"
    light = ET.SubElement(world, "light", name="arena_light", type="directional")
    ET.SubElement(light, "pose").text = "6 4 10 0 0 0"
    ET.SubElement(light, "direction").text = "-.3 .2 -1"
    ET.SubElement(light, "diffuse").text = ".45 .45 .45 1"
    ET.SubElement(light, "specular").text = ".1 .1 .1 1"
    gui = ET.SubElement(world, "gui", fullscreen="false")
    p = ET.SubElement(gui, "plugin", filename="MinimalScene", name="3D View")
    for tag, value in [
        ("engine", "ogre2"),
        ("scene", "scene"),
        ("ambient_light", ".75 .75 .75"),
        ("background_color", ".16 .17 .19"),
        ("camera_pose", "15 -9 12 0 .62 2.18"),
    ]:
        ET.SubElement(p, tag).text = value
    ET.SubElement(gui, "plugin", filename="GzSceneManager", name="Scene Manager")
    ET.SubElement(
        gui,
        "plugin",
        filename="InteractiveViewControl",
        name="Interactive view control",
    )
    ET.SubElement(gui, "plugin", filename="SelectEntities", name="Select Entities")
    for uri, name, pose in [
        ("model://rmul_2027", "rmul_2027", "0 0 0 0 0 0"),
        (
            "model://guodong_sentry" + ("_preview" if preview else ""),
            "guodong_sentry",
            f"{A['spawn_xy'][0]} {A['spawn_xy'][1]} {PARAMS.get('effective_simulation',{}).get('spawn_z',0)} 0 0 0",
        ),
    ]:
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = uri
        ET.SubElement(include, "name").text = name
        ET.SubElement(include, "pose").text = pose
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def traversability(items):
    """Conservative chassis projection, deliberately not a 279.62mm lidar slice."""
    res, pad = A["map_resolution"], 0.4
    width, height = round((12 + 2 * pad) / res), round((8 + 2 * pad) / res)
    im = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(im)

    def pixels(poly):
        return [((x + pad) / res, (8 + pad - y) / res) for x, y in poly]

    draw.polygon(pixels(rect(0, 0, 12, 8)), fill=254)
    for item in items:
        if item["z1"] > 0.1:
            draw.polygon(pixels(item["polygon"]), fill=0)
    im.save(MAP / "rmul_2027.pgm")
    (MAP / "rmul_2027.yaml").write_text(
        "image: rmul_2027.pgm\nmode: trinary\nresolution: 0.025\n"
        "origin: [-0.4, -0.4, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
        newline="\n",
    )
    # Preview follows the source figure, with actual geometry edges for verification.
    atlas = Image.open(MODEL / "materials/textures/official_top.png").convert("RGB")
    atlas = atlas.resize((1500, 1000))
    d = ImageDraw.Draw(atlas)
    for item in items:
        if item["z1"] > 0.1 and not item["name"].startswith("perimeter"):
            poly = [(x * 125, (8 - y) * 125) for x, y in item["polygon"]]
            d.line(poly + [poly[0]], fill=(245, 195, 70), width=2)
    atlas.save(MAP / "geometry_overlay.png")
    scenario = {
        "world": "rmul_2027",
        "spawn": {"xy": A["spawn_xy"], "yaw": 0},
        "map_type": "conservative chassis traversability projection, NOT SLAM",
        "resolution_m": res,
        "origin": [-pad, -pad, 0],
        "size_cells": [width, height],
        "robot_terrain_following": False,
        "navigation_validated": False,
        "warning": "Do not reuse Depot patrol coordinates or assume this map matches the horizontal lidar slice.",
    }
    (MAP / "scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8"
    )


def generate():
    items = parts()
    (MODEL / "meshes").mkdir(parents=True, exist_ok=True)
    (PKG / "worlds").mkdir(parents=True, exist_ok=True)
    for item in items:
        (MODEL / "meshes" / f"{item['name']}.dae").write_bytes(collada(item))
    (MODEL / "model.sdf").write_bytes(model_xml(items))
    (MODEL / "model.config").write_text(
        '<?xml version="1.0"?>\n<model><name>RMUL 2027 3v3</name><version>1.0</version>'
        '<sdf version="1.9">model.sdf</sdf><author><name>Local manual reconstruction</name>'
        "</author><description>Official STEP geometry when present; manual reconstruction retained for comparison. See parameters.json."
        "</description></model>\n",
        encoding="utf-8",
    )
    for preview in (False, True):
        path = PKG / "worlds" / ("rmul_2027" + ("_preview" if preview else "") + ".sdf")
        path.write_bytes(world_xml(preview))
    traversability(items)
    if (MAP / "step_geometry.json.gz").is_file():
        from step_field import generate as generate_step
        from step_field import model_xml as step_model_xml

        print(json.dumps(generate_step(PKG), indent=2))
        for preview in (False, True):
            root = ET.fromstring(world_xml(preview))
            world = root.find("world")
            for inc in list(world.findall("include")):
                if inc.findtext("uri") == "model://rmul_2027":
                    world.remove(inc)
            world.append(ET.fromstring(step_model_xml(native=True)).find("model"))
            ET.indent(root)
            (
                PKG
                / "worlds"
                / ("rmul_2027_native" + ("_preview" if preview else "") + ".sdf")
            ).write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    print(
        json.dumps(
            {
                "legacy_manual_primitives": len(items),
                "world": str(PKG / "worlds/rmul_2027.sdf"),
                "atlas_sha256": hashlib.sha256(
                    (MODEL / "materials/textures/official_top.png").read_bytes()
                ).hexdigest(),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    argparse.ArgumentParser(description=__doc__).parse_args()
    generate()
