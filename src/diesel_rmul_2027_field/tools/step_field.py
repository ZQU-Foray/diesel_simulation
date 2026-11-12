"""Generate simulation meshes directly from audited official STEP tessellation.

OCCT is only needed by audit_rmul_step.py during conversion, not on ROS computers.
The compressed face table preserves source colors, metre coordinates and provenance.
"""

import gzip
import json
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

NS = "http://www.collada.org/2005/11/COLLADASchema"


def load(package):
    return json.loads(
        gzip.decompress((package / "maps/rmul_2027/step_geometry.json.gz").read_bytes())
    )


def normal(triangle):
    a, b, c = triangle
    ab = [b[i] - a[i] for i in range(3)]
    ac = [c[i] - a[i] for i in range(3)]
    n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ]
    length = math.sqrt(sum(x * x for x in n))
    return [x / length for x in n] if length > 1e-15 else None


def triangles(data):
    return [t for f in data["faces"] for t in f["triangles_m"] if normal(t)]


def dae(data, textured):
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
    groups = {}
    points = []
    normals = []
    uv = []
    colors = {}
    for face in data["faces"]:
        rgb = tuple(round(x, 8) for x in face["srgb"])
        glyph = max(rgb) - min(rgb) > 0.8 and max(rgb) > 0.99
        for tri in face["triangles_m"]:
            n = normal(tri)
            if n is None:
                continue
            paper = (
                textured
                and n[2] > 0.99
                and not glyph
                and all(
                    -1e-6 <= p[0] <= 12.000001
                    and -1e-6 <= p[1] <= 8.000001
                    and -0.0001 <= p[2] <= 0.404
                    for p in tri
                )
            )
            material = (
                "paper"
                if paper
                else "rgb_" + "_".join(str(round(c * 1e8)) for c in rgb)
            )
            colors[material] = rgb
            ids = groups.setdefault(material, [])
            for p in tri:
                ids.append(len(points) // 3)
                points.extend(p)
                normals.extend(n)
                uv.extend((p[0] / 12, p[1] / 8))
    if "paper" in groups:
        images = sub(root, "library_images")
        sub(
            sub(images, "image", id="atlas"),
            "init_from",
            "../materials/textures/official_top.png",
        )
    effects = sub(root, "library_effects")
    materials = sub(root, "library_materials")
    for key, rgb in colors.items():
        profile = sub(sub(effects, "effect", id=key + "-fx"), "profile_COMMON")
        if key == "paper":
            surf = sub(sub(profile, "newparam", sid="surface"), "surface", type="2D")
            sub(surf, "init_from", "atlas")
            sub(
                sub(sub(profile, "newparam", sid="sampler"), "sampler2D"),
                "source",
                "surface",
            )
        shader = sub(sub(profile, "technique", sid="common"), "lambert")
        diffuse = sub(shader, "diffuse")
        if key == "paper":
            sub(diffuse, "texture", texture="sampler", texcoord="UVSET0")
        else:
            sub(diffuse, "color", " ".join(str(c) for c in (*rgb, 1)))
        sub(
            sub(materials, "material", id=key + "-mat"),
            "instance_effect",
            url="#" + key + "-fx",
        )
    mesh = sub(sub(sub(root, "library_geometries"), "geometry", id="geometry"), "mesh")
    for key, values, stride, axes in [
        ("pos", points, 3, "XYZ"),
        ("normal", normals, 3, "XYZ"),
        ("uv", uv, 2, "ST"),
    ]:
        source = sub(mesh, "source", id=key)
        sub(
            source,
            "float_array",
            " ".join(f"{v:.10f}" for v in values),
            id=key + "-array",
            count=str(len(values)),
        )
        access = sub(
            sub(source, "technique_common"),
            "accessor",
            source="#" + key + "-array",
            count=str(len(values) // stride),
            stride=str(stride),
        )
        for axis in axes:
            sub(access, "param", name=axis, type="float")
    sub(
        sub(mesh, "vertices", id="vertices"),
        "input",
        semantic="POSITION",
        source="#pos",
    )
    for material, ids in groups.items():
        node = sub(mesh, "triangles", material=material, count=str(len(ids) // 3))
        sub(node, "input", semantic="VERTEX", source="#vertices", offset="0")
        sub(node, "input", semantic="NORMAL", source="#normal", offset="1")
        sub(node, "input", semantic="TEXCOORD", source="#uv", offset="2", set="0")
        sub(node, "p", " ".join(str(i) for i in ids for _ in range(3)))
    scene = sub(sub(root, "library_visual_scenes"), "visual_scene", id="scene")
    inst = sub(sub(scene, "node", id="object"), "instance_geometry", url="#geometry")
    common = sub(sub(inst, "bind_material"), "technique_common")
    for material in groups:
        item = sub(
            common, "instance_material", symbol=material, target="#" + material + "-mat"
        )
        sub(
            item,
            "bind_vertex_input",
            semantic="UVSET0",
            input_semantic="TEXCOORD",
            input_set="0",
        )
    sub(sub(root, "scene"), "instance_visual_scene", url="#scene")
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def model_xml(native=False):
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name="rmul_2027")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="arena")
    for kind in ("collision", "visual"):
        node = ET.SubElement(link, kind, name="official_step_" + kind)
        mesh = ET.SubElement(ET.SubElement(node, "geometry"), "mesh")
        file = "step_native.dae" if kind == "visual" and native else "step_textured.dae"
        ET.SubElement(mesh, "uri").text = "model://rmul_2027/meshes/" + file
        if kind == "collision":
            ode = ET.SubElement(
                ET.SubElement(ET.SubElement(node, "surface"), "friction"), "ode"
            )
            for key in ("mu", "mu2"):
                ET.SubElement(ode, key).text = ".8"
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def generate(package):
    data = load(package)
    model = package / "models/rmul_2027"
    for textured in (True, False):
        (
            model / "meshes" / ("step_textured.dae" if textured else "step_native.dae")
        ).write_bytes(dae(data, textured))
    (model / "model.sdf").write_bytes(model_xml())
    ts = triangles(data)
    allpoints = [p for t in ts for p in t]
    bounds = [min(p[i] for p in allpoints) for i in range(3)] + [
        max(p[i] for p in allpoints) for i in range(3)
    ]
    mapping = package / "maps/rmul_2027"
    # Keep the existing conservative chassis projection contract, with exact CAD footprints.
    resolution = 0.025
    pad = 0.4
    image = Image.new("L", (512, 352), 0)
    draw = ImageDraw.Draw(image)
    draw.rectangle(
        (
            pad / resolution,
            pad / resolution,
            (12 + pad) / resolution,
            (8 + pad) / resolution,
        ),
        fill=254,
    )
    for tri in ts:
        if max(p[2] for p in tri) > 0.1:
            draw.polygon(
                [
                    ((p[0] + pad) / resolution, (8 + pad - p[1]) / resolution)
                    for p in tri
                ],
                fill=0,
            )
    image.save(mapping / "rmul_2027.pgm")
    # Review overlay now follows actual CAD face boundaries, not legacy boxes.
    from collections import Counter

    overlay = (
        Image.open(model / "materials/textures/official_top.png")
        .convert("RGB")
        .resize((1500, 1000))
    )
    overlay_draw = ImageDraw.Draw(overlay)
    for face in data["faces"]:
        edges = Counter()
        for tri in face["triangles_m"]:
            n = normal(tri)
            if (
                not n
                or n[2] < 0.99
                or min(p[2] for p in tri) < 0.1
                or max(p[2] for p in tri) > 0.404
            ):
                continue
            for a, b in zip(tri, tri[1:] + tri[:1]):
                edges[tuple(sorted((tuple(a), tuple(b))))] += 1
        for (a, b), count in edges.items():
            if count == 1:
                overlay_draw.line(
                    [(p[0] * 125, (8 - p[1]) * 125) for p in (a, b)],
                    fill=(245, 195, 70),
                    width=2,
                )
    overlay.save(mapping / "geometry_overlay.png")
    report = {
        "source": data["source"],
        "source_sha256": data["source_sha256"],
        "geometry": "OCCT tessellation of the full official B-rep, including frame cavities and perimeter cap",
        "faces": len(data["faces"]),
        "triangles": len(ts),
        "bounds_m": bounds,
        "linear_deflection_mm": data["linear_deflection_mm"],
        "transform": data["transform"],
        "default_appearance": "PDF top-surface atlas with STEP glyph colors; no geometry flattened",
        "native_appearance": "AP214 source face colors in rmul_2027_native world",
        "simulation_assumptions": {
            "friction_mu": 0.8,
            "map_projection_height_threshold_m": 0.1,
            "robot_dynamics": "unchanged planar VelocityControl; no terrain following",
        },
    }
    (mapping / "step_import.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    scenario = json.loads((mapping / "scenario.json").read_text(encoding="utf-8"))
    scenario.update(
        geometry_source="official STEP",
        source_sha256=data["source_sha256"],
        navigation_validated=False,
    )
    scenario["spawn"]["z"] = 0.001
    (mapping / "scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8"
    )
    return report


def scan_segments(package, height):
    """Exact segments of the mesh at the lidar plane, including native rounded edges."""
    lines = []
    for tri in triangles(load(package)):
        hits = []
        for a, b in zip(tri, tri[1:] + tri[:1]):
            if (a[2] - height) * (b[2] - height) < 0:
                t = (height - a[2]) / (b[2] - a[2])
                p = [a[k] + t * (b[k] - a[k]) for k in range(2)]
                if not any(math.dist(p, q) < 1e-9 for q in hits):
                    hits.append(p)
        if len(hits) == 2:
            lines.append(hits)
    return lines
