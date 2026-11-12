"""Dimension, symmetry, tunnel topology and installed asset regression checks."""

import unittest
import xml.etree.ElementTree as ET

from rmul_field import MODEL, PKG, D, S, collada, parts


class FieldTests(unittest.TestCase):
    def setUp(self):
        self.items = {p["name"]: p for p in parts()}

    def test_manual_dimensions(self):
        p = self.items["red_highland_pvc_2mm"]
        self.assertEqual(S["field_size"], [12.0, 8.0])
        self.assertAlmostEqual(
            max(x for x, _ in p["polygon"]) - min(x for x, _ in p["polygon"]), 2.0
        )
        self.assertAlmostEqual(
            max(y for _, y in p["polygon"]) - min(y for _, y in p["polygon"]), 4.06
        )
        self.assertAlmostEqual(p["z1"], 0.2)
        self.assertAlmostEqual(self.items["red_parapet"]["z1"], 0.4)
        self.assertAlmostEqual(self.items["red_workbench"]["z1"], 0.4)
        self.assertAlmostEqual(S["pvc_thickness"], 0.002)

    def test_central_symmetry_every_paired_solid(self):
        for name, red in self.items.items():
            if not name.startswith("red_"):
                continue
            blue = self.items[name.replace("red_", "blue_", 1)]
            for (rx, ry), (bx, by) in zip(red["polygon"], blue["polygon"]):
                self.assertAlmostEqual(rx + bx, 12.0)
                self.assertAlmostEqual(ry + by, 8.0)
            for key in ("z0", "z1", "textured"):
                self.assertEqual(red[key], blue[key])

    def test_tunnel_is_not_a_filled_box(self):
        roof = self.items["red_tunnel_metal_roof"]
        self.assertAlmostEqual(roof["z0"], 0.25)
        self.assertAlmostEqual(roof["z1"], 0.27)
        north = D["highland_north_y"]
        support = self.items["red_tunnel_far_support"]
        self.assertAlmostEqual(min(y for _, y in support["polygon"]) - north, 0.8)
        # A ray at chassis height under the metal strip must not intersect a support.
        x, y = 3.5, north + 0.4
        for p in self.items.values():
            if not (p["z0"] < 0.125 < p["z1"]):
                continue
            xs, ys = zip(*p["polygon"])
            self.assertFalse(min(xs) < x < max(xs) and min(ys) < y < max(ys), p["name"])

    def test_meshes_are_closed_and_nondegenerate(self):
        # Every geometric edge of the closed triangle mesh occurs twice.
        from collections import Counter

        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        for p in self.items.values():
            root = ET.fromstring(collada(p))
            values = list(
                map(
                    float,
                    root.find('.//c:float_array[@id="pos-array"]', ns).text.split(),
                )
            )
            vertices = [tuple(values[i : i + 3]) for i in range(0, len(values), 3)]
            edges = Counter()
            for triangle in root.findall(".//c:triangles/c:p", ns):
                indices = list(map(int, triangle.text.split()))[::3]
                for i in range(0, len(indices), 3):
                    pts = [vertices[k] for k in indices[i : i + 3]]
                    for a, b in zip(pts, pts[1:] + pts[:1]):
                        self.assertNotEqual(a, b)
                        edges[tuple(sorted((a, b)))] += 1
            self.assertTrue(all(n == 2 for n in edges.values()), p["name"])

    def test_worlds_and_resources(self):
        model = ET.parse(MODEL / "model.sdf")
        for link in model.findall(".//link"):
            self.assertEqual(
                len(link.findall("visual")), len(link.findall("collision"))
            )
        for uri in model.findall(".//mesh/uri"):
            self.assertTrue(
                (PKG / "models" / uri.text.removeprefix("model://")).is_file(), uri.text
            )
        for suffix in ("", "_preview"):
            world = ET.parse(PKG / "worlds" / f"rmul_2027{suffix}.sdf")
            self.assertEqual(world.getroot().find("world").get("name"), "sentry_lab")
            uris = [n.text for n in world.findall(".//include/uri")]
            self.assertIn("model://rmul_2027", uris)
            self.assertIn("model://guodong_sentry" + suffix, uris)


if __name__ == "__main__":
    unittest.main()
