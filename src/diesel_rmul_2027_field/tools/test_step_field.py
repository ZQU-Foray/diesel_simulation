"""Check official CAD dimensions, coordinate mapping and simulation mesh identity."""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from step_field import load, scan_segments, triangles

PKG = Path(__file__).resolve().parents[1] / "src/guodong_sentry_sim"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


class StepFieldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = load(PKG)
        cls.faces = {(f["occurrence"], f["face"]): f for f in cls.data["faces"]}

    def bounds(self, index, occurrence=0):
        ps = [p for t in self.faces[occurrence, index]["triangles_m"] for p in t]
        return [min(p[i] for p in ps) for i in range(3)] + [
            max(p[i] for p in ps) for i in range(3)
        ]

    def near(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=7)

    def test_source_transform_and_extents(self):
        self.assertEqual(
            self.data["source_sha256"],
            "7d47c1b6a6a0253a732e46cb122bb84410b11d5134bcb5658104299db4a48ca4",
        )
        ps = [p for t in triangles(self.data) for p in t]
        self.near(
            [min(p[i] for p in ps) for i in range(3)]
            + [max(p[i] for p in ps) for i in range(3)],
            [-0.3, -0.3, -0.1, 12.3, 8.3, 0.42],
        )
        self.near(self.bounds(108), [0, 0, 0, 12, 8, 0])

    def test_verified_height_and_control_corrections(self):
        self.near(self.bounds(862), [0, 7.55, 0.403, 0.75, 8, 0.403])
        self.near(self.bounds(10), [0, 6, 0.003, 2.5, 8, 0.003])
        self.near(self.bounds(596), [4.5, 2.5, 0.01, 7.5, 5.5, 0.01])
        self.near(self.bounds(678), [2.9, 4.2, 0.2012, 4.1, 5.4, 0.2012])
        self.near(self.bounds(661), [2.5, 3, 0.2, 4.5, 7.06, 0.2])

    def test_tunnel_clearance_and_roof_endpoints(self):
        self.near(self.bounds(666), [3.325, 7.06, 0.25, 3.675, 7.86, 0.25])
        self.near(self.bounds(669), [3.325, 6.98, 0.27, 3.675, 7.94, 0.27])
        for f in self.data["faces"]:
            for tri in f["triangles_m"]:
                if max(p[2] for p in tri) < 0.249 and min(p[2] for p in tri) > 0.02:
                    xs = [p[0] for p in tri]
                    ys = [p[1] for p in tri]
                    self.assertFalse(
                        min(xs) < 3.5 < max(xs) and min(ys) < 7.46 < max(ys)
                    )

    def test_red_blue_orientation_and_source_colors(self):
        red = self.faces[0, 21]
        blue = self.faces[0, 570]
        self.near(red["srgb"], [1, 0, 0])
        self.assertGreater(blue["srgb"][2], 0.99)
        rb = self.bounds(21)
        bb = self.bounds(570)
        self.assertLess(rb[3], 2.5)
        self.assertGreater(rb[1], 6)
        self.assertGreater(bb[0], 9.5)
        self.assertLess(bb[4], 2)

    def test_rendering_and_collision_share_geometry(self):
        positions = []
        for name in ["step_textured.dae", "step_native.dae"]:
            root = ET.parse(PKG / "models/rmul_2027/meshes" / name)
            vals = list(
                map(
                    float,
                    root.find('.//c:float_array[@id="pos-array"]', NS).text.split(),
                )
            )
            self.assertEqual(len(vals), len(triangles(self.data)) * 9)
            positions.append(vals)
        self.assertEqual(
            sorted(zip(*(iter(positions[0]),) * 3)),
            sorted(zip(*(iter(positions[1]),) * 3)),
        )
        root = ET.parse(PKG / "models/rmul_2027/model.sdf")
        self.assertEqual(
            root.find(".//collision/geometry/mesh/uri").text,
            root.find(".//visual/geometry/mesh/uri").text,
        )
        self.assertGreater(len(scan_segments(PKG, 0.2796202475)), 100)


if __name__ == "__main__":
    unittest.main()
