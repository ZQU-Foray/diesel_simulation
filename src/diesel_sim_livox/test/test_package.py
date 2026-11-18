"""diesel_sim_livox 包结构自检（I1 占位，I2 起替换为采样数学单测）。

只断言包骨架自洽：包名=目录名、声明了 ament_cmake 构建类型。
真正的采样数学单测在 I2 引入 livox_pattern 后补齐。
"""

import pathlib
import unittest
import xml.etree.ElementTree as ET

PKG_DIR = pathlib.Path(__file__).resolve().parent.parent


class TestPackageSkeleton(unittest.TestCase):
    def test_package_name_matches_dir(self):
        root = ET.parse(PKG_DIR / "package.xml").getroot()
        name = root.findtext("name")
        self.assertEqual(name, PKG_DIR.name)

    def test_build_type_is_ament_cmake(self):
        root = ET.parse(PKG_DIR / "package.xml").getroot()
        build_type = root.find("export/build_type")
        self.assertIsNotNone(build_type)
        self.assertEqual(build_type.text, "ament_cmake")


if __name__ == "__main__":
    unittest.main()
