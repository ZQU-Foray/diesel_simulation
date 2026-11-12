#!/usr/bin/env python3
"""从场地包的分件网格派生 3D 先验地图（PCD），供 diesel_localization 消费。

设计约束（与 diesel_localization 的 PCD 加载器一致）：
- 只写 `x y z` 三个 FLOAT32 字段，DATA binary，未压缩
  （见 `diesel_localization_map/src/pcd_loader.cpp` 支持的格式集合）；
- 坐标即 `map` 帧坐标：场地内侧西南角为原点，+x 东、+y 北、z 向上，
  与官方地图 `rmul_2027.yaml` 的 origin `[-0.4, -0.4, 0]` 共用同一坐标系；
- 输出按字典序排序 + 固定头部，保证同样输入得到同样 sha256（manifest 强校验需要）。

输入只读 `models/rmul_2027/meshes/*.dae`（排除 `step_*.dae`：那是官方 STEP 合并网格，
用于外观与碰撞；分件网格与官方地图同源，二者交叉校验见 `test_pcd_align.py`）。

分件 DAE 每个只含一组三角面（顶面 + 侧面 + 底面），顶点按三角形连续排列，因此按
每 3 个顶点取一个三角形，再在三角形内按边长间隔做格点采样，得到可做 GICP 的表面点云。
"""

import argparse
import hashlib
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

COLLADA_NS = "{http://www.collada.org/2005/11/COLLADASchema}"
# 采样间隔：8 cm。定位侧 voxel_leaf_m=0.1 / map_voxel_leaf_m=0.25 会再降采样一次，
# 8 cm 已足以保留隧道 0.2495 m 净空与 0.2 m 高地，同时把 PCD 控制在 ~1.2 MB。
DEFAULT_SPACING = 0.08
PCD_HEADER = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {width}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points}
DATA binary
"""


def mesh_triangles(dae_path):
    """读出一个 DAE 的三角形列表（顶点每 3 个一组）。"""
    root = ET.parse(dae_path).getroot()
    array = root.find(f".//{COLLADA_NS}float_array[@id='pos-array']")
    if array is None or not array.text:
        raise SystemExit(f"{dae_path} 缺少 pos-array")
    values = [float(token) for token in array.text.split()]
    if len(values) % 9:
        raise SystemExit(f"{dae_path} 的顶点数不是 3 的倍数（每三角形 3 顶点）")
    vertices = [
        (values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)
    ]
    return [tuple(vertices[i : i + 3]) for i in range(0, len(vertices), 3)]


def sample_triangle(triangle, spacing):
    """在三角形内取格点；步长按最长边折算，保证小三角形至少取到 3 个顶点。"""
    a, b, c = triangle
    longest = max(
        sum((p[k] - q[k]) ** 2 for k in range(3)) ** 0.5
        for p, q in ((a, b), (b, c), (c, a))
    )
    steps = max(1, int(longest / spacing + 0.5))
    for i in range(steps + 1):
        for j in range(steps + 1 - i):
            u = i / steps
            v = j / steps
            yield tuple(a[k] + (b[k] - a[k]) * u + (c[k] - a[k]) * v for k in range(3))


def voxel_key(point, voxel):
    """体素去重用的整数键。"""
    return tuple(round(component / voxel) for component in point)


def collect(package_dir, spacing):
    """合并分件网格的表面采样点，按体素去重后返回排序点表。"""
    meshes = sorted((package_dir / "models" / "rmul_2027" / "meshes").glob("*.dae"))
    parts = [path for path in meshes if not path.name.startswith("step_")]
    if not parts:
        raise SystemExit("没有找到分件网格（step_*.dae 以外的 *.dae）")

    voxel = spacing / 2
    unique = {}
    for path in parts:
        for triangle in mesh_triangles(path):
            for point in sample_triangle(triangle, spacing):
                unique.setdefault(voxel_key(point, voxel), point)
    points = [unique[key] for key in sorted(unique)]
    return points, [path.name for path in parts]


def write_pcd(points, output):
    """按 x/y/z FLOAT32 binary 写出，返回文件 sha256。"""
    output.parent.mkdir(parents=True, exist_ok=True)
    header = PCD_HEADER.format(width=len(points), points=len(points)).encode("ascii")
    body = bytearray()
    for x, y, z in points:
        body += struct.pack("<fff", x, y, z)
    blob = header + bytes(body)
    output.write_bytes(blob)
    return hashlib.sha256(blob).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--field-package",
        required=True,
        type=pathlib.Path,
        help="diesel_rmul_2027_field 包目录",
    )
    parser.add_argument(
        "--output", required=True, type=pathlib.Path, help="输出 .pcd 路径"
    )
    parser.add_argument(
        "--spacing", type=float, default=DEFAULT_SPACING, help="表面采样间隔（米）"
    )
    args = parser.parse_args()

    points, sources = collect(args.field_package.resolve(), args.spacing)
    digest = write_pcd(points, args.output.resolve())

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    print(
        f"源网格 {len(sources)} 个，表面采样点 {len(points)}（间隔 {args.spacing} m）"
    )
    print(
        f"范围 x[{min(xs):.3f}, {max(xs):.3f}] y[{min(ys):.3f}, {max(ys):.3f}] z[{min(zs):.3f}, {max(zs):.3f}]"
    )
    print(f"输出 {args.output} sha256={digest}")


if __name__ == "__main__":
    sys.exit(main())
