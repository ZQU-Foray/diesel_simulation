#!/usr/bin/env python3
"""A5 回归：派生 PCD 的通行投影与官方地图必须落在同一占据区（IoU 门槛）。

两条数据链相互独立，因此这个断言不是自证：
- 占据图：`maps/rmul_2027/rmul_2027.pgm`（guodong 迁移包随包提供，由 STEP 复核参数生成）；
- 点云：`scripts/make_pcd.py` 从同包分件网格 `models/rmul_2027/meshes/*.dae` 表面采样得到。

判定口径（写进 README，勿改而不留痕）：
- 点云占据：栅格内存在 z > 0.02 m 的点。2 cm 是「底盘可通行」的分界——中央控制区顶面
  10 mm、补给区垫面 3 mm 按规则手册属可通行，不记占据；围挡 420 mm、高地 200 mm、
  工作台 403 mm、隧道顶 270 mm 记占据。
- 比较范围：外场矩形（内场 12×8 m + 两侧 0.3 m 围挡）。官方 pmg 把场外 0.1 m 余量也标成
  占据，而点云不含该区域，故余量环不参与比较（否则 IoU 会被系统性拉低约 12%）。

用法：
    python3 test/test_pcd_align.py --pcd map/rmul_2027/rmul_2027.pcd \
        --grid <field_package>/maps/rmul_2027/rmul_2027.pgm [--min-iou 0.90] [--spacing 0.08]
"""

import argparse
import pathlib
import struct
import sys

# 与官方 yaml 一致；diesel_localization 的 grid_from_pgm 也用同一组阈值。
RESOLUTION = 0.025
ORIGIN = (-0.4, -0.4)
PASSABLE_HEIGHT = 0.02
# 外场外廓：内场 0..12 × 0..8，围挡厚 0.3 m 向外生长 → x -0.3..12.3, y -0.3..8.3。
OUTER_FIELD = (-0.3, -0.3, 12.3, 8.3)


def read_pcd(path):
    """读 binary PCD 的 x/y/z（只支持本仓 make_pcd.py 写出的定长格式）。"""
    blob = path.read_bytes()
    marker = b"DATA binary\n"
    split = blob.find(marker)
    if split < 0:
        raise SystemExit(f"{path} 不是 binary PCD")
    header = blob[:split].decode("ascii")
    fields = {}
    for line in header.splitlines():
        key, _, value = line.partition(" ")
        fields[key] = value.split()
    if fields.get("FIELDS") != ["x", "y", "z"]:
        raise SystemExit(f"{path} 字段不是 x/y/z：{fields.get('FIELDS')}")
    body = blob[split + len(marker) :]
    count = int(fields["POINTS"][0])
    if len(body) < count * 12:
        raise SystemExit(f"{path} 数据段长度不足")
    return list(struct.iter_unpack("<fff", body[: count * 12]))


def read_pgm(path):
    """读 P5 二进制 PGM，返回 (width, height, bytes)。"""
    blob = path.read_bytes()
    offset = 0
    tokens = []
    while len(tokens) < 4:
        end = blob.index(b"\n", offset)
        tokens.extend(blob[offset:end].split())
        offset = end + 1
    if tokens[0] != b"P5":
        raise SystemExit(f"{path} 不是 P5 PGM")
    width, height = int(tokens[1]), int(tokens[2])
    return width, height, blob[offset : offset + width * height]


def project(points, radius):
    """点云占据投影：场地坐标 → 栅格索引集合。

    点云按 `--spacing` 的格点采样，直接投影只会点亮「恰好有点」的像元，留下锯齿空隙。
    因此每个点按其采样半径（默认 spacing/2）膨胀：像元中心到点的水平距离 <= radius 即记占据。
    这是表面重建的标准做法，等价于把离散表面采样还原成连续面。
    """
    span = int(radius / RESOLUTION + 0.5)
    cells = set()
    for x, y, z in points:
        if z <= PASSABLE_HEIGHT:
            continue
        base_col = int((x - ORIGIN[0]) / RESOLUTION)
        base_row = int((y - ORIGIN[1]) / RESOLUTION)
        for d_col in range(-span, span + 1):
            for d_row in range(-span, span + 1):
                col = base_col + d_col
                row = base_row + d_row
                center_x = ORIGIN[0] + (col + 0.5) * RESOLUTION
                center_y = ORIGIN[1] + (row + 0.5) * RESOLUTION
                if (center_x - x) ** 2 + (center_y - y) ** 2 <= radius**2:
                    cells.add((col, row))
    return cells


def grid_cells(width, height, data):
    """官方地图占据像元集合（值 < 65 视为占据；255-v 的占据阈 0.65 等价）。"""
    cells = set()
    for index, value in enumerate(data):
        if value < 65:
            cells.add((index % width, (height - 1) - index // width))
    return cells


def in_outer_field(col, row):
    """栅格索引是否落在外场矩形内。"""
    x = ORIGIN[0] + (col + 0.5) * RESOLUTION
    y = ORIGIN[1] + (row + 0.5) * RESOLUTION
    x0, y0, x1, y1 = OUTER_FIELD
    return x0 <= x <= x1 and y0 <= y <= y1


def main():
    parser = argparse.ArgumentParser(description="PCD 与官方地图的一致性回归")
    parser.add_argument("--pcd", required=True, type=pathlib.Path)
    parser.add_argument("--grid", required=True, type=pathlib.Path)
    parser.add_argument("--min-iou", type=float, default=0.90)
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.08,
        help="make_pcd.py 的表面采样间隔，用于折算膨胀半径",
    )
    args = parser.parse_args()

    points = read_pcd(args.pcd.resolve())
    width, height, data = read_pgm(args.grid.resolve())

    cloud = {
        cell for cell in project(points, args.spacing / 2) if in_outer_field(*cell)
    }
    official = {
        cell for cell in grid_cells(width, height, data) if in_outer_field(*cell)
    }
    if not official:
        raise SystemExit("官方地图在外场范围内没有占据像元，口径有误")

    intersection = len(cloud & official)
    union = len(cloud | official)
    iou = intersection / union if union else 0.0
    recall = intersection / len(official)
    precision = intersection / len(cloud) if cloud else 0.0

    print(f"点云 {len(points)} 点 → 占据栅格 {len(cloud)}（外场范围内）")
    print(f"官方地图占据栅格 {len(official)}（外场范围内）")
    print(
        f"IoU={iou:.4f} 召回={recall:.4f} 精确={precision:.4f} 交集={intersection} 并集={union}"
    )
    if iou < args.min_iou:
        print(f"FAIL: IoU {iou:.4f} < 门槛 {args.min_iou:.2f}", file=sys.stderr)
        return 1
    print(f"PASS: IoU {iou:.4f} >= 门槛 {args.min_iou:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
