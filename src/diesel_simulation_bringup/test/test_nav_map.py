#!/usr/bin/env python3
"""A3 的前置回归：派生的导航栅格必须「只开了隧道，没动别处」。

三条断言（口径写进 `decision.md` D16，勿改而不留痕）：

1. 差异有界：派生图与官方图逐像素比较，**变自由的格必须全部落在隧道开放段投影矩形内**，
   且矩形内原本被官方图全标占据（说明清的就是隧道顶，不是空转）。这一步是「派生图没有
   悄悄放过围挡/高地/工作台」的唯一机械保证——差一格就可能让规划器穿墙。
2. 通道够宽：x = 2.6/3.0/3.325/3.5/3.675/3.9/4.3 各列，y 方向存在 ≥ 0.70 m 的连续自由带
   （实测 0.75 m）。0.70 是下限：底盘宽 0.24 m + 两侧各留 0.2 m 安全余量 ≈ 0.64，再留一格
   抖动余量。
3. 矩形来源可信：由 `parameters.json` 推出来的矩形必须等于工程上认定的
   x 3.325..3.675 / y 7.06..7.86。参数改了脚本没跟，这条会先炸。

用法：
    python3 test/test_nav_map.py [--official-pgm ...] [--nav-pgm ...]
"""

import argparse
import json
import pathlib
import sys

RESOLUTION = 0.025
ORIGIN = (-0.4, -0.4)
# 隧道开放段：x 由「高地西南角 x + 金属顶西偏 = 2.5 + 0.825」起、宽 metal_strip_width=0.35；
# y 由 highland_north_y=7.06 起、长 tunnel_clear_span=0.8。与
# parameters.json 的 step_verified/red_roof_xy_bounds 的 x 分量一致。
EXPECTED_RECT = (3.325, 3.675, 7.06, 7.86)
# 走廊宽度下限：底盘 0.24 m + 0.2 m 安全余量 × 2 ≈ 0.64，取 0.70 留一格抖动。
MIN_CORRIDOR_WIDTH = 0.70
# 取样的列：隧道在 y 7.06..7.86 沿 x 贯通（高地北缘到远端支承之间），金属顶只压住
# x 3.325..3.675 这一段。此处刻意取**格中心**（3.3375 / 3.6625）而不是矩形边界值 3.325 /
# 3.675——后者正好落在光栅化后的边界格里，被顶板判为占据，测出来必然是 0，属于取错样点。
PROBE_X = (2.6, 3.0, 3.3375, 3.5, 3.6625, 3.9, 4.3)
YAML_MUST_MATCH = (
    "mode",
    "resolution",
    "origin",
    "negate",
    "occupied_thresh",
    "free_thresh",
)

TEST_DIR = pathlib.Path(__file__).resolve().parent
BRINGUP = TEST_DIR.parent
FIELD = BRINGUP.parent / "diesel_rmul_2027_field"


def read_pgm(path):
    """读 P5 二进制 PGM，返回 (width, height, maxval, data)。"""
    blob = path.read_bytes()
    offset = 0
    tokens = []
    while len(tokens) < 4:
        end = blob.index(b"\n", offset)
        tokens.extend(blob[offset:end].split())
        offset = end + 1
    if tokens[0] != b"P5":
        raise SystemExit(f"{path} 不是 P5 PGM")
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    data = blob[offset : offset + width * height]
    if len(data) != width * height:
        raise SystemExit(f"{path} 像素段长度不足：{len(data)} != {width * height}")
    return width, height, maxval, data


def read_yaml(path):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, sep, value = line.partition(":")
        if sep:
            values[key.strip()] = value.strip()
    return values


def tunnel_open_rect(parameters):
    """与 make_nav_map.py 同一条推导式（两边都必须由 parameters.json 决定）。"""
    manual = parameters["manual_reference"]
    specified = manual["specified"]
    derived = manual["derived"]
    x0 = specified["highland_southwest"][0] + specified["metal_strip_west_offset"]
    x1 = x0 + specified["metal_strip_width"]
    y0 = derived["highland_north_y"]
    y1 = y0 + specified["tunnel_clear_span"]
    return x0, x1, y0, y1


def rect_cells(width, height, rect):
    """与矩形有重叠的像元集合，row 自上而下。"""
    x0, x1, y0, y1 = rect
    cols = [
        col
        for col in range(width)
        if ORIGIN[0] + col * RESOLUTION < x1 and ORIGIN[0] + (col + 1) * RESOLUTION > x0
    ]
    rows = [
        row
        for row in range(height)
        if ORIGIN[1] + (height - 1 - row) * RESOLUTION < y1
        and ORIGIN[1] + (height - row) * RESOLUTION > y0
    ]
    return {(col, row) for col in cols for row in rows}


def is_occupied(value):
    """与 test_pcd_align.py 同一阈值：值 < 65 视为占据（255-v 的 0.65 占据阈等价）。"""
    return value < 65


def free_run_at_column(data, width, height, col, target_y):
    """某列上包含 target_y 的连续自由带长度（米）：从 target_y 所在格向上下扩展到障碍为止。"""
    row = round((target_y - ORIGIN[1]) / RESOLUTION)
    row = min(max(row, 0), height - 1)
    # PGM 自上而下存储：索引行 = height-1-row
    index_row = height - 1 - row
    if is_occupied(data[index_row * width + col]):
        return 0.0
    top = index_row
    while top - 1 >= 0 and not is_occupied(data[(top - 1) * width + col]):
        top -= 1
    bottom = index_row
    while bottom + 1 < height and not is_occupied(data[(bottom + 1) * width + col]):
        bottom += 1
    return (bottom - top + 1) * RESOLUTION


def check_rect_source(parameters):
    rect = tunnel_open_rect(parameters)
    problems = [
        f"矩形第 {index} 项：parameters.json 推出 {got:.4f}，工程口径 {want:.4f}"
        for index, (got, want) in enumerate(zip(rect, EXPECTED_RECT))
        if abs(got - want) > 1e-6
    ]
    if problems:
        raise SystemExit(
            "FAIL: 隧道开放段矩形与 parameters.json 不一致\n" + "\n".join(problems)
        )
    print(f"矩形来源 OK: x {rect[0]:.3f}..{rect[1]:.3f} y {rect[2]:.3f}..{rect[3]:.3f}")


def check_diff(official, nav):
    off_w, off_h, off_max, off_data = official
    nav_w, nav_h, nav_max, nav_data = nav
    if (off_w, off_h, off_max) != (nav_w, nav_h, nav_max):
        raise SystemExit(
            f"FAIL: 尺寸/量程不同 official=({off_w},{off_h},{off_max}) nav=({nav_w},{nav_h},{nav_max})"
        )
    grid = rect_cells(nav_w, nav_h, EXPECTED_RECT)

    opened = set()
    closed = set()
    for row in range(nav_h):
        for col in range(nav_w):
            index = row * nav_w + col
            off_occ = is_occupied(off_data[index])
            nav_occ = is_occupied(nav_data[index])
            if off_occ and not nav_occ:
                opened.add((col, row))
            elif nav_occ and not off_occ:
                closed.add((col, row))

    if closed:
        raise SystemExit(
            f"FAIL: 派生图把 {len(closed)} 个自由格改成了占据（不允许新增障碍）"
        )
    if not opened:
        raise SystemExit("FAIL: 派生图与官方图没有差异，隧道没被打开")
    outside = sorted(opened - grid)
    if outside:
        raise SystemExit(
            f"FAIL: {len(outside)} 个变自由的格落在隧道开放段之外，前 5 个：{outside[:5]}"
        )
    blocked_in_rect = {
        cell for cell in grid if is_occupied(nav_data[cell[1] * nav_w + cell[0]])
    }
    if blocked_in_rect:
        raise SystemExit(f"FAIL: 隧道开放段内仍有 {len(blocked_in_rect)} 个占据格")
    still_occupied = sum(
        1 for col, row in grid if is_occupied(off_data[row * nav_w + col])
    )
    if still_occupied:
        print(f"官方图在隧道开放段内有 {still_occupied} 个占据格，已全部清空")
    print(f"差异有界 OK: 仅 {len(opened)} 格 占据→自由，且全部落在隧道开放段内")


def check_corridor_width(nav):
    width, height, _maxval, data = nav
    target_y = (EXPECTED_RECT[2] + EXPECTED_RECT[3]) / 2
    narrow = []
    for x in PROBE_X:
        col = int((x - ORIGIN[0]) / RESOLUTION)
        run = free_run_at_column(data, width, height, col, target_y)
        print(f"x={x:<6.3f} 自由带 {run:.3f} m")
        if run < MIN_CORRIDOR_WIDTH:
            narrow.append(f"x={x} 自由带 {run:.3f} m < {MIN_CORRIDOR_WIDTH} m")
    if narrow:
        raise SystemExit("FAIL: 隧道走廊过窄\n" + "\n".join(narrow))
    print(f"走廊宽度 OK: 各列自由带 ≥ {MIN_CORRIDOR_WIDTH:.2f} m")


def check_yaml(official_yaml, nav_yaml):
    for key in YAML_MUST_MATCH:
        if official_yaml.get(key) != nav_yaml.get(key):
            raise SystemExit(
                f"FAIL: yaml 字段 {key} 与官方不一致："
                f"{nav_yaml.get(key)!r} != {official_yaml.get(key)!r}"
            )
    print(f"yaml 一致 OK: {', '.join(YAML_MUST_MATCH)}")


def main():
    parser = argparse.ArgumentParser(description="派生导航栅格的有界性回归")
    parser.add_argument(
        "--official-pgm",
        type=pathlib.Path,
        default=FIELD / "maps/rmul_2027/rmul_2027.pgm",
    )
    parser.add_argument(
        "--official-yaml",
        type=pathlib.Path,
        default=FIELD / "maps/rmul_2027/rmul_2027.yaml",
    )
    parser.add_argument(
        "--parameters",
        type=pathlib.Path,
        default=FIELD / "maps/rmul_2027/parameters.json",
    )
    parser.add_argument(
        "--nav-pgm", type=pathlib.Path, default=BRINGUP / "map/nav2/rmul_2027_nav.pgm"
    )
    parser.add_argument(
        "--nav-yaml",
        type=pathlib.Path,
        default=BRINGUP / "map/nav2/rmul_2027_nav.yaml",
    )
    args = parser.parse_args()

    parameters = json.loads(args.parameters.read_text(encoding="utf-8"))
    official = read_pgm(args.official_pgm)
    nav = read_pgm(args.nav_pgm)

    check_rect_source(parameters)
    check_yaml(read_yaml(args.official_yaml), read_yaml(args.nav_yaml))
    check_diff(official, nav)
    check_corridor_width(nav)
    print("PASS: 派生导航栅格只打开了隧道开放段，走廊宽度满足 A3 规划需要")
    return 0


if __name__ == "__main__":
    sys.exit(main())
