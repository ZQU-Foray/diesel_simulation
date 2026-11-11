#!/usr/bin/env python3
"""从官方地图派生 Nav2 用的「车体可通行」栅格（`map/nav2/`）。

为什么需要第二张图
------------------
官方 `maps/rmul_2027/rmul_2027.pgm` 的占据判据是「高于 2 cm 即不可通行」：围挡 420 mm、
高地 200 mm、工作台 403 mm、隧道顶 270 mm 一律记占据（口径写在
`test/test_pcd_align.py` 文件头，A5 在守它，A4 的 manifest 哈希也钉着它）。对定位侧这是
正确且必须保持不变的口径；但 Nav2 需要的是**底盘可通行**口径——仿车底盘只有 0.15 m 高、
雷达装在 0.27962 m，隧道 0.25 m 净空物理上可以穿。直接拿官方图规划，机器人永远进不了隧道。

场地包自己在 `maps/rmul_2027/parameters.json` 的 `limitations` 里写明了这件事：

    The supplied PGM conservatively blocks highlands and tunnel roof footprints for this
    planar robot. It is a model projection, not a SLAM result or localization-validated map.
    A lidar-only occupancy slice differs from a chassis traversability map.

因此本脚本**不改官方图**（AGENTS.md 禁改用户素材，A4/A5 的契约也不能动），而是派生一张
导航图：逐像素复制官方图，只把「隧道开放段」的投影矩形清成自由，其余一格不动。差异因此
可审计——`test/test_nav_map.py` 断言差异格全部落在这个矩形内。

清哪里、为什么是这个矩形
------------------------
隧道开放段（口径全部来自 `parameters.json`，本脚本不写死数值）：

- x：高地西南角 x + `metal_strip_west_offset` = 2.5 + 0.825 = 3.325，宽 `metal_strip_width`
  = 0.35 → 3.325..3.675（与 `step_verified/red_roof_xy_bounds` 的 x 分量一致）；
- y：`highland_north_y` = 7.06 起，长 `tunnel_clear_span` = 0.8 → 7.06..7.86。

7.86 以北的 140 mm 是 `tunnel_far_support`（z 0..0.25，真实不可穿越障碍），必须保持占据，
所以矩形止于开放段末端，不按金属顶的 STEP 外廓 6.98..7.94 清。

输出必须可复现
--------------
同样的输入得到同样的字节（固定头部、逐字节复制、无时间戳），便于把 sha256 写进回执。

用法：
    python3 make_nav_map.py \
        --official-pgm <field>/maps/rmul_2027/rmul_2027.pgm \
        --official-yaml <field>/maps/rmul_2027/rmul_2027.yaml \
        --parameters <field>/maps/rmul_2027/parameters.json \
        --out-pgm src/foray_simulation_bringup/map/nav2/rmul_2027_nav.pgm \
        --out-yaml src/foray_simulation_bringup/map/nav2/rmul_2027_nav.yaml
"""

import argparse
import hashlib
import json
import pathlib
import sys

# 官方图与 yaml 的栅格参数必须一致，漂移时直接报错而不是悄悄派生错图
EXPECTED_RESOLUTION = 0.025
EXPECTED_ORIGIN = (-0.4, -0.4)


def read_pgm(path):
    """读 P5 二进制 PGM，返回 (width, height, maxval, data)。

    与 `test/test_pcd_align.py` 同一个读取口径：头部四个 token 后即像素区，
    行优先、自上而下（第 0 行是 y 最大处）。
    """
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
    data = bytearray(blob[offset : offset + width * height])
    if len(data) != width * height:
        raise SystemExit(f"{path} 像素段长度不足：{len(data)} != {width * height}")
    return width, height, maxval, data


def write_pgm(path, width, height, maxval, data):
    """写 P5 二进制 PGM；头部固定，保证同输入同字节。"""
    header = f"P5\n{width} {height}\n{maxval}\n".encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + bytes(data))


def read_pgm_yaml(path):
    """读地图 yaml 的关键字段（只取派生图必须与官方完全一致的那几项）。"""
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, sep, value = line.partition(":")
        if sep:
            values[key.strip()] = value.strip()
    return values


def tunnel_open_rect(parameters):
    """隧道开放段投影矩形 (x0, x1, y0, y1)，数值全部来自场地包 parameters.json。"""
    manual = parameters["manual_reference"]
    specified = manual["specified"]
    derived = manual["derived"]
    x0 = specified["highland_southwest"][0] + specified["metal_strip_west_offset"]
    x1 = x0 + specified["metal_strip_width"]
    y0 = derived["highland_north_y"]
    y1 = y0 + specified["tunnel_clear_span"]
    return x0, x1, y0, y1


def rect_cells(width, height, resolution, origin, rect):
    """与矩形有任何重叠的像元索引集合（col, row），row 自上而下。

    用「有重叠」而不是「中心落在矩形内」：官方图是场地网格光栅化出来的，占据格可能只
    压到矩形半边；中心判据会留下锯齿状残留，正好把 0.8 m 通道卡死。
    """
    x0, x1, y0, y1 = rect
    cols = [
        col
        for col in range(width)
        if origin[0] + col * resolution < x1 and origin[0] + (col + 1) * resolution > x0
    ]
    rows = [
        row
        for row in range(height)
        if origin[1] + (height - 1 - row) * resolution < y1
        and origin[1] + (height - row) * resolution > y0
    ]
    return {(col, row) for col in cols for row in rows}


def main():
    parser = argparse.ArgumentParser(description="派生 Nav2 可通行栅格")
    parser.add_argument("--official-pgm", required=True, type=pathlib.Path)
    parser.add_argument("--official-yaml", required=True, type=pathlib.Path)
    parser.add_argument("--parameters", required=True, type=pathlib.Path)
    parser.add_argument("--out-pgm", required=True, type=pathlib.Path)
    parser.add_argument("--out-yaml", required=True, type=pathlib.Path)
    args = parser.parse_args()

    official_pgm = args.official_pgm.resolve()
    width, height, maxval, data = read_pgm(official_pgm)

    yaml_values = read_pgm_yaml(args.official_yaml.resolve())
    resolution = float(yaml_values["resolution"])
    origin = tuple(float(item) for item in yaml_values["origin"].strip("[]").split(","))
    # origin 是 (x, y, yaw)，本仓只用前两项；多出来的 yaw 必须为 0，否则像元索引不再是
    # 轴对齐的，下面的行列换算会失效。
    if abs(resolution - EXPECTED_RESOLUTION) > 1e-9:
        raise SystemExit(f"官方图分辨率 {resolution} 与预期 {EXPECTED_RESOLUTION} 不符")
    if (
        abs(origin[0] - EXPECTED_ORIGIN[0]) > 1e-9
        or abs(origin[1] - EXPECTED_ORIGIN[1]) > 1e-9
    ):
        raise SystemExit(f"官方图 origin {origin[:2]} 与预期 {EXPECTED_ORIGIN} 不符")
    if len(origin) > 2 and abs(origin[2]) > 1e-9:
        raise SystemExit(f"官方图 origin yaw={origin[2]} 不为 0，不支持旋转栅格")

    parameters = json.loads(args.parameters.resolve().read_text(encoding="utf-8"))
    rect = tunnel_open_rect(parameters)
    cells = rect_cells(width, height, resolution, origin[:2], rect)

    # rect_cells 返回的 row 已经是「自上而下」的行号（与 PGM 存储顺序一致），别再翻转一次
    for col, row in cells:
        data[row * width + col] = 255

    write_pgm(args.out_pgm.resolve(), width, height, maxval, data)

    # yaml 只改 image 一行，其余（resolution/origin/mode/negate/thresholds）逐字沿用官方，
    # 保证两张图和定位侧的读取路径完全同构。
    out_yaml = []
    for line in args.official_yaml.read_text(encoding="utf-8").splitlines():
        if line.split(":")[0].strip() == "image":
            out_yaml.append(f"image: {args.out_pgm.name}")
        else:
            out_yaml.append(line)
    args.out_yaml.resolve().parent.mkdir(parents=True, exist_ok=True)
    args.out_yaml.resolve().write_text("\n".join(out_yaml) + "\n", encoding="utf-8")

    digest = hashlib.sha256(args.out_pgm.resolve().read_bytes()).hexdigest()
    x0, x1, y0, y1 = rect
    print(f"官方图 {official_pgm.name} {width}x{height} 派生 → {args.out_pgm.name}")
    print(
        f"隧道开放段 x {x0:.3f}..{x1:.3f} y {y0:.3f}..{y1:.3f} → 清空 {len(cells)} 格"
    )
    print(f"sha256 {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
