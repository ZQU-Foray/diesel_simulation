#!/usr/bin/env python3
"""生成 diesel_localization 消费的地图清单（manifest.yaml）。

清单契约见 `diesel_localization_core/src/manifest.cpp`：
必填 `version` / `frame_id`(必须是 `map`) / `origin[3]` / `width` / `height` /
`pcd_file` / `grid_file` / `created_at` / `resolution` / `pcd_sha256` / `grid_sha256`；
`pcd_file`、`grid_file` 必须是清单所在目录下的相对路径（不含 `..`）。

栅格与 PCD 必须放在清单同目录：节点按 `manifest 目录 / 字段值` 定位文件，这也是
`verify_map_hashes:=true` 能做篡改检测的前提。

用法：
    python3 scripts/make_manifest.py --map-dir map/rmul_2027 --out map/rmul_2027/manifest.yaml
"""

import argparse
import hashlib
import pathlib
import sys

# 地图资产冻结日：取迁移包随附网格的 <created> 时间戳（2026-10-16T00:00:00Z），
# 让同一份输入在任何机器上生成逐字节相同的 manifest。
DEFAULT_CREATED_AT = "2026-10-16T00:00:00Z"
DEFAULT_VERSION = "1.0.0"
DEFAULT_SOURCE = "RMUL 2027 3v3 官方 STEP（规则手册 V1.0.0 §3.2）"


def sha256_file(path):
    """返回文件 sha256（小写十六进制）。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_map_yaml(path):
    """读出 nav2 地图 yaml 的 resolution/origin/image（只解析本仓用到的键）。"""
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, _, value = line.partition(":")
        fields[key.strip()] = value.strip()
    missing = {"image", "resolution", "origin"} - fields.keys()
    if missing:
        raise SystemExit(f"{path} 缺少键：{sorted(missing)}")
    origin = fields["origin"].strip("[]").replace(",", " ").split()
    if len(origin) != 3:
        raise SystemExit(f"{path} 的 origin 必须是 3 个数值：{fields['origin']}")
    return fields["image"], float(fields["resolution"]), [float(v) for v in origin]


def read_pgm_size(path):
    """返回 PGM 的 (width, height)。"""
    tokens = path.read_bytes()[:64].split()
    if tokens[0] != b"P5":
        raise SystemExit(f"{path} 不是 P5 PGM")
    return int(tokens[1]), int(tokens[2])


def main():
    parser = argparse.ArgumentParser(description="生成地图清单 manifest.yaml")
    parser.add_argument(
        "--map-dir",
        required=True,
        type=pathlib.Path,
        help="同时存放 pcd / pgm / yaml 的目录",
    )
    parser.add_argument(
        "--out", required=True, type=pathlib.Path, help="输出 manifest.yaml"
    )
    parser.add_argument(
        "--pcd", default="rmul_2027.pcd", help="地图目录内的 PCD 文件名"
    )
    parser.add_argument(
        "--map-yaml", default="rmul_2027.yaml", help="地图目录内的 nav2 地图文件名"
    )
    parser.add_argument("--version", default=DEFAULT_VERSION)
    parser.add_argument("--created-at", default=DEFAULT_CREATED_AT)
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    args = parser.parse_args()

    map_dir = args.map_dir.resolve()
    pcd = map_dir / args.pcd
    map_yaml = map_dir / args.map_yaml
    for path in (pcd, map_yaml):
        if not path.is_file():
            raise SystemExit(f"找不到 {path}（先跑 make_pcd.py 并拷入官方 pgm/yaml）")

    image, resolution, origin = read_map_yaml(map_yaml)
    grid = map_dir / image
    width, height = read_pgm_size(grid)

    lines = [
        "# diesel_localization 地图清单（由 scripts/make_manifest.py 生成，勿手改）",
        f"version: {args.version}",
        "frame_id: map",
        f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f}]",
        f"resolution: {resolution}",
        f"width: {width}",
        f"height: {height}",
        f"pcd_file: {args.pcd}",
        f"grid_file: {image}",
        f"pcd_sha256: {sha256_file(pcd)}",
        f"grid_sha256: {sha256_file(grid)}",
        f"created_at: {args.created_at}",
        f"source: {args.source}",
        "",
    ]
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"写出 {out}")
    print(f"栅格 {width}x{height} @ {resolution} m，原点 {origin}")
    print(
        f"pcd {pcd.name} {pcd.stat().st_size} 字节，栅格 {grid.name} {grid.stat().st_size} 字节"
    )


if __name__ == "__main__":
    sys.exit(main())
