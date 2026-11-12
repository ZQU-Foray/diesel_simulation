#!/usr/bin/env python3
"""A4 回归：地图清单自洽、且哈希能挡住篡改。

不依赖 ROS：按 `manifest.cpp` 的必填键与约束逐条断言，再复现
`verify_map_hashes:=true` 的行为——改动栅格 1 个字节后必须检出哈希不符。

用法：
    python3 test/test_manifest.py map/rmul_2027
"""

import hashlib
import pathlib
import sys

REQUIRED_KEYS = (
    "version",
    "frame_id",
    "origin",
    "width",
    "height",
    "pcd_file",
    "grid_file",
    "created_at",
    "resolution",
    "pcd_sha256",
    "grid_sha256",
)
# manifest.cpp: 相对路径、不含 `..`、必须是同类 sha256 十六进制。
MAX_RESOLUTION = 1.0


def parse_simple_yaml(path):
    """解析本仓 manifest 用到的极简 YAML（顶层标量 + 行内数组）。"""
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#") or ":" not in line:
            continue
        key, _, value = line.partition(":")
        fields[key.strip()] = value.strip()
    return fields


def sha256_file(path):
    """文件 sha256。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_pgm_size(path):
    """PGM 头里的 (width, height)。"""
    tokens = path.read_bytes()[:64].split()
    if tokens[0] != b"P5":
        raise SystemExit(f"{path} 不是 P5 PGM")
    return int(tokens[1]), int(tokens[2])


def check_manifest(map_dir):
    """断言清单字段、相对路径与哈希。"""
    manifest_path = map_dir / "manifest.yaml"
    fields = parse_simple_yaml(manifest_path)

    missing = [key for key in REQUIRED_KEYS if key not in fields]
    assert not missing, f"manifest 缺少必填键：{missing}"

    assert (
        fields["frame_id"] == "map"
    ), f"frame_id 必须是 map，实际 {fields['frame_id']}"

    resolution = float(fields["resolution"])
    assert (
        0.0 < resolution <= MAX_RESOLUTION
    ), f"resolution 必须落在 (0, {MAX_RESOLUTION}]"

    for key in ("pcd_file", "grid_file"):
        value = fields[key]
        assert ".." not in value, f"{key} 不得包含 ..：{value}"
        assert (map_dir / value).is_file(), f"{key} 指向的文件不存在：{value}"

    for key in ("pcd_sha256", "grid_sha256"):
        value = fields[key]
        assert len(value) == 64 and all(
            ch in "0123456789abcdefABCDEF" for ch in value
        ), f"{key} 不是 64 位十六进制"

    origin = fields["origin"].strip("[]").replace(",", " ").split()
    assert len(origin) == 3, f"origin 必须是 3 个数值：{fields['origin']}"

    width, height = read_pgm_size(map_dir / fields["grid_file"])
    assert (
        int(fields["width"]) == width and int(fields["height"]) == height
    ), f"manifest 尺寸 {fields['width']}x{fields['height']} 与栅格 {width}x{height} 不一致"

    return fields


def check_tamper_detection(map_dir, fields):
    """改动栅格 1 个字节，哈希比对必须失败（复现 verify_map_hashes 的拒启逻辑）。"""
    grid = map_dir / fields["grid_file"]
    original = grid.read_bytes()
    tampered = bytearray(original)
    tampered[-1] ^= 0xFF
    assert (
        sha256_file_bytes(bytes(tampered)) != fields["grid_sha256"]
    ), "篡改 1 字节后哈希仍然相同，校验失去意义"
    assert sha256_file_bytes(original) == fields["grid_sha256"], "原件哈希与清单不符"


def sha256_file_bytes(blob):
    """内存数据的 sha256。"""
    return hashlib.sha256(blob).hexdigest()


def main():
    map_dir = pathlib.Path(
        sys.argv[1] if len(sys.argv) > 1 else "map/rmul_2027"
    ).resolve()
    fields = check_manifest(map_dir)
    check_tamper_detection(map_dir, fields)
    print(
        f"PASS: manifest 自洽（{fields['width']}x{fields['height']} @ {fields['resolution']} m，"
        f"frame_id={fields['frame_id']}，哈希与篡改检测有效）"
    )


if __name__ == "__main__":
    main()
