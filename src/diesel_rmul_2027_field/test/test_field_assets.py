#!/usr/bin/env python3
"""RMUL 2027 场地资源包的一致性断言（不需要 Gazebo 运行时）。

用法：
    python3 test/test_field_assets.py <package_dir>

断言覆盖四类事实：
1. 世界文件已移植到 Fortress 的插件名前缀，且去掉了上游占位机器人；
2. 官方地图的几何（分辨率、原点、像素尺寸）与 STEP 复核参数自洽；
3. 网格引用可在包内解析（防止资产搬移时丢文件）；
4. 场地的关键尺寸为官方 STEP R1 的复核值（手册 V1.0.0 §3.2）。
"""

import json
import pathlib
import sys
import xml.etree.ElementTree as ET

# STEP 复核值：RMUL 2027 3V3 规则手册 V1.0.0（20261015）§3.2，由官方 STEP
# RMUL2027_3V3_V1.0.0.stp 复核（见 maps/rmul_2027/parameters.json 的 step_verified）。
INNER_FIELD = (12.0, 8.0)
PERIMETER_THICKNESS = 0.3
PERIMETER_HEIGHT = 0.42
TUNNEL_CLEARANCE = 0.2495
MAP_RESOLUTION = 0.025
MAP_ORIGIN = (-0.4, -0.4)
FLOOR_FRICTION_MU = 0.8
# 地图在场地外留 0.1 m 余量：(12.0 + 2*0.3) + 2*0.1 = 12.8
EXPECTED_MAP_METERS = (
    INNER_FIELD[0] + 2 * PERIMETER_THICKNESS + 0.2,
    INNER_FIELD[1] + 2 * PERIMETER_THICKNESS + 0.2,
)
TOLERANCE = 0.02


def read_pgm_size(path):
    """返回 (width, height)，只解析 PGM 头。"""
    data = path.read_bytes()[:64]
    tokens = data.split()
    assert tokens[0] == b"P5", f"{path} 不是二进制 PGM"
    return int(tokens[1]), int(tokens[2])


def check_world(root):
    """世界文件：Fortress 插件前缀 + 单一场地模型 + 无上游占位机器人。"""
    world = root.find("world")
    assert world is not None, "world 元素缺失"
    assert (
        world.get("name") == "rmul_2027"
    ), f"世界名应为 rmul_2027，实际 {world.get('name')}"

    plugins = [
        p
        for p in world.findall("plugin")
        if p.get("filename", "").startswith("ignition-gazebo-")
    ]
    names = {p.get("filename") for p in plugins}
    expected = {
        "ignition-gazebo-physics-system",
        "ignition-gazebo-user-commands-system",
        "ignition-gazebo-scene-broadcaster-system",
        "ignition-gazebo-sensors-system",
        "ignition-gazebo-imu-system",
    }
    assert expected <= names, f"缺少 Fortress 系统插件：{expected - names}"

    imu = [p for p in plugins if p.get("filename") == "ignition-gazebo-imu-system"]
    assert imu, "世界必须加载 imu-system，否则 gpu_lidar 正常但 IMU 话题不会出现"
    sensors = [
        p for p in plugins if p.get("filename") == "ignition-gazebo-sensors-system"
    ]
    assert (
        sensors[0].findtext("render_engine") == "ogre2"
    ), "Sensors 插件必须显式指定 ogre2 渲染引擎"

    text = ET.tostring(world, encoding="unicode")
    assert "gz-sim-" not in text, "世界文件仍含 Harmonic 插件前缀 gz-sim-"
    assert (
        "guodong_sentry" not in text
    ), "世界文件仍引用上游占位机器人 model://guodong_sentry"

    uris = [i.findtext("uri") for i in world.findall("include")]
    assert uris == ["model://rmul_2027"], f"include 应只有场地模型，实际 {uris}"


def check_map(maps_dir):
    """地图：分辨率/原点与 PGM 像素尺寸折算后必须等于场地外廓。"""
    yaml_text = (maps_dir / "rmul_2027.yaml").read_text(encoding="utf-8")
    fields = {}
    for line in yaml_text.splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            key, _, value = line.partition(":")
            fields[key.strip()] = value.strip()

    assert fields["image"] == "rmul_2027.pgm", f"image 字段异常：{fields['image']}"
    assert fields["mode"] == "trinary", f"mode 字段异常：{fields['mode']}"
    assert (
        float(fields["resolution"]) == MAP_RESOLUTION
    ), f"分辨率异常：{fields['resolution']}"
    origin = json.loads(fields["origin"].replace(" ", ""))
    assert tuple(origin[:2]) == MAP_ORIGIN, f"原点异常：{origin}"

    width, height = read_pgm_size(maps_dir / fields["image"])
    meters = (width * MAP_RESOLUTION, height * MAP_RESOLUTION)
    for got, want, label in zip(meters, EXPECTED_MAP_METERS, ("宽", "高")):
        assert (
            abs(got - want) < TOLERANCE
        ), f"地图{label}度 {got:.2f} m 与场地外廓 {want:.2f} m 不符"

    parameters = json.loads((maps_dir / "parameters.json").read_text(encoding="utf-8"))
    verified = parameters["step_verified"]
    assert tuple(verified["inner_field_size"]) == INNER_FIELD, "内场尺寸与手册不符"
    assert (
        verified["perimeter_thickness"] == PERIMETER_THICKNESS
    ), "围挡厚度与 STEP 复核值不符"
    assert (
        verified["perimeter_height"] == PERIMETER_HEIGHT
    ), "围挡高度与 STEP 复核值不符"
    assert (
        abs(verified["tunnel_geometric_clearance"] - TUNNEL_CLEARANCE) < 1e-6
    ), "隧道净空与复核值不符"
    effective = parameters["effective_simulation"]
    assert (
        effective["map_resolution"] == MAP_RESOLUTION
    ), "parameters.json 的分辨率与 yaml 不一致"
    assert effective["floor_friction_mu"] == FLOOR_FRICTION_MU, "地面摩擦系数与模型不符"


def check_model(models_dir):
    """模型：静态、网格引用可解析、摩擦与参数一致。"""
    model_root = ET.parse(models_dir / "rmul_2027" / "model.sdf").getroot()
    model = model_root.find("model")
    assert model.findtext("static") == "true", "场地模型必须是 static"

    uris = [e.text for e in model.iter("uri")]
    assert uris, "模型没有引用任何网格"
    for uri in uris:
        assert uri.startswith("model://rmul_2027/"), f"网格 URI 不在本模型内：{uri}"
        relative = uri[len("model://rmul_2027/") :]
        assert (
            models_dir / "rmul_2027" / relative
        ).is_file(), f"网格文件缺失：{relative}"

    mu = model.findtext(".//friction/ode/mu")
    assert (
        mu is not None and abs(float(mu) - FLOOR_FRICTION_MU) < 1e-9
    ), "摩擦系数与 parameters.json 不一致"

    config = ET.parse(models_dir / "rmul_2027" / "model.config").getroot()
    assert config.findtext("name"), "model.config 缺少 name"
    sdf_ref = config.find("sdf")
    assert (
        models_dir / "rmul_2027" / sdf_ref.text
    ).is_file(), "model.config 指向的 sdf 不存在"


def main():
    package_dir = pathlib.Path(sys.argv[1]).resolve()
    check_world(ET.parse(package_dir / "worlds" / "rmul_2027.sdf").getroot())
    check_map(package_dir / "maps" / "rmul_2027")
    check_model(package_dir / "models")
    print(
        "PASS: RMUL 2027 场地资源一致（世界插件前缀 / 地图几何 / 网格引用 / STEP 复核值）"
    )


if __name__ == "__main__":
    main()
