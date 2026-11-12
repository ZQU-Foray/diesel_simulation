#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""profile 单测：场地/机器人规格读取、路径解析、视场一致性校验。

用临时目录伪造 `share/config/{fields,robots}` 结构，并用注入的 `share_lookup`
替身模拟 ament 索引——这样单测不依赖 colcon 安装产物。
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diesel_sim_tools import AssemblyError, profile

ROBOT_YAML = """name: sentry
model: diesel_sim_robot
sdf: {package: bringup, path: models/diesel_sim_robot/model.sdf}
frames:
  base: base_footprint
  body: body
lidar:
  link: front_mid360
  sensor: front_mid360_lidar
  imu_sensor: front_mid360_imu
  frame: front_mid360
  pose: [0.08, 0.0, 0.27962, 0.0, 0.0, 0.0]
  channels: 32
  horizontal_samples: 1875
  horizontal_fov_deg: 360.0
  vertical_fov_deg: 59.0
  ang_res_y: 1.9032258064516129
  ang_bottom: 7.0
"""

FIELD_YAML = """name: rmul_2027
world: {package: assets, path: worlds/rmul_2027.sdf}
models: {package: assets, path: models}
maps:
  official: {package: assets, path: maps/rmul_2027/rmul_2027.yaml}
  nav: {package: bringup, path: map/nav2/rmul_2027_nav.yaml}
  localization: {package: bringup, path: map/rmul_2027}
nav2_params: config/nav2_params_sim.yaml
acceptance:
  goal: {x: 2.0, y: 7.4, yaw: 3.141592653589793}
  tunnel_rect: [3.325, 3.675, 7.06, 7.86]
"""


def _tree(root, kind, name, text):
    path = os.path.join(root, "config", kind, f"{name}.yaml")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(text)
    return path


def _lookup(root):
    return lambda package: {"assets": os.path.join(root, "assets"), "bringup": root}[
        package
    ]


def test_load_robot():
    root = tempfile.mkdtemp(prefix="simprofile_")
    path = _tree(root, "robots", "sentry", ROBOT_YAML)
    robot = profile.load_robot(path, share_lookup=_lookup(root))
    assert robot.name == "sentry", robot.name
    assert robot.model == "diesel_sim_robot", robot.model
    assert robot.sdf == os.path.join(
        root, "models/diesel_sim_robot/model.sdf"
    ), robot.sdf
    assert robot.base_frame == "base_footprint" and robot.body_frame == "body"
    assert robot.lidar.channels == 32 and robot.lidar.horizontal_samples == 1875
    assert robot.lidar.horizontal_fov_deg == 360.0
    assert abs(robot.lidar.ang_res_y - 59 / 31) < 1e-12, robot.lidar.ang_res_y
    assert robot.lidar.ang_bottom == 7.0
    assert len(robot.lidar.pose) == 6 and robot.lidar.pose[2] == 0.27962
    print("load_robot: 字段与绝对路径 OK")


def test_robot_angle_mismatch_is_loud():
    root = tempfile.mkdtemp(prefix="simprofile_")
    broken = ROBOT_YAML.replace("1.9032258064516129", "3.0")
    path = _tree(root, "robots", "sentry", broken)
    try:
        profile.load_robot(path, share_lookup=_lookup(root))
    except AssemblyError as error:
        assert "ang_res_y" in str(error), error
    else:
        raise AssertionError("ang_res_y 与视场不一致时必须抛 AssemblyError")
    print("load_robot: 视场与角分辨率不一致即失败 OK")


def test_ang_bottom_sign_is_normalized():
    root = tempfile.mkdtemp(prefix="simprofile_")
    path = _tree(
        root,
        "robots",
        "sentry",
        ROBOT_YAML.replace("ang_bottom: 7.0", "ang_bottom: -7.0"),
    )
    robot = profile.load_robot(path, share_lookup=_lookup(root))
    assert robot.lidar.ang_bottom == 7.0, robot.lidar.ang_bottom
    print("load_robot: ang_bottom 取绝对值（D11 口径）OK")


def test_load_field():
    root = tempfile.mkdtemp(prefix="simprofile_")
    path = _tree(root, "fields", "rmul_2027", FIELD_YAML)
    field = profile.load_field(path, share_lookup=_lookup(root))
    assert field.name == "rmul_2027", field.name
    assert field.world == os.path.join(root, "assets/worlds/rmul_2027.sdf"), field.world
    assert field.models_dir == os.path.join(root, "assets/models"), field.models_dir
    assert field.official_map.endswith(
        "assets/maps/rmul_2027/rmul_2027.yaml"
    ), field.official_map
    assert field.nav_map == os.path.join(
        root, "map/nav2/rmul_2027_nav.yaml"
    ), field.nav_map
    assert field.localization_dir == os.path.join(
        root, "map/rmul_2027"
    ), field.localization_dir
    assert field.nav2_params == os.path.join(root, "config/nav2_params_sim.yaml")
    assert field.goal == (2.0, 7.4, 3.141592653589793), field.goal
    assert field.tunnel_rect == (3.325, 3.675, 7.06, 7.86), field.tunnel_rect
    print("load_field: 路径/目标点/隧道窗口 OK")


def test_field_validation():
    root = tempfile.mkdtemp(prefix="simprofile_")
    bad_rect = FIELD_YAML.replace("[3.325, 3.675, 7.06, 7.86]", "[3.325, 3.675, 7.06]")
    path = _tree(root, "fields", "rmul_2027", bad_rect)
    try:
        profile.load_field(path, share_lookup=_lookup(root))
    except AssemblyError as error:
        assert "tunnel_rect" in str(error), error
    else:
        raise AssertionError("tunnel_rect 长度错误必须抛 AssemblyError")

    unknown = _tree(root, "fields", "bad", FIELD_YAML + "robot: sentry\n")
    try:
        profile.load_field(unknown, share_lookup=_lookup(root))
    except AssemblyError as error:
        assert "未知字段" in str(error), error
    else:
        raise AssertionError("未知字段必须抛 AssemblyError")
    print("load_field: tunnel_rect 长度与未知字段校验 OK")


def test_assembly_context():
    root = tempfile.mkdtemp(prefix="simprofile_")
    robot = profile.load_robot(
        _tree(root, "robots", "sentry", ROBOT_YAML), share_lookup=_lookup(root)
    )
    field = profile.load_field(
        _tree(root, "fields", "rmul_2027", FIELD_YAML), share_lookup=_lookup(root)
    )
    context = profile.assembly_context(field, robot)
    assert context == {
        "world": "rmul_2027",
        "robot": "diesel_sim_robot",
        "lidar_link": "front_mid360",
        "lidar_sensor": "front_mid360_lidar",
        "lidar_imu_sensor": "front_mid360_imu",
    }, context
    print("assembly_context: 只暴露场地/机器人相关占位符 OK")


def main():
    test_load_robot()
    test_robot_angle_mismatch_is_loud()
    test_ang_bottom_sign_is_normalized()
    test_load_field()
    test_field_validation()
    test_assembly_context()
    print("PASS test_profile")


if __name__ == "__main__":
    main()
