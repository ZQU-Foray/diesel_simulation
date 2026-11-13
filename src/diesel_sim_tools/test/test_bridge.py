#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""bridge 单测：契约校验、模板渲染、残留占位符、参数与 remappings 生成。

重点守 D20 那类静默错接：占位符写错或 profile 缺字段时**必须抛错**，
而不是生成一条看起来正常的桥接参数。
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diesel_sim_tools import AssemblyError, bridge

CONTEXT = {
    "world": "rmul_2027",
    "robot": "diesel_sim_robot",
    "lidar_link": "front_mid360",
    "lidar_sensor": "front_mid360_lidar",
}

LIDAR_ENTRY = (
    "- id: lidar\n"
    "  gz_topic: /world/{world}/model/{robot}/link/{lidar_link}"
    "/sensor/{lidar_sensor}/scan/points\n"
    "  ros_topic: /livox/lidar\n"
    "  ros_type: sensor_msgs/msg/PointCloud2\n"
    "  gz_type: ignition.msgs.PointCloudPacked\n"
    "  direction: gz2ros\n"
)


def _write(text):
    handle, path = tempfile.mkstemp(prefix="bridge_", suffix=".yaml")
    os.close(handle)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(text)
    return path


def test_render_expands_placeholders():
    entries = bridge.load(_write(LIDAR_ENTRY))
    assert len(entries) == 1 and entries[0].id == "lidar", entries
    assert entries[0].required is True, entries[0].required
    expected_gz_topic = (
        "/world/rmul_2027/model/diesel_sim_robot/link/front_mid360"
        "/sensor/front_mid360_lidar/scan/points"
    )
    expected_argument = (
        f"{expected_gz_topic}"
        "@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"
    )
    arguments, remappings = bridge.render_entries(entries, CONTEXT)
    assert arguments == [expected_argument], arguments
    assert remappings == [(expected_gz_topic, "/livox/lidar")], remappings
    print("render: gz2ros 参数与 remapping OK")


def test_direction_marks():
    for direction, mark in (("gz2ros", "["), ("ros2gz", "]")):
        text = LIDAR_ENTRY.replace("gz2ros", direction)
        arguments, _ = bridge.render_entries(bridge.load(_write(text)), CONTEXT)
        assert f"{mark}ignition.msgs.PointCloudPacked" in arguments[0], arguments
    print("render: 方向符号 gz2ros=[ / ros2gz=] OK")


def test_missing_field_is_loud():
    entries = bridge.load(_write(LIDAR_ENTRY))
    broken = dict(CONTEXT, lidar_sensor=None)
    del broken["lidar_sensor"]
    try:
        bridge.render_entries(entries, broken)
    except AssemblyError as error:
        message = str(error)
        assert "`lidar`" in message, message
        assert "lidar_sensor" in message, message
    else:
        raise AssertionError("缺 profile 字段时必须抛 AssemblyError")
    print("render: 缺字段报出条目 id 与字段名 OK")


def test_leftover_placeholder_is_loud():
    text = LIDAR_ENTRY.replace("{lidar_link}", "{{lidar_link}}")
    try:
        bridge.render_entries(bridge.load(_write(text)), CONTEXT)
    except AssemblyError as error:
        assert "未替换占位符" in str(error), error
    else:
        raise AssertionError("残留占位符时必须抛 AssemblyError")
    assert bridge.unresolved("a{b}c") == ["b"], bridge.unresolved("a{b}c")
    assert bridge.unresolved("abc") == []
    print("render: 残留占位符即失败 OK")


def test_contract_validation():
    cases = {
        "未知字段": LIDAR_ENTRY + "  expects:\n    required: true\n",
        "缺少 id": LIDAR_ENTRY.replace("- id: lidar\n  gz_topic:", "- gz_topic:"),
        "非法方向": LIDAR_ENTRY.replace("gz2ros", "gzboth"),
        "重复 id": LIDAR_ENTRY + LIDAR_ENTRY,
    }
    for label, text in cases.items():
        try:
            bridge.load(_write(text))
        except AssemblyError:
            continue
        raise AssertionError(f"{label} 必须被拒绝")
    print("load: 未知字段 / 缺 id / 非法方向 / 重复 id 全部拒绝 OK")


def test_optional_entry():
    text = LIDAR_ENTRY + "  expect:\n    required: false\n"
    entries = bridge.load(_write(text))
    assert entries[0].required is False, entries[0].required
    print("load: expect.required=false 生效 OK")


def test_same_topic_has_no_remapping():
    text = LIDAR_ENTRY.replace("  ros_topic: /livox/lidar\n", "")
    arguments, remappings = bridge.render_entries(bridge.load(_write(text)), CONTEXT)
    assert len(arguments) == 1, arguments
    assert remappings == [], remappings
    print("render: ROS 侧与 gz 侧同名时不产生 remapping OK")


CONTRACT = (
    LIDAR_ENTRY + "- id: clock\n"
    "  gz_topic: /clock\n"
    "  ros_type: rosgraph_msgs/msg/Clock\n"
    "  gz_type: ignition.msgs.Clock\n"
    "  direction: gz2ros\n" + "- id: cmd_vel\n"
    "  gz_topic: /model/{robot}/cmd_vel\n"
    "  ros_topic: /diesel/sentry/cmd_vel\n"
    "  ros_type: geometry_msgs/msg/Twist\n"
    "  gz_type: ignition.msgs.Twist\n"
    "  direction: ros2gz\n"
)


def test_ros_endpoints():
    entries = bridge.load(_write(CONTRACT))
    endpoints = {item[0]: item for item in bridge.ros_endpoints(entries, CONTEXT)}
    assert endpoints["lidar"][1] == "/livox/lidar", endpoints["lidar"]
    assert endpoints["lidar"][2] == "gz2ros"
    assert endpoints["cmd_vel"][1] == "/diesel/sentry/cmd_vel", endpoints["cmd_vel"]
    assert endpoints["cmd_vel"][2] == "ros2gz"
    # 没写 ros_topic 的条目，ROS 侧就是渲染后的话题名
    assert endpoints["clock"][1] == "/clock", endpoints["clock"]
    assert endpoints["clock"][3] is True
    print("ros_endpoints: ROS 侧话题 / 方向 / 必需标记 OK")


def main():
    test_render_expands_placeholders()
    test_direction_marks()
    test_missing_field_is_loud()
    test_leftover_placeholder_is_loud()
    test_contract_validation()
    test_optional_entry()
    test_same_topic_has_no_remapping()
    print("PASS test_bridge")


if __name__ == "__main__":
    main()
