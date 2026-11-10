#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
#
# rviz / gz GUI 配置的结构门禁（I11 起）：宿主即可跑，不需要 ROS、RViz 或 gazebo。
#
# 守两类静默故障：
#   1) 手写 .rviz 里的话题名打错 —— RViz 不会报错，只会让那个显示项永远没数据；
#   2) gui.config 里插件类名写错 —— gz 直接丢掉该面板，窗口里一声不响。
# 所以这里把「话题白名单」「必需显示项」「已有最小集不许被改动」写成断言。
# 窗口真的开出来这件事只能在容器里取证，不在本文件的职责范围。

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
RVIZ_DIR = PKG_DIR / "rviz"
GUI_CONFIG = PKG_DIR / "config" / "gui.config"

# 允许出现在 .rviz 里的话题，逐个注明来源。
ALLOWED_TOPICS = {
    "/robot_description": "robot_state_publisher（simulation.launch.py）",
    "/livox/lidar": "config/bridge.yaml",
    "/livox/imu": "config/bridge.yaml",
    "/cloud_registered": "navigation_sim.launch.py 的 LOAM_INTERFACE_PARAMS",
    "/lidar_odometry": "navigation_sim.launch.py 的 LOAM_INTERFACE_PARAMS",
    "/aft_mapped_to_init": "point_lio 里程计",
    "/odometry": "sensor_scan_generation（nav2 odom_topic）",
    "/map": "foray_map_server / nav2 map_server",
    "/global_costmap/costmap": "nav2 costmap（global）",
    "/local_costmap/costmap": "nav2 costmap（local）",
    "/local_costmap/published_footprint": "nav2 costmap footprint",
    "/plan": "nav2 planner_server",
    "/local_plan": "nav2 controller_server",
    "/initialpose": "RViz SetInitialPose 工具",
    "/goal_pose": "RViz SetGoal 工具",
    "/clicked_point": "RViz PublishPoint 工具",
}

# sim.rviz 是验收最小集（A0/A2 的取证口径），显示项不许被顺手改掉。
SIM_RVIZ_DISPLAYS = {
    ("rviz_default_plugins/TF", None),
    ("rviz_default_plugins/PointCloud2", "/livox/lidar"),
    ("rviz_default_plugins/Map", "/map"),
}

# gz 插件类名白名单：按上游 ign-gui6 / ign-gazebo6 的插件目录核对得出，
# 容器内最终以 `ign gui -l` 的输出为准。
KNOWN_GZ_GUI_PLUGINS = {
    "MinimalScene",
    "GzSceneManager",
    "WorldControl",
    "WorldStats",
    "EntityTree",
    "ComponentInspector",
    "TransformControl",
    "ViewAngle",
    "ImageDisplay",
    "Plotting",
    "TopicViewer",
}

REQUIRED_TUNE_DISPLAYS = {
    ("rviz_default_plugins/RobotModel", "/robot_description"),
    ("rviz_default_plugins/TF", None),
    ("rviz_default_plugins/PointCloud2", "/cloud_registered"),
    ("rviz_default_plugins/Map", "/map"),
    ("rviz_default_plugins/Map", "/global_costmap/costmap"),
    ("rviz_default_plugins/Map", "/local_costmap/costmap"),
    ("rviz_default_plugins/Polygon", "/local_costmap/published_footprint"),
    ("rviz_default_plugins/Path", "/plan"),
    ("rviz_default_plugins/Path", "/local_plan"),
    ("rviz_default_plugins/Odometry", "/odometry"),
}

REQUIRED_TUNE_TOOLS = {
    "rviz_default_plugins/SetInitialPose",
    "rviz_default_plugins/SetGoal",
    "rviz_default_plugins/PublishPoint",
    "rviz_default_plugins/Measure",
}


class CheckError(AssertionError):
    """断言失败（沿用 assert 的语义，但带可读前缀）。"""


def load_rviz(name):
    path = RVIZ_DIR / name
    if not path.is_file():
        raise CheckError(f"缺少 RViz 配置：{path}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def manager(config, name):
    try:
        return config["Visualization Manager"]
    except (KeyError, TypeError):
        raise CheckError(f"{name} 缺少 Visualization Manager 段") from None


def display_pairs(config, name):
    """取 (Class, 话题值) 集合；没有话题的显示项返回 None。

    RobotModel 的话题在 `Description Topic` 下（其余显示项统一是 `Topic`）。
    """
    pairs = set()
    for display in manager(config, name).get("Displays") or []:
        topic = display.get("Topic") or display.get("Description Topic")
        value = topic.get("Value") if isinstance(topic, dict) else None
        pairs.add((display.get("Class"), value))
    return pairs


def check_sim_rviz():
    config = load_rviz("sim.rviz")
    pairs = display_pairs(config, "sim.rviz")
    if pairs != SIM_RVIZ_DISPLAYS:
        raise CheckError(
            "sim.rviz 的显示项被改动了（它是验收最小集，调车视图请放 tune.rviz）："
            f"期望 {sorted(SIM_RVIZ_DISPLAYS)}，实际 {sorted(pairs)}"
        )
    fixed = manager(config, "sim.rviz").get("Global Options", {}).get("Fixed Frame")
    if fixed != "map":
        raise CheckError(f"sim.rviz 的 Fixed Frame 应为 map，实际 {fixed!r}")


def check_tune_rviz():
    config = load_rviz("tune.rviz")
    view = manager(config, "tune.rviz")
    fixed = view.get("Global Options", {}).get("Fixed Frame")
    if fixed != "map":
        raise CheckError(f"tune.rviz 的 Fixed Frame 应为 map，实际 {fixed!r}")

    pairs = display_pairs(config, "tune.rviz")
    missing = REQUIRED_TUNE_DISPLAYS - pairs
    if missing:
        raise CheckError(f"tune.rviz 缺少必需显示项：{sorted(missing)}")

    unknown = {
        value for _, value in pairs if value is not None and value not in ALLOWED_TOPICS
    }
    if unknown:
        raise CheckError(
            f"tune.rviz 用了白名单外的话题（多半是笔误，RViz 会静默显示空数据）：{sorted(unknown)}；"
            f"确有新话题请把它加进 ALLOWED_TOPICS 并注明来源"
        )

    tools = {tool.get("Class") for tool in view.get("Tools") or []}
    missing_tools = REQUIRED_TUNE_TOOLS - tools
    if missing_tools:
        raise CheckError(f"tune.rviz 缺少工具：{sorted(missing_tools)}")

    saved = (view.get("Views") or {}).get("Saved") or []
    saved_names = {entry.get("Name") for entry in saved if isinstance(entry, dict)}
    if {"TopDownOrtho", "Orbit"} - saved_names:
        raise CheckError(f"tune.rviz 需要 TopDownOrtho 与 Orbit 两套视图，实际 {sorted(saved_names)}")


def check_gui_config():
    if not GUI_CONFIG.is_file():
        raise CheckError(f"缺少 gazebo GUI 配置：{GUI_CONFIG}")
    try:
        root = ET.parse(GUI_CONFIG).getroot()
    except ET.ParseError as exc:
        raise CheckError(f"gui.config 不是合法 XML：{exc}") from None
    if root.tag != "gui":
        raise CheckError(f"gui.config 根节点应为 <gui>，实际 <{root.tag}>")

    plugins = {plugin.get("filename") for plugin in root.iter("plugin")}
    if None in plugins:
        raise CheckError("gui.config 有 <plugin> 缺 filename 属性")
    unknown = plugins - KNOWN_GZ_GUI_PLUGINS
    if unknown:
        raise CheckError(
            f"gui.config 用了白名单外的插件（类名写错时 gz 会静默丢掉面板）：{sorted(unknown)}；"
            f"容器内用 `ign gui -l` 核对后把确认可用的名字加进 KNOWN_GZ_GUI_PLUGINS"
        )
    if {"MinimalScene", "GzSceneManager"} - plugins:
        raise CheckError(f"gui.config 丢了 3D 视图插件，实际 {sorted(plugins)}")


CHECKS = (
    ("sim.rviz 仍是验收最小集", check_sim_rviz),
    ("tune.rviz 结构与话题白名单", check_tune_rviz),
    ("gui.config 结构与插件白名单", check_gui_config),
)


def main():
    failures = 0
    for title, check in CHECKS:
        try:
            check()
        except CheckError as exc:
            failures += 1
            print(f"FAIL {title}: {exc}")
        else:
            print(f"PASS {title}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
