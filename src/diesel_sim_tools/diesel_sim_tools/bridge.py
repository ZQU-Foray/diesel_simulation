# Copyright 2026 ZQU-Foray
"""桥接契约：从 `config/bridge.yaml` 生成 `ros_gz_bridge` 的参数与重映射。

契约里的 gz 话题是**模板**，用 `{world}` / `{robot}` / `{lidar_link}` 等占位符引用
profile 的值（decision.md D28 第 2 条）。渲染后若还残留 `{...}`，说明占位符名字写错
或 profile 缺字段——那正是「桥建起来了、日志不报错、数据全断」的成因（D20），
所以必须在启动前失败，并指出是哪个条目。

方向符号沿用既有写法：`[` = Gazebo→ROS，`]` = ROS→Gazebo。话题名在桥里统一用
gz 侧名字，ROS 侧靠 remappings 改名，不依赖 ros_gz_bridge 的双侧命名语法
（不同小版本行为不一致）。
"""

import os
import re

import yaml

from .errors import AssemblyError

# 方向 → parameter_bridge 的分隔符。
_DIRECTION_MARKS = {"gz2ros": "[", "ros2gz": "]"}

# 契约条目的合法字段。多写字段（例如把 expect 拼成 expects）会让那条断言静默失效，
# 所以未知字段一律报错而不是忽略。
_ENTRY_KEYS = {
    "id",
    "gz_topic",
    "ros_topic",
    "ros_type",
    "gz_type",
    "direction",
    "expect",
}
_EXPECT_KEYS = {"required"}

_PLACEHOLDER_RE = re.compile(r"\{([^{}]+)\}")


class BridgeEntry:
    """一条桥接契约。

    `required` 决定 `accept.py ready` 的契约自检是否把它算作失败项（README §1 A9）：
    缺一条必需桥接件就是「整链不完整」，可选条目只提示不判失败。
    """

    __slots__ = (
        "direction",
        "gz_topic",
        "gz_type",
        "id",
        "required",
        "ros_topic",
        "ros_type",
    )

    def __init__(
        self,
        entry_id,
        gz_topic,
        ros_type,
        gz_type,
        direction,
        ros_topic=None,
        required=True,
    ):
        self.id = entry_id
        self.gz_topic = gz_topic
        self.ros_topic = ros_topic or gz_topic
        self.ros_type = ros_type
        self.gz_type = gz_type
        self.direction = direction
        self.required = required

    @property
    def mark(self):
        """parameter_bridge 的方向分隔符。"""
        return _DIRECTION_MARKS[self.direction]

    def __repr__(self):
        return f"BridgeEntry(id={self.id!r}, direction={self.direction!r})"


def placeholders(template):
    """模板里出现的占位符名字（按出现顺序、去重）。"""
    names = []
    for name in _PLACEHOLDER_RE.findall(template):
        if name not in names:
            names.append(name)
    return names


def unresolved(text):
    """渲染后仍未替换的占位符——非空即契约或 profile 有问题。"""
    return _PLACEHOLDER_RE.findall(text)


def render(template, context, *, entry_id="?"):
    """用 profile 的值展开模板；缺字段时报出条目与字段名。"""
    missing = [name for name in placeholders(template) if name not in context]
    if missing:
        raise AssemblyError(
            f"桥接条目 `{entry_id}` 的模板 {template!r} 缺少 profile 字段："
            f"{', '.join(sorted(missing))}；可用字段：{', '.join(sorted(context))}"
        )
    return template.format(**context)


def load(path):
    """读契约文件并校验；任何结构问题都抛 AssemblyError。"""
    with open(path, encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, list):
        raise AssemblyError(f"桥接契约必须是条目列表：{path}")

    entries = []
    seen = set()
    for index, item in enumerate(raw):
        where = f"第 {index + 1} 条"
        if not isinstance(item, dict):
            raise AssemblyError(f"桥接契约{where}不是映射：{item!r}")
        entry_id = item.get("id")
        if not entry_id:
            raise AssemblyError(f"桥接契约{where}缺少 id")
        unknown = set(item) - _ENTRY_KEYS
        if unknown:
            raise AssemblyError(
                f"桥接条目 `{entry_id}` 有未知字段：{', '.join(sorted(unknown))}"
            )
        if entry_id in seen:
            raise AssemblyError(f"桥接条目 id 重复：`{entry_id}`")
        seen.add(entry_id)
        for key in ("gz_topic", "ros_type", "gz_type"):
            if not item.get(key):
                raise AssemblyError(f"桥接条目 `{entry_id}` 缺少 {key}")
        direction = item.get("direction", "gz2ros")
        if direction not in _DIRECTION_MARKS:
            raise AssemblyError(
                f"桥接条目 `{entry_id}` 的 direction 只能是 gz2ros/ros2gz：{direction}"
            )
        expect = item.get("expect") or {}
        if not isinstance(expect, dict):
            raise AssemblyError(f"桥接条目 `{entry_id}` 的 expect 必须是映射")
        unknown = set(expect) - _EXPECT_KEYS
        if unknown:
            raise AssemblyError(
                f"桥接条目 `{entry_id}` 的 expect 有未知字段：{', '.join(sorted(unknown))}"
            )
        entries.append(
            BridgeEntry(
                entry_id=entry_id,
                gz_topic=item["gz_topic"],
                ros_type=item["ros_type"],
                gz_type=item["gz_type"],
                direction=direction,
                ros_topic=item.get("ros_topic"),
                required=bool(expect.get("required", True)),
            )
        )
    return entries


def render_entries(entries, context):
    """展开所有条目，返回 (`parameter_bridge` 参数, remappings)。"""
    arguments = []
    remappings = []
    for entry in entries:
        gz_topic = render(entry.gz_topic, context, entry_id=entry.id)
        ros_topic = render(entry.ros_topic, context, entry_id=entry.id)
        leftover = unresolved(gz_topic) + unresolved(ros_topic)
        if leftover:
            raise AssemblyError(
                f"桥接条目 `{entry.id}` 渲染后仍有未替换占位符：{', '.join(leftover)}"
            )
        arguments.append(f"{gz_topic}@{entry.ros_type}{entry.mark}{entry.gz_type}")
        if ros_topic != gz_topic:
            remappings.append((gz_topic, ros_topic))
    return arguments, remappings


def ros_endpoints(entries, context):
    """契约里每条桥在 ROS 侧的那个话题：`[(entry_id, topic, direction, required)]`。

    给验收自检用（README §1 A9）：`gz2ros` 条目要求 ROS 侧**有发布者**（数据从仿真出来），
    `ros2gz` 条目要求 ROS 侧**有订阅者**（指令进了仿真）。话题名与启动时渲染用的是同一套
    规则，所以这里不会和 parameter_bridge 看到的对不上。
    """
    endpoints = []

    for entry in entries:
        # 显式 ros_topic 也可能带占位符（例如按机器人名分话题），统一渲染一次
        topic = render(entry.ros_topic or entry.gz_topic, context, entry_id=entry.id)
        endpoints.append((entry.id, topic, entry.direction, entry.required))

    return endpoints


def default_path(share_dir):
    """本仓约定的契约位置：`<bringup share>/config/bridge.yaml`。"""
    return os.path.join(share_dir, "config", "bridge.yaml")
