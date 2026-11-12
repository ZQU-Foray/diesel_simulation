# Copyright 2026 ZQU-Foray
"""gz-sim（Ignition Fortress）启动参数的唯一出口。

装配层只描述「用哪个世界、哪台机器人、摆在哪」，把它们翻译成某个具体引擎的
命令行是这一层的事。将来真要接第二个引擎，替换本文件即可，profile 与 launch
不动（decision.md D28 第 4 条：现在只有 Fortress 一个引擎，不做多引擎插件化）。
"""

import os

from .errors import AssemblyError

# 服务器的固定开关：`-r` 立刻开跑（不必等 GUI 按开始），`-v 3` 是 Fortress 的
# 默认日志级别——info 级启动日志是 A0「无报错加载」的取证依据，降级会让取证失效。
WORLD_FLAGS = " -r -v 3"

# 无界面时追加的两项：`-s` 只跑服务器；`--headless-rendering` 保留离屏渲染，
# 传感器才有数据（README §1 A1 的 CI 口径就是软件渲染下的实测频率）。
HEADLESS_FLAGS = " -s --headless-rendering"

# 出生高度：放在地面之上 0.02 m，让 gazebo 先解算一次接触再沉降。取 0 会初始穿模，
# 起机瞬间的冲击会污染 IMU 重力对齐（D12）。
SPAWN_Z = "0.02"


def resource_paths(*directories):
    """把资产目录转成引擎资源搜索路径（绝对化、保序、去重）。

    世界 SDF 里的 `model://rmul_2027` 只能靠这条路径解析。直接 `ign gazebo` 起世界
    而不设它，会得到 `Error Code 13: Unable to find uri[model://rmul_2027]`。
    """
    paths = []
    for directory in directories:
        if not directory:
            continue
        absolute = os.path.abspath(directory)
        if absolute not in paths:
            paths.append(absolute)
    return paths


def check_world(world_sdf):
    """世界文件必须存在：路径拼错时 gz 只会起一个空世界，比启动失败更难查。"""
    if not world_sdf or not os.path.isfile(world_sdf):
        raise AssemblyError(f"世界 SDF 不存在：{world_sdf!r}")
    return world_sdf


def world_args(world_sdf, *, headless=True, gui_config=None):
    """`gz_args` 的取值：世界文件 + 固定开关 (+ 无界面 / GUI 配置)。

    返回列表而不是整串，是为了让 launch 按需拼接：launch 会把列表元素直接接起来，
    所以每个元素自带前导空格。
    """
    args = [check_world(world_sdf), WORLD_FLAGS]
    if headless:
        args.append(HEADLESS_FLAGS)
    elif gui_config:
        args.append(f" --gui-config {gui_config}")
    return args


def spawn_args(world, sdf, model_name, x, y, yaw, *, z=SPAWN_Z):
    """`ros_gz_sim create` 的参数。

    `x`/`y`/`yaw` 允许传 LaunchConfiguration 那样的替换对象——这里只负责顺序与
    命名，不对它们求值，求值由 launch 在启动时完成。
    """
    for label, value in (("world", world), ("sdf", sdf), ("model_name", model_name)):
        if not value:
            raise AssemblyError(f"spawn 参数缺少 {label}")
    return [
        "-world",
        world,
        "-file",
        sdf,
        "-name",
        model_name,
        "-x",
        x,
        "-y",
        y,
        "-z",
        z,
        "-Y",
        yaw,
    ]


def truth_topic(world):
    """仿真真值话题（gz `pose/info`）在 gz 侧的完整名字。

    验收脚本要靠它算位置误差，所以它跟着世界名走、不能写死；桥接契约里同名条目的
    `gz_topic` 写成 `/world/{world}/pose/info`，与这里同一个来源。
    """
    if not world:
        raise AssemblyError("truth_topic 需要世界名")
    return f"/world/{world}/pose/info"
