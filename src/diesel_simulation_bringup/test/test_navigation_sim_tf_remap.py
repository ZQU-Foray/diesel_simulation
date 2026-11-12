#!/usr/bin/env python3
"""M12 回归：Nav2 进 namespace 后，命名空间里必须有 TF 中继。

为什么用静态断言：症状是「Nav2 起得来、节点全在、参数也对，但代价地图每秒刷
`Timed out waiting for transform from base_footprint to odom … Invalid frame ID "odom"`
`bt_navigator` 永远 `inactive`，整链 `accept.py ready` 超时」——不报错，只刷 INFO。
起一次仿真要两分钟且需要 gazebo，`colcon test` 不该为此付出这个代价。

实测根因（2026-11-12，`ros2 topic list` / `topic info`，t≈40 s）：

    /diesel/sentry/tf         Publisher count: 0   Subscription count: 5
    /tf                       Publisher count: 4   Subscription count: 3

Nav2 各节点的 tf2 监听器按相对名订阅 `tf`/`tf_static`，`PushRosNamespace` 之后落到
`/diesel/sentry/tf`；TF 发布者（`point_lio`、`sensor_scan_generation`、
`diesel_localization`）都在根 `/tf`，于是监听器永远收不到帧（decision.md D28）。

修法不是给 include 挂 `SetRemap(src="tf", dst="/tf")`——实测**不生效**：`launch` 的
remap 不随 `IncludeLaunchDescription` 传播进被 include 的描述，子进程命令行里没有
`-r tf:=/tf`。改用 `tf_relay` 节点把根 `/tf`、`/tf_static` 中继进命名空间。

断言四件事：
  1. Nav2 那个 `GroupAction` 里有 `PushRosNamespace`（namespace 真的加了）；
  2. 同一组里引用了 `tf_relay`（中继随 `start_nav2` 一起起）；
  3. `tf_relay` 节点的 `target_namespace` 引用 `robot_namespace` 这个 launch 参数；
  4. 没有 `SetRemap` 残留那种「看着对但不生效」的写法，避免误导后人。

用法：
    python3 test/test_navigation_sim_tf_remap.py
"""

from __future__ import annotations

import ast
import pathlib
import sys

LAUNCH = (
    pathlib.Path(__file__).resolve().parents[1] / "launch" / "navigation_sim.launch.py"
)

GLOBAL_TF_TOPICS = ("tf", "tf_static")
RELAY_EXECUTABLE = "tf_relay.py"


def _call_name(call: ast.Call) -> str:
    """`PushRosNamespace(...)` / `launch_ros.actions.SetRemap(...)` 都归一到末段名。"""
    func = call.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _keyword_const(call: ast.Call, name: str) -> str | None:
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


def _node_assignments(tree: ast.Module) -> dict[str, ast.Call]:
    """`name = Node(...)`（含 `generate_launch_description()` 内部的局部赋值）。"""
    found: dict[str, ast.Call] = {}

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            if _call_name(node.value) != "Node":
                continue
            for target in node.targets:
                if isinstance(target, ast.Name):
                    found[target.id] = node.value

    return found


def main() -> int:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    nodes = _node_assignments(tree)

    groups = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and _call_name(node) == "GroupAction"
    ]
    if not groups:
        print("FAIL：navigation_sim.launch.py 里找不到 GroupAction（Nav2 组）")
        return 1

    problems: list[str] = []
    nav2_group_found = False
    relay_ref: str | None = None

    for group in groups:
        actions = None
        for keyword in group.keywords:
            if keyword.arg == "actions":
                actions = keyword.value
        if not isinstance(actions, ast.List):
            continue

        names = [
            _call_name(item) for item in actions.elts if isinstance(item, ast.Call)
        ]

        if "PushRosNamespace" not in names:
            continue  # 不是 Nav2 那一组

        nav2_group_found = True

        for item in actions.elts:
            if isinstance(item, ast.Call) and _call_name(item) == "SetRemap":
                problems.append(
                    "Nav2 组里还有 SetRemap —— launch 的 remap 不会传播进 nav2 的 include"
                    "（子进程命令行没有 -r tf:=/tf），这条不生效，改用 tf_relay"
                )
            if (
                isinstance(item, ast.Name)
                and item.id in nodes
                and _keyword_const(nodes[item.id], "executable") == RELAY_EXECUTABLE
            ):
                relay_ref = item.id

    if not nav2_group_found:
        problems.append("找不到带 PushRosNamespace 的 Nav2 GroupAction")
    elif relay_ref is None:
        problems.append(
            "Nav2 组里引用的节点里没有 tf_relay —— 命名空间内没有 /tf 发布者，"
            "代价地图会一直等 odom"
        )
    else:
        relay = nodes[relay_ref]
        text = ast.unparse(relay)
        if "target_namespace" not in text:
            problems.append("tf_relay 没有 target_namespace 参数，中继不知道往哪送")
        if "robot_namespace" not in text:
            problems.append(
                "tf_relay 的 target_namespace 没引用 robot_namespace launch 参数，"
                "换机器人时会中继到错的命名空间"
            )

    if problems:
        for problem in problems:
            print(f"FAIL：{problem}")
        return 1

    print(
        "PASS：Nav2 组（PushRosNamespace）里挂着 tf_relay，把 "
        + " / ".join(f"/{topic}" for topic in GLOBAL_TF_TOPICS)
        + " 中继进命名空间"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
