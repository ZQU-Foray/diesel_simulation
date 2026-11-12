#!/usr/bin/env python3
"""L1 回归：仿真链必须给 point_lio 的 `body` 帧一条父级（`base_footprint → body`）。

为什么用静态断言、而不是起仿真断言：这条契约一丢，症状是「定位一帧也吃不到、状态冻在
`INITIALIZING`」，而 `colcon test` 不该为此拉起 gazebo。断言只读 launch 的语法结构，
能挡住四类回归：

  1. 整条 `base_footprint → body` 静态 TF 消失（L1 原样复发）；
  2. 方向被写成 `body → base_footprint`——tf2 单亲规则会拆树，实测日志
     `Could not find a connection between 'base_footprint' and 'body'`（decision.md D25）；
  3. `body_tf` 定义了却忘了放进 `LaunchDescription`（静默不启动，症状与 1 相同）；
  4. 另有一条静态 TF 以 `body` 为父（同一个 frame 两个父，同样断树）。

用法：
    python3 test/test_localization_sim_launch.py
"""

from __future__ import annotations

import ast
import pathlib
import sys

LAUNCH = pathlib.Path(__file__).resolve().parents[1] / "launch" / "localization_sim.launch.py"

BASE_FRAME = "base_footprint"
BODY_FRAME = "body"
TF_NODE_NAME = "body_static_tf"
TF_EXECUTABLE = "static_transform_publisher"


def _calls(tree, func_name):
    """按出现顺序收集 `func_name(...)` 调用。"""
    found = []

    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == func_name
        ):
            found.append(node)

    return found


def _keyword(call, name):
    """取关键字实参的语法节点，没有就 None。"""
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value

    return None


def _literal(call, name):
    """取字符串字面量关键字实参，非字面量返回 None。"""
    node = _keyword(call, name)

    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value

    return None


def _token(element):
    """把参数表里的一项还原成「(字面量值 | None, 源码文本)」。"""
    if isinstance(element, ast.Constant) and isinstance(element.value, str):
        return element.value, repr(element.value)

    return None, ast.unparse(element)


def _frames(call):
    """解析 static_transform_publisher 的 `--frame-id X --child-frame-id Y`。

    参数写成表达式（比如 `LIDAR_POSE[0]`）时字面量是 None，这里只需要判断帧名，
    所以非字面量一律保留源码文本，供报错时指认。
    """
    arguments = _keyword(call, "arguments")

    if not isinstance(arguments, ast.List):
        return None

    tokens = [_token(element) for element in arguments.elts]
    frames = {}
    index = 0

    while index < len(tokens):
        flag = tokens[index][0]

        if flag in ("--frame-id", "--child-frame-id"):
            frames[flag] = tokens[index + 1] if index + 1 < len(tokens) else (None, "<缺失>")
            index += 2
        else:
            index += 1

    return frames


def _in_launch_description(tree, variable):
    """断言变量被放进了 `LaunchDescription([...])` 的列表里。"""
    for call in _calls(tree, "LaunchDescription"):
        for argument in call.args:
            if not isinstance(argument, ast.List):
                continue

            for element in argument.elts:
                if isinstance(element, ast.Name) and element.id == variable:
                    return True

    return False


def check(launch_path):
    """返回问题列表；空表示通过。"""
    tree = ast.parse(launch_path.read_text(encoding="utf-8"), filename=str(launch_path))
    problems = []

    static_tfs = [call for call in _calls(tree, "Node") if _literal(call, "executable") == TF_EXECUTABLE]
    body_nodes = [call for call in static_tfs if _literal(call, "name") == TF_NODE_NAME]

    if not body_nodes:
        problems.append(
            f"launch 里没有名为 {TF_NODE_NAME} 的 static_transform_publisher："
            f"point_lio 的 {BODY_FRAME} 帧会没有父级，定位收不到点云（L1）"
        )
    elif len(body_nodes) > 1:
        problems.append(f"{TF_NODE_NAME} 出现了 {len(body_nodes)} 次，只该有一条")
    else:
        node = body_nodes[0]

        if _literal(node, "package") != "tf2_ros":
            problems.append(f"{TF_NODE_NAME} 的 package 必须是 tf2_ros")

        frames = _frames(node) or {}
        parent = frames.get("--frame-id", (None, "<缺失>"))
        child = frames.get("--child-frame-id", (None, "<缺失>"))

        if parent[0] is not None and parent[0] != BASE_FRAME:
            problems.append(f"{TF_NODE_NAME} 的父帧是 {parent[1]}，必须是 {BASE_FRAME!r}（方向不可反，D25）")

        if child[0] is not None and child[0] != BODY_FRAME:
            problems.append(f"{TF_NODE_NAME} 的子帧是 {child[1]}，必须是 {BODY_FRAME!r}（point_lio 硬编码，D56）")

        if not _in_launch_description(tree, "body_tf"):
            problems.append("body_tf 定义了但没有放进 LaunchDescription，起链时不会启动")

    for call in static_tfs:
        if _literal(call, "name") == TF_NODE_NAME:
            continue

        frames = _frames(call) or {}

        if frames.get("--frame-id", (None, ""))[0] == BODY_FRAME:
            problems.append(
                f"{_literal(call, 'name') or '<未命名>'} 也把 {BODY_FRAME} 当父帧："
                "同一个 frame 两个父会拆成两棵互不相连的树（tf2 单亲规则）"
            )

    return problems


def main():
    """跑断言并按仓库习惯报告。"""
    problems = check(LAUNCH)

    for problem in problems:
        print(f"FAIL {problem}")

    if problems:
        return 1

    print(f"PASS {LAUNCH.name}：{TF_NODE_NAME} 发布 {BASE_FRAME} -> {BODY_FRAME}，"
          f"{len(_calls(ast.parse(LAUNCH.read_text(encoding='utf-8')), 'Node'))} 个 Node 中无第二条以 "
          f"{BODY_FRAME} 为父的静态 TF")

    return 0


if __name__ == "__main__":
    sys.exit(main())
