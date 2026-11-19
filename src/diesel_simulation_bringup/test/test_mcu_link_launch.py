#!/usr/bin/env python3
"""A10 回归：`mcu_link:=true` 这条装配从整链入口一路透到仿真层。

为什么用静态断言：真跑一遍要多起一个固件进程和 PTY，`colcon test` 不该为此付出
两分钟；而这条链最容易坏的方式恰恰是「参数在某层被漏转发」——launch 不会报错，
只会静默退回直连 gz，验收时看到的仍是能动的车。

断言四件事：
  1. `simulation.launch.py` 声明了四个 mcu 参数，并用 OpaqueFunction 装配
     （虚拟 CDC + 可选固件进程），默认 `mcu_link=false`；
  2. `localization_sim.launch.py` 把四个参数转发进 simulation 的 include；
  3. `navigation_sim.launch.py` 把四个参数转发进 localization 的 include；
  4. `sim_up.sh` 有 `--mcu-link`/`--mcu-firmware`，后者顺带把 `mcu_link` 打开；
  5. `mcu_link:=true` 时 `cmd_vel` 这条桥接契约被让给过渡桥（独占约束），
     并且过渡桥节点确实被拉起——否则两个发布者抢同一个 gz 话题，或者干脆没人写。

用法：
    python3 test/test_mcu_link_launch.py
"""

from __future__ import annotations

import ast
import pathlib
import sys

LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"
SIM_UP = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sim_up.sh"

ARGS = ("mcu_link", "mcu_link_host_port", "mcu_link_port", "mcu_firmware")


def _parse(name: str) -> ast.Module:
    path = LAUNCH_DIR / name
    return ast.parse(path.read_text(encoding="utf-8"))


def _declared(tree: ast.Module) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
        if name != "DeclareLaunchArgument" or not node.args:
            continue
        arg = node.args[0]
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            names.add(arg.value)
    return names


def _forwarded(tree: ast.Module, source: str) -> set[str]:
    """`IncludeLaunchDescription(..., launch_arguments={...})` 里转发了哪些参数。"""
    found: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
        if name != "IncludeLaunchDescription":
            continue
        text = ast.unparse(node)
        if source not in text:
            continue
        for keyword in node.keywords:
            if keyword.arg != "launch_arguments":
                continue
            for key in ast.walk(keyword.value):
                if isinstance(key, ast.Constant) and key.value in ARGS:
                    found.add(key.value)
    return found


def _string_literals(node: ast.AST) -> set[str]:
    """一棵子树里出现过的全部字符串字面量。"""
    return {
        child.value
        for child in ast.walk(node)
        if isinstance(child, ast.Constant) and isinstance(child.value, str)
    }


def _yields_cmd_vel(text: str) -> bool:
    """`simulation.launch.py` 是否在 mcu_link 路径上让出了 `cmd_vel`。

    只认字面量会和实现形态耦合：最初写的是
    `skip_ids=("cmd_vel",) if mcu_link else ()`，后来 lidar_points 成了
    常驻跳过项，写法变成「先建列表、mcu_link 时再 append」，行为没变而
    字面量消失，断言因此假红（D32 合并时撞上）。所以改成看语义：

      1. `skip_ids=<值>` 的实参里出现 `"cmd_vel"`（元组字面量的老写法）；
      2. 或者名为 `skip_ids` 的容器在构造里含 `"cmd_vel"`（`[... "cmd_vel" ...]`）；
      3. 或者 `skip_ids.append("cmd_vel")` / `skip_ids.insert(...)` 这类原地追加。

    命中在不在 if 分支里不追究：断言的是「mcu_link 为真时这条桥会被跳过」
    这一意图能否从源码读出来，守卫 `mcu_link` 本身由上面那组参数用例负责。
    """
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            for keyword in node.keywords:
                if keyword.arg == "skip_ids" and "cmd_vel" in _string_literals(
                    keyword.value
                ):
                    return True
            # skip_ids.append("cmd_vel") / skip_ids.insert(0, "cmd_vel")
            owner = node.func.value if isinstance(node.func, ast.Attribute) else None
            if (
                isinstance(node.func, ast.Attribute)
                and isinstance(owner, ast.Name)
                and owner.id == "skip_ids"
                and "cmd_vel" in _string_literals(node)
            ):
                return True
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if (
                    isinstance(target, ast.Name)
                    and target.id == "skip_ids"
                    and "cmd_vel" in _string_literals(node.value)
                ):
                    return True
    return False


def main() -> int:
    problems: list[str] = []

    sim = _parse("simulation.launch.py")
    sim_declared = _declared(sim)
    missing = [arg for arg in ARGS if arg not in sim_declared]
    if missing:
        problems.append(
            "simulation.launch.py 没声明参数："
            + "、".join(missing)
            + "（装配层会静默退化）"
        )
    sim_text = LAUNCH_DIR.joinpath("simulation.launch.py").read_text(encoding="utf-8")
    if "virtual_cdc.py" not in sim_text:
        problems.append(
            "simulation.launch.py 里没有 virtual_cdc.py —— 虚拟 CDC 没被装配"
        )
    for node in ast.walk(sim):
        if isinstance(node, ast.Call):
            func = node.func
            name = (
                func.attr
                if isinstance(func, ast.Attribute)
                else getattr(func, "id", "")
            )
            if name == "DeclareLaunchArgument" and node.args:
                arg = node.args[0]
                if isinstance(arg, ast.Constant) and arg.value == "mcu_link":
                    for keyword in node.keywords:
                        if keyword.arg == "default_value":
                            value = keyword.value
                            if (
                                isinstance(value, ast.Constant)
                                and value.value != "false"
                            ):
                                problems.append(
                                    f"mcu_link 默认值必须是 false（当前 {value.value!r}），"
                                    "否则整链默认态就不是直连 gz 了"
                                )

    for name, source in (
        ("localization_sim.launch.py", "simulation.launch.py"),
        ("navigation_sim.launch.py", "localization_sim.launch.py"),
    ):
        tree = _parse(name)
        forwarded = _forwarded(tree, source)
        missing = [arg for arg in ARGS if arg not in forwarded]
        if missing:
            problems.append(f"{name} 没把参数转发进 {source}：" + "、".join(missing))
        declared = _declared(tree)
        missing = [arg for arg in ARGS if arg not in declared]
        if missing:
            problems.append(
                f"{name} 没声明参数："
                + "、".join(missing)
                + "（ros2 launch 传进来会直接报未知参数）"
            )

    if "diesel_sim_mcu" not in sim_text or "sim_mcu_bridge" not in sim_text:
        problems.append(
            "simulation.launch.py 没起 diesel_sim_mcu/sim_mcu_bridge —— "
            "mcu_link:=true 时没人把命令写进 gz"
        )
    if not _yields_cmd_vel(sim_text):
        problems.append(
            "simulation.launch.py 没在 mcu_link 时让出 cmd_vel 这条桥接契约，"
            "过渡桥与 ros_gz_bridge 会同时发布 gz 的 /cmd_vel"
        )

    script = SIM_UP.read_text(encoding="utf-8")
    if "--mcu-link" not in script:
        problems.append("sim_up.sh 没有 --mcu-link，整链入口用不到 A10")
    if "--mcu-firmware" not in script:
        problems.append("sim_up.sh 没有 --mcu-firmware，只能手工另起固件进程")
    if '"mcu_link:=true"' not in script:
        problems.append("sim_up.sh 没把 --mcu-link 翻成 mcu_link:=true")

    if problems:
        for problem in problems:
            print(f"FAIL：{problem}")
        return 1

    print(
        "PASS：mcu_link 从 sim_up.sh 经 navigation → localization → simulation 全层转发"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
