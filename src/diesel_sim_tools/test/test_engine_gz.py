#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""engine_gz 单测：世界参数、spawn 参数、资源路径、真值话题。

脚本式（与 field/test 一致）：`python3 test/test_engine_gz.py`，退出码即结论。
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diesel_sim_tools import (
    AssemblyError,
    engine_gz,
)


def _touch(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("<sdf/>\n")
    return path


def test_resource_paths():
    base = tempfile.mkdtemp(prefix="simgz_")
    models = os.path.join(base, "models")
    os.makedirs(models)
    paths = engine_gz.resource_paths(models, models, "", None)
    assert paths == [os.path.abspath(models)], paths
    print("resource_paths: 绝对化 + 去重 OK")


def test_world_args():
    world = _touch(os.path.join(tempfile.mkdtemp(prefix="simgz_"), "worlds/w.sdf"))
    headless = engine_gz.world_args(world, headless=True)
    joined = "".join(headless)
    assert world in joined, joined
    assert " -r -v 3" in joined, joined
    assert "-s --headless-rendering" in joined, joined
    assert "--gui-config" not in joined, joined

    gui = "".join(
        engine_gz.world_args(world, headless=False, gui_config="/tmp/gui.config")
    )
    assert "--gui-config /tmp/gui.config" in gui, gui
    assert "--headless-rendering" not in gui, gui
    assert "--gui-config" not in "".join(engine_gz.world_args(world, headless=False))

    try:
        engine_gz.world_args(os.path.join(world, "missing.sdf"))
    except AssemblyError as error:
        assert "世界 SDF 不存在" in str(error), error
    else:
        raise AssertionError("世界文件缺失时必须抛 AssemblyError")
    print("world_args: 无界面/GUI/缺失世界 OK")


def test_spawn_args():
    args = engine_gz.spawn_args(
        "rmul_2027", "/assets/model.sdf", "robot_x", "6.0", "4.0", "0.0"
    )
    assert args == [
        "-world",
        "rmul_2027",
        "-file",
        "/assets/model.sdf",
        "-name",
        "robot_x",
        "-x",
        "6.0",
        "-y",
        "4.0",
        "-z",
        engine_gz.SPAWN_Z,
        "-Y",
        "0.0",
    ], args
    try:
        engine_gz.spawn_args("", "/a.sdf", "r", 0, 0, 0)
    except AssemblyError as error:
        assert "world" in str(error), error
    else:
        raise AssertionError("缺少世界名时必须抛 AssemblyError")
    print("spawn_args: 参数顺序 + 缺字段报错 OK")


def test_truth_topic():
    assert engine_gz.truth_topic("rmul_2027") == "/world/rmul_2027/pose/info"
    print("truth_topic: 跟随世界名 OK")


def main():
    test_resource_paths()
    test_world_args()
    test_spawn_args()
    test_truth_topic()
    print("PASS test_engine_gz")


if __name__ == "__main__":
    main()
