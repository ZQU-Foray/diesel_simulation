"""pytest 驱动 livox_pattern 纯核心单测。

纯核心（pattern.cpp）无 ROS/gz 依赖，这里直接用 g++ 把它和 pattern_test.cpp
编成临时二进制再跑，断言退出码 0。这样 dev_task verify 只认 pytest 也能覆盖 C++ 数学。
"""
import pathlib
import subprocess
import tempfile

import pytest

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent
SRC = PKG / "src" / "pattern.cpp"
TEST_SRC = HERE / "pattern_test.cpp"
INCLUDE = PKG / "include"
FIXTURE = HERE / "fixtures" / "mini_pattern.csv"


@pytest.fixture(scope="module")
def pattern_binary(tmp_path_factory):
    out = tmp_path_factory.mktemp("livox_pattern") / "pattern_test"
    cmd = [
        "g++", "-std=c++17", "-O2", "-Wall", "-Wextra",
        "-I", str(INCLUDE),
        str(SRC), str(TEST_SRC),
        "-o", str(out),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode == 0, f"编译失败:\n{proc.stderr}"
    return out


def test_pattern_core(pattern_binary):
    proc = subprocess.run(
        [str(pattern_binary), str(FIXTURE)], capture_output=True, text=True
    )
    assert proc.returncode == 0, f"单测失败:\n{proc.stdout}\n{proc.stderr}"
    assert "0 failures" in proc.stdout, proc.stdout


def test_fixture_exists():
    assert FIXTURE.is_file(), "缺少 mini_pattern.csv 测试夹具"
    lines = FIXTURE.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0] == "Time/s,Azimuth/deg,Zenith/deg"
    assert len(lines) == 17  # 表头 + 16 行
