#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
#
# 调车 rqt 启动器的宿主级门禁（I13）：宿主没有 rqt 也能跑。
#
# 守三件事：
#   1) 脚本语法过得去 —— 调车时敲下 `rqt:=true` 才发现语法错是最糟的时机；
#   2) 面板清单没被删空 —— 六个面板是「信息够不够」的底线；
#   3) 一个面板都没有时必须非零退出并打印 apt 包名，而不是安静地开一片空白。

import os
import subprocess
import sys
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PKG_DIR / "scripts" / "tune_rqt.sh"
LAUNCH = PKG_DIR / "launch" / "navigation_sim.launch.py"

EXPECTED_PANELS = {
    "graph",
    "topic",
    "plot",
    "console",
    "tf_tree",
    "image_view",
}

# rqt 不存在时的最小 PATH：留下 bash 与 coreutils，去掉 ROS 与 pip 的 bin
BARE_PATH = "/usr/bin:/bin"


class CheckError(AssertionError):
    """断言失败。"""


def run(args, PATH=None):
    env = dict(os.environ)
    if PATH is not None:
        env["PATH"] = PATH
    return subprocess.run(
        args,
        capture_output=True,
        text=True,
        env=env,
        cwd=str(PKG_DIR),
        check=False,
    )


def check_syntax():
    if not SCRIPT.is_file():
        raise CheckError(f"缺少启动器：{SCRIPT}")
    result = run(["bash", "-n", str(SCRIPT)])
    if result.returncode != 0:
        raise CheckError(f"tune_rqt.sh 语法错误：{result.stderr.strip()}")


def check_dry_run():
    result = run(["bash", str(SCRIPT), "--dry-run"])
    if result.returncode != 0:
        raise CheckError(f"--dry-run 应始终返回 0，实际 {result.returncode}：{result.stderr.strip()}")
    panel_names = set()
    for line in result.stdout.splitlines():
        key, _, value = line.partition("=")
        if key != "panel":
            continue
        name, _, available = value.partition(" available=")
        if available not in {"true", "false"}:
            raise CheckError(f"--dry-run 行格式不对：{line!r}")
        panel_names.add(name.strip())
    if panel_names != EXPECTED_PANELS:
        raise CheckError(f"面板清单被改动了：期望 {sorted(EXPECTED_PANELS)}，实际 {sorted(panel_names)}")


def check_missing_behaviour():
    result = run(["bash", str(SCRIPT)], PATH=BARE_PATH)
    if result.returncode == 0:
        raise CheckError("一个 rqt 面板都没有时应当非零退出，实际返回 0")
    if "一个 rqt 面板都没找到" not in result.stderr:
        raise CheckError(f"缺少「全都没有」的明确提示，实际 stderr：{result.stderr.strip()!r}")
    # 缺包提示要能直接拿去 apt install
    for pkg in ("rqt-graph", "rqt-plot", "rqt-image-view"):
        if f"ros-humble-{pkg}" not in result.stderr:
            raise CheckError(f"缺包提示里没有 ros-humble-{pkg}：{result.stderr.strip()!r}")


def check_launch_wiring():
    text = LAUNCH.read_text(encoding="utf-8")
    if 'executable="tune_rqt.sh"' not in text:
        raise CheckError("navigation_sim.launch.py 没有起 tune_rqt.sh 的节点")
    if '"rqt"' not in text:
        raise CheckError("navigation_sim.launch.py 没有声明 rqt 参数")

    helper = (PKG_DIR / "scripts" / "sim_up.sh").read_text(encoding="utf-8")
    if "--rqt)" not in helper:
        raise CheckError("sim_up.sh 没有 --rqt 选项")
    if '"rqt:=$rqt"' not in helper:
        raise CheckError("sim_up.sh 没有把 rqt 透传给 launch")
    # --no-nav 会切到没有 rqt 参数的 localization_sim.launch.py，必须提前收掉
    if "rqt=false" not in helper:
        raise CheckError("sim_up.sh 缺少 --no-nav 时收掉 rqt 的分支")


CHECKS = (
    ("tune_rqt.sh 语法", check_syntax),
    ("--dry-run 面板清单", check_dry_run),
    ("缺 rqt 时非零退出并给包名", check_missing_behaviour),
    ("launch 已接上 rqt 参数", check_launch_wiring),
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
