#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
#
# tune_report.py 的宿主级门禁（I15）：不需要 ROS，只测纯函数与输出契约。
#
# 为什么能在宿主跑：脚本顶层不 import rclpy（rclpy 只在 main() 里导入），
# 所以这里可以按文件路径加载模块、喂合成时间戳。
#
# 守四件事：
#   1) 10 Hz 的合成序列必须算出 ~10 Hz，且判 ok；
#   2) 恰好 2.0 s 的间隔必须判 stale（边界值按「超时即失效」处理，D13 口径）；
#   3) 一帧都没有必须判 missing，且 hz 是 None 而不是 0（不补零）；
#   4) --json 的顶层键集合固定，供脚本化消费。

import importlib.util
import json
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "tune_report.py"

EXPECTED_JSON_KEYS = {
    "generated_at",
    "window_s",
    "topics",
    "tf",
    "localization",
    "cmd_vs_truth",
}


class CheckError(AssertionError):
    """断言失败。"""


def load_module():
    spec = importlib.util.spec_from_file_location("tune_report", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_hz(module):
    stamps = [i / 10.0 for i in range(11)]  # 10 Hz，1 s 内 11 帧
    summary = module.summarize_hz(stamps)
    if summary["count"] != 11:
        raise CheckError(f"帧数应为 11，实际 {summary['count']}")
    if summary["hz"] is None or abs(summary["hz"] - 10.0) > 0.01:
        raise CheckError(f"10 Hz 序列应算出 10.0 Hz，实际 {summary['hz']}")
    status = module.topic_status(summary, module.LIDAR_TIMEOUT_SEC)
    if status != "ok":
        raise CheckError(f"10 Hz 且间隔 0.1 s 应判 ok，实际 {status}")


def check_stale_boundary(module):
    # 间隔恰好等于阈值：D13 的判据是「超过 N 秒没新帧算失联」，边界按失效处理
    summary = module.summarize_hz([0.0, 2.0])
    status = module.topic_status(summary, module.ODOM_TIMEOUT_SEC)
    if status != "stale":
        raise CheckError(
            f"恰 {module.ODOM_TIMEOUT_SEC} s 的间隔应判 stale，实际 {status}"
        )


def check_missing(module):
    summary = module.summarize_hz([])
    if summary["hz"] is not None:
        raise CheckError(f"空序列的 hz 应为 None（不补零），实际 {summary['hz']}")
    if module.topic_status(summary, module.LIDAR_TIMEOUT_SEC) != "missing":
        raise CheckError("空序列应判 missing")


def check_json_contract(module):
    topics = {
        name: dict(module.summarize_hz([0.0, 0.1]), status="ok")
        for name in module.REQUIRED_TOPICS
    }
    report = module.build_report(
        topics=topics,
        tf={"chain_ok": True, "detail": "map->odom"},
        localization={"state": "OK", "pose_valid": True, "age_s": 0.1},
        cmd_vs_truth=module.summarize_cmd_vs_truth(
            0.5, [(0.0, 6.0, 4.0), (1.0, 6.5, 4.0)]
        ),
        window_s=1.0,
    )
    keys = set(json.loads(json.dumps(report)))
    if keys != EXPECTED_JSON_KEYS:
        raise CheckError(
            f"--json 顶层键被改动了：期望 {sorted(EXPECTED_JSON_KEYS)}，实际 {sorted(keys)}"
        )
    if report["cmd_vs_truth"]["truth_speed_mps"] is None:
        raise CheckError("真值位移 0.5 m/1 s 应算出 0.5 m/s，实际 None")
    if abs(report["cmd_vs_truth"]["truth_speed_mps"] - 0.5) > 1e-6:
        raise CheckError(
            f"真值速度应为 0.5 m/s，实际 {report['cmd_vs_truth']['truth_speed_mps']}"
        )
    if module.overall_ok(report):
        raise CheckError(f"全部正常时不该报红，实际 {module.overall_ok(report)}")


CHECKS = (
    ("10 Hz 合成序列", check_hz),
    ("2.0 s 边界判 stale", check_stale_boundary),
    ("空序列判 missing 且不补零", check_missing),
    ("--json 顶层键契约", check_json_contract),
)


def main():
    module = load_module()
    failures = 0
    for title, check in CHECKS:
        try:
            check(module)
        except CheckError as exc:
            failures += 1
            print(f"FAIL {title}: {exc}")
        else:
            print(f"PASS {title}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
