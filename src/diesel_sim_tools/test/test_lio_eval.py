#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""lio_eval 单测：插值边界、刚体对齐、延迟不算漂移、真漂移不被吸收。

重点守 D31 那类假漂移：point_lio 的里程计戳落后 /clock 0.1–0.3 s，
拿「最新真值」比「最新里程计」会把这段延迟量成 0.10 m 的误差，
而真实漂移只有 0.03 m。量具自己制造误差，比没有量具更危险。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from diesel_sim_tools import lio_eval


def _track(speed=0.5, span=10.5, hz=50.0, x0=1.0, y0=2.0, yaw=0.0):
    """匀速直行的真值轨迹。"""
    track = lio_eval.TruthTrack()
    n = int(span * hz) + 1
    for i in range(n):
        t = i / hz
        track.push(
            t, x0 + speed * t * math.cos(yaw), y0 + speed * t * math.sin(yaw), yaw
        )
    return track


def test_yaw_and_angle_helpers():
    assert abs(lio_eval.yaw_from_quaternion(0, 0, 0, 1)) < 1e-12
    half = math.sin(math.pi / 4)
    assert abs(lio_eval.yaw_from_quaternion(0, 0, half, half) - math.pi / 2) < 1e-9
    # 跨 ±pi 必须走短弧，否则量出 ~360° 的假偏航误差
    wrapped = lio_eval.angle_diff(-math.pi + 0.01, math.pi - 0.01)
    assert abs(wrapped - 0.02) < 1e-9, wrapped
    print("helpers: 偏航解算与跨 ±pi 短弧 OK")


def test_interp_boundary_is_loud():
    track = _track(span=2.0)
    assert track.at(-0.1) is None, "区间外必须 None，不能外推"
    assert track.at(2.5) is None, "区间外必须 None，不能外推"
    assert lio_eval.TruthTrack().at(0.0) is None, "样本不足必须 None"
    x, y, yaw = track.at(1.0)
    assert abs(x - 1.5) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw) < 1e-9, (x, y, yaw)
    # 重复戳直接丢弃：插值分母为 0 会炸，宁可少一个样本
    assert track.push(1.0, 9.0, 9.0, 0.0) is False
    assert track.push(0.5, 9.0, 9.0, 0.0) is False
    print("interp: 区间外返回 None、区间内线性插值、重复戳丢弃 OK")


def test_alignment_removes_frame_offset():
    """camera_init 与 map 不同原点不同朝向：完美运动必须量出 0。"""
    track = _track()
    yaw_offset = 0.7
    ox, oy = -3.0, 5.0
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    ev = lio_eval.DriftEvaluator()
    for i in range(1, 101):
        t = i * 0.1
        dx, dy = 0.5 * t, 0.0  # 在 odom 系里沿自己的 x 轴直行
        odom = (ox + c * dx - s * dy, oy + s * dx + c * dy, yaw_offset)
        ev.add("move", t, odom, track.at(t))
    summary = ev.summary()["phases"]["move"]
    assert summary["max_xy_err"] < 1e-9, summary
    assert summary["max_yaw_err_deg"] < 1e-6, summary
    print("align: 首帧刚体对齐消掉原点与朝向差 OK")


def test_latency_is_not_drift():
    """核心回归：0.2 s 处理延迟 + 0.5 m/s，不得量成 0.10 m 漂移。"""
    track = _track()
    ev = lio_eval.DriftEvaluator()
    naive = []
    for i in range(5, 101):
        stamp = i * 0.1
        odom = track.at(stamp)  # 里程计**没有**漂移，只是戳落后
        now_truth = track.at(stamp + 0.2)  # 错误口径：最新真值
        naive.append(math.hypot(now_truth[0] - odom[0], now_truth[1] - odom[1]))
        ev.add("move", stamp, odom, track.at(stamp), latency=0.2)
    summary = ev.summary()["phases"]["move"]
    assert summary["max_xy_err"] < 1e-9, summary
    assert abs(max(naive) - 0.1) < 1e-6, max(naive)  # 旧口径确实会量出 0.10 m
    assert summary["max_latency_s"] == 0.2, summary
    print("latency: 时间对齐后延迟归零（旧口径 0.10 m）OK")


def test_real_drift_is_measured():
    """真漂移必须被量出，且不被后续对齐吸收。"""
    track = _track()
    ev = lio_eval.DriftEvaluator()
    for i in range(5, 101):
        stamp = i * 0.1
        x, y, yaw = track.at(stamp)
        drift = 0.02 * (stamp - 0.5)  # 每米约 0.02 m 的横向漂移
        ev.add("move", stamp, (x, y + drift, yaw), track.at(stamp))
    summary = ev.summary()["phases"]["move"]
    assert abs(summary["max_xy_err"] - 0.19) < 1e-6, summary
    assert abs(summary["final_xy_err"] - 0.19) < 1e-6, summary
    assert summary["max_yaw_err_deg"] < 1e-6, summary
    print("drift: 真实漂移被量出且不被对齐吸收 OK")


def test_path_length_and_empty_phase():
    track = _track()
    ev = lio_eval.DriftEvaluator()
    for i in range(5, 101):
        stamp = i * 0.1
        pose = track.at(stamp)
        ev.add("move", stamp, pose, pose)
    out = ev.summary()
    assert abs(out["truth_path_m"] - 4.75) < 1e-6, out["truth_path_m"]
    assert out["samples"] == 96, out["samples"]
    # 「没量到」必须是 None，不能是 0——否则空跑会被读成完美
    assert out["phases"]["settle"] is None, out["phases"]
    assert out["phases"]["stop"] is None, out["phases"]
    print("summary: 真值路程、样本数、空阶段 None OK")


def main():
    test_yaw_and_angle_helpers()
    test_interp_boundary_is_loud()
    test_alignment_removes_frame_offset()
    test_latency_is_not_drift()
    test_real_drift_is_measured()
    test_path_length_and_empty_phase()
    print("PASS test_lio_eval")


if __name__ == "__main__":
    main()
