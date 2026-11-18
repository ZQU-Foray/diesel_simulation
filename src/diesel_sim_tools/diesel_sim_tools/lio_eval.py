# Copyright 2026 ZQU-Foray
#
# 里程计漂移的评估口径（decision.md D31）。纯标准库、不 import rclpy，
# 所以宿主没有 ROS 也能跑单测（与 profile.py / bridge.py 同一约定）。
#
# 为什么需要单独一份口径：point_lio 的 /aft_mapped_to_init 有两个特性，
# 直接拿「最新一帧里程计」减「最新一帧真值」会量出假漂移——
#   1) 消息戳取 lidar_end_time（laserMapping.cpp:274；开了
#      publish_odometry_without_downsample 时取 time_current），比 /clock 落后
#      0.1–0.3 s（处理耗时）。0.5 m/s 下 0.2 s 延迟就是 0.10 m，而真实漂移只有 0.03 m：
#      延迟被当成漂移，量出来的数比被测对象大一个数量级。
#      所以真值必须**插值到里程计自己的 stamp**。
#   2) camera_init 系原点在初始机位、朝向由重力对齐决定，与 map 系不同原点。
#      绝对坐标不可直接比，只能比**位移**：用首帧做一次刚体对齐（平移 + 绕 z 旋转）。
#
# 这两步都不修正被测对象，只让量具不再自己制造误差。

import bisect
import math


def yaw_from_quaternion(x, y, z, w):
    """四元数取绕 z 的偏航（rad）。输入是分量，避免依赖任何 ROS 消息类型。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a, b):
    """a-b 归一到 (-pi, pi]，跨 ±pi 时不产生 2pi 的假误差。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TruthTrack:
    """按时间递增的真值轨迹，支持按任意 stamp 线性插值。

    只接受严格递增的时间戳：gz 的 pose/info 偶尔重复同一时刻，
    重复戳会让插值分母为 0，直接丢掉而不是特殊处理。
    """

    def __init__(self):
        self._t = []
        self._v = []

    def __len__(self):
        return len(self._t)

    def push(self, stamp, x, y, yaw):
        if self._t and stamp <= self._t[-1]:
            return False
        self._t.append(stamp)
        self._v.append((x, y, yaw))
        return True

    def at(self, stamp):
        """返回 (x, y, yaw) 或 None。落在采样区间之外一律 None——
        外推会把「还没采到」伪装成「量到了」，宁可少一个样本。"""
        n = len(self._t)
        if n < 2:
            return None
        i = bisect.bisect_left(self._t, stamp)
        if i == 0 or i >= n:
            return None
        t0, t1 = self._t[i - 1], self._t[i]
        span = t1 - t0
        if span <= 0.0:
            return None
        w = (stamp - t0) / span
        x0, y0, yaw0 = self._v[i - 1]
        x1, y1, yaw1 = self._v[i]
        return (
            x0 + w * (x1 - x0),
            y0 + w * (y1 - y0),
            yaw0 + w * angle_diff(yaw1, yaw0),
        )


class DriftEvaluator:
    """首帧刚体对齐 + 逐帧位移误差。

    odom / truth 都是 (x, y, yaw)。第一对样本确立对齐，之后只累加误差，
    不再回头改对齐——否则慢漂移会被对齐吸收掉，量具就失效了。
    """

    def __init__(self):
        self._align = None
        self.rows = []

    @property
    def aligned(self):
        return self._align is not None

    def add(self, phase, stamp, odom, truth, latency=None):
        """喂一对**同一时刻**的 odom/truth，返回该帧的误差行。"""
        if self._align is None:
            dx = angle_diff(truth[2], odom[2])
            self._align = (
                math.cos(dx),
                math.sin(dx),
                odom[0],
                odom[1],
                truth[0],
                truth[1],
            )
        c, s, ox, oy, tx, ty = self._align
        dx, dy = odom[0] - ox, odom[1] - oy
        px = tx + c * dx - s * dy
        py = ty + s * dx + c * dy
        row = {
            "phase": phase,
            "stamp": stamp,
            "err": math.hypot(px - truth[0], py - truth[1]),
            "yerr_deg": math.degrees(
                abs(angle_diff(odom[2] + math.atan2(s, c), truth[2]))
            ),
            "truth": (truth[0], truth[1]),
            "odom": (odom[0], odom[1]),
        }
        if latency is not None:
            row["latency"] = latency
        self.rows.append(row)
        return row

    def summary(self, phases=("settle", "move", "stop", "observe")):
        """按阶段汇总。空阶段返回 None 而不是 0——「没量到」不等于「没误差」。"""
        out = {"samples": len(self.rows), "phases": {}}
        path = 0.0
        for a, b in zip(self.rows, self.rows[1:]):
            path += math.hypot(
                b["truth"][0] - a["truth"][0], b["truth"][1] - a["truth"][1]
            )
        out["truth_path_m"] = path
        for phase in phases:
            sub = [r for r in self.rows if r["phase"] == phase]
            if not sub:
                out["phases"][phase] = None
                continue
            item = {
                "n": len(sub),
                "max_xy_err": max(r["err"] for r in sub),
                "final_xy_err": sub[-1]["err"],
                "max_yaw_err_deg": max(r["yerr_deg"] for r in sub),
                "span_s": sub[-1]["stamp"] - sub[0]["stamp"],
            }
            lat = [r["latency"] for r in sub if "latency" in r]
            if lat:
                item["max_latency_s"] = max(lat)
            out["phases"][phase] = item
        return out
