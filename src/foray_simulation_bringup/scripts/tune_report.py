#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
"""调车用话题/坐标链健康快照（I15）。

用法：
    ros2 run foray_simulation_bringup tune_report.py            # 人读表格
    ros2 run foray_simulation_bringup tune_report.py --json     # 机器读
    ros2 run foray_simulation_bringup tune_report.py --window 10 --deadline 20

为什么要有它：调车时「车不动」有十几种原因（速度没进物理引擎、点云断流、TF 断链、
定位 LOST），逐个 `ros2 topic hz` 敲一遍很慢，而且各条命令的采样窗口不一致、结论对不上。
这里用同一个窗口把六路话题、TF 链与「下发速度 vs 真值位移」一次采样，输出一张表。

设计约束：
- 顶层不 import rclpy：宿主没有 ROS 时也能 `import` 本模块跑单元测试（rclpy 只在
  `main()` 里导入）。纯函数（`summarize_hz` / `evaluate_health` / `build_report`）负责全部判断。
- 取不到的数据标 `missing`，绝不补零——把「没数据」显示成「0 Hz」会让人以为链路是通的。
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from itertools import pairwise

# 与 decision.md D13 的定位判据同源：超过这些秒数没有新帧，就按「失联」处理
ODOM_TIMEOUT_SEC = 2.0
LIDAR_TIMEOUT_SEC = 2.5
LOST_TIMEOUT_SEC = 5.0

# 各话题的失效阈值（秒）。前三条直接来自 D13；/cmd_vel 与 /sim/pose_info 是调试通道，
# 手拨遥控时可能长时间不发，因此只用来算「下发 vs 实际」，不参与 overall 判定。
STALE_SEC = {
    "/livox/lidar": LIDAR_TIMEOUT_SEC,
    "/cloud_registered": LIDAR_TIMEOUT_SEC,
    "/aft_mapped_to_init": ODOM_TIMEOUT_SEC,
    "/odometry": ODOM_TIMEOUT_SEC,
    "/localization/state": LOST_TIMEOUT_SEC,
    "/cmd_vel": ODOM_TIMEOUT_SEC,
    "/sim/pose_info": ODOM_TIMEOUT_SEC,
}

# 必须一直有数据的通道（/cmd_vel、/sim/pose_info 只做对照，缺了不算故障）
REQUIRED_TOPICS = (
    "/livox/lidar",
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/odometry",
    "/localization/state",
)

DEBUG_TOPICS = ("/cmd_vel", "/sim/pose_info")

TF_CHAIN = (
    ("map", "odom"),
    ("odom", "base_footprint"),
    ("base_footprint", "front_mid360"),
)

DEFAULT_WINDOW_SEC = 5.0
DEFAULT_DEADLINE_SEC = 15.0


def summarize_hz(stamps):
    """时间戳序列 → {count, hz, max_gap_s}。空序列返回 count=0、hz=None（不补零）。"""
    count = len(stamps)
    if count == 0:
        return {"count": 0, "hz": None, "max_gap_s": None}
    if count == 1:
        return {"count": 1, "hz": None, "max_gap_s": None}
    ordered = sorted(stamps)
    span = ordered[-1] - ordered[0]
    gaps = [b - a for a, b in pairwise(ordered)]
    return {
        "count": count,
        # 用「样本数 - 1」除以跨度：首尾各算一次，含 0 样本偏差更小
        "hz": (count - 1) / span if span > 0 else None,
        "max_gap_s": max(gaps) if gaps else None,
    }


def topic_status(summary, stale_sec):
    """状态三档：missing（一帧都没来）/ stale（间隔超阈值）/ ok。"""
    if summary["count"] == 0:
        return "missing"
    if summary["max_gap_s"] is None:
        # 只收到一帧：还没法算间隔，按「刚起来」处理，不判 stale
        return "warmup"
    if summary["max_gap_s"] >= stale_sec:
        return "stale"
    return "ok"


def evaluate_health(records):
    """{topic: [stamps]} → {topic: {count, hz, max_gap_s, status}}。"""
    topics = {}
    for name, stamps in records.items():
        summary = summarize_hz(stamps)
        summary["status"] = topic_status(summary, STALE_SEC.get(name, ODOM_TIMEOUT_SEC))
        topics[name] = summary
    return topics


def build_report(topics, tf, localization, cmd_vs_truth, window_s, now=None):
    """把三块结论拼成 --json 的顶层结构（键集合由 test_tune_report.py 锁住）。"""
    moment = now or datetime.now(timezone.utc)
    return {
        "generated_at": moment.isoformat(timespec="seconds"),
        "window_s": window_s,
        "topics": topics,
        "tf": tf,
        "localization": localization,
        "cmd_vs_truth": cmd_vs_truth,
    }


def overall_ok(report):
    """谁该判红：必需话题没有数据/失联，或 TF 链断，或定位 LOST。"""
    problems = []
    for name in REQUIRED_TOPICS:
        status = report["topics"].get(name, {}).get("status", "missing")
        if status in {"missing", "stale"}:
            problems.append(f"{name}={status}")
    if not report["tf"].get("chain_ok", False):
        problems.append("tf_chain_broken")
    state = report["localization"].get("state")
    if state is not None and str(state).upper() in {"LOST", "FAILED"}:
        problems.append(f"localization={state}")
    return problems


# LocalizationState 的状态常量（见 foray_localization_ros/msg/LocalizationState.msg）
LOCALIZATION_STATES = ("STATE_INITIALIZING", "STATE_OK", "STATE_DEGRADED", "STATE_LOST")


def localization_state_name(msg):
    """把 LocalizationState.state 的数字翻译成 OK/DEGRADED/LOST……。

    用消息类上的常量反查，不写死数字——枚举值变了这里不会静默错位。
    """
    if msg is None:
        return None
    value = getattr(msg, "state", None)
    for attr in LOCALIZATION_STATES:
        if getattr(type(msg), attr, None) == value:
            return attr.replace("STATE_", "")
    return str(value)


def summarize_localization(msg, now):
    """LocalizationState → {state, pose_valid, age_s}。取不到就是 None，不猜。"""
    if msg is None:
        return {"state": None, "pose_valid": None, "age_s": None}
    header = getattr(msg, "header", None)
    age = None
    if header is not None and getattr(header, "stamp", None) is not None:
        stamp = header.stamp
        age = now - (stamp.sec + stamp.nanosec * 1e-9)
    return {
        "state": localization_state_name(msg),
        "pose_valid": getattr(msg, "pose_valid", None),
        "age_s": age,
    }


MODEL_NAME = "foray_sim_robot"


def extract_truth_pose(msg, model_name=MODEL_NAME):
    """TFMessage（gz pose/info）→ 机器人真值 (x, y)，取不到返回 None。

    优先 chassis：那是 base_footprint 的原点；没有就退回模型根位姿。
    """
    transforms = getattr(msg, "transforms", None) or []
    by_child = {item.child_frame_id: item.transform for item in transforms}
    for name in (f"{model_name}/chassis", model_name):
        found = by_child.get(name)
        if found is not None:
            return (found.translation.x, found.translation.y)
    return None


def summarize_cmd_vs_truth(cmd_speed, truth_samples):
    """下发速度 vs 真值位移速度。truth_samples 是 [(t, x, y), ...]。"""
    truth_speed = None
    if len(truth_samples) >= 2:
        (t0, x0, y0), (t1, x1, y1) = truth_samples[0], truth_samples[-1]
        dt = t1 - t0
        if dt > 0:
            dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            truth_speed = dist / dt
    deviation = None
    if cmd_speed is not None and truth_speed is not None:
        deviation = truth_speed - cmd_speed
    return {
        "cmd_speed_mps": cmd_speed,
        "truth_speed_mps": truth_speed,
        "deviation_mps": deviation,
    }


def format_table(report):
    """人读输出：一行一个话题，末尾给 verdict。"""
    lines = [f"窗口 {report['window_s']:.1f} s @ {report['generated_at']}"]
    lines.append(f"{'话题':<24}{'帧数':>6}{'频率(Hz)':>10}{'最大间隔(s)':>12}  状态")
    for name, item in report["topics"].items():
        hz = "-" if item["hz"] is None else f"{item['hz']:.1f}"
        gap = "-" if item["max_gap_s"] is None else f"{item['max_gap_s']:.3f}"
        lines.append(f"{name:<24}{item['count']:>6}{hz:>10}{gap:>12}  {item['status']}")
    tf = report["tf"]
    lines.append(
        f"TF 链 {'完整' if tf.get('chain_ok') else '断裂'}：{tf.get('detail', '')}"
    )
    loc = report["localization"]
    age = "-" if loc.get("age_s") is None else f"{loc['age_s']:.1f}s"
    lines.append(
        f"定位：state={loc.get('state')} pose_valid={loc.get('pose_valid')} 距今={age}"
    )
    cmd = report["cmd_vs_truth"]
    lines.append(
        "速度对照：cmd={} truth={} 偏差={}".format(
            "-" if cmd["cmd_speed_mps"] is None else f"{cmd['cmd_speed_mps']:.2f}",
            "-" if cmd["truth_speed_mps"] is None else f"{cmd['truth_speed_mps']:.2f}",
            "-" if cmd["deviation_mps"] is None else f"{cmd['deviation_mps']:.2f}",
        )
    )
    problems = overall_ok(report)
    verdict = "PASS" if not problems else "FAIL " + ", ".join(problems)
    lines.append("verdict=" + verdict)
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="调车用话题/坐标链健康快照")
    parser.add_argument(
        "--window", type=float, default=DEFAULT_WINDOW_SEC, help="采样窗口（秒）"
    )
    parser.add_argument(
        "--deadline",
        type=float,
        default=DEFAULT_DEADLINE_SEC,
        help="最长等待（秒）；到点仍无数据就按 missing 出报告",
    )
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    return parser.parse_args(argv)


def collect(args):
    """在真实 ROS 图上采样；ROS 相关导入只在这里发生，宿主跑单测不会碰它。"""
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    watched = REQUIRED_TOPICS + DEBUG_TOPICS

    class Collector(Node):  # 内部一次性节点
        def __init__(self):
            super().__init__("tune_report")
            self.stamps = {name: [] for name in watched}
            self.cmd_speed = None
            self.truth = []
            self.localization = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            from rclpy.qos import QoSPresetProfiles

            for name in watched:
                # 传感器用 Best Effort，其余用默认 profile：用错 profile 会「订阅得上但收不到」
                sensor = name in {"/livox/lidar", "/cloud_registered"}
                qos = QoSPresetProfiles.SENSOR_DATA.value if sensor else 10
                self.create_subscription(
                    get_type(name), name, self._make_cb(name), qos, raw=False
                )

        def _make_cb(self, name):
            def callback(msg, _name=name):
                stamp = self.get_clock().now().nanoseconds * 1e-9
                self.stamps[_name].append(stamp)
                if _name == "/cmd_vel":
                    self.cmd_speed = msg.linear.x
                elif _name == "/sim/pose_info":
                    pose = extract_truth_pose(msg)
                    if pose is not None:
                        self.truth.append((stamp, pose[0], pose[1]))
                elif _name == "/localization/state":
                    self.localization = msg

            return callback

    return Collector


def get_type(name):
    """话题 → 消息类型（延迟导入，避免宿主没有 ROS 时 import 失败）。"""
    if name == "/livox/lidar":
        from sensor_msgs.msg import PointCloud2

        return PointCloud2
    if name == "/cloud_registered":
        from sensor_msgs.msg import PointCloud2

        return PointCloud2
    if name in {"/aft_mapped_to_init", "/odometry"}:
        from nav_msgs.msg import Odometry

        return Odometry
    if name == "/cmd_vel":
        from geometry_msgs.msg import Twist

        return Twist
    if name == "/localization/state":
        from foray_localization_ros.msg import LocalizationState

        return LocalizationState
    # 真值走 gz 的 pose/info，桥过来是 TFMessage（见 config/bridge.yaml 与 accept.py）
    from tf2_msgs.msg import TFMessage

    return TFMessage


def check_tf(buffer):
    """逐段查 TF 链，返回 {chain_ok, detail}。"""
    from rclpy.time import Time

    broken = []
    for parent, child in TF_CHAIN:
        try:
            buffer.lookup_transform(parent, child, Time())
        except Exception as exc:  # noqa: BLE001 - tf2 抛的异常类型不止一种
            broken.append(f"{parent}->{child}（{type(exc).__name__}）")
    return {
        "chain_ok": not broken,
        "detail": " → ".join(f"{p}->{c}" for p, c in TF_CHAIN)
        + ("" if not broken else "；断在：" + "、".join(broken)),
    }


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    try:
        import rclpy
    except ModuleNotFoundError:
        # 宿主没有 ROS：给出可执行的提示，而不是一坨 traceback
        print(
            "需要 ROS 2 环境（rclpy）：先 source /opt/ros/humble/setup.bash，"
            "或在 foray_sim:humble-fortress 容器里跑",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = collect(args)()
    started = node.get_clock().now().nanoseconds * 1e-9
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            got_required = all(node.stamps[name] for name in REQUIRED_TOPICS)
            if got_required and now - started >= args.window:
                break
            if now - started >= args.deadline:
                break

        topics = evaluate_health(node.stamps)
        report = build_report(
            topics=topics,
            tf=check_tf(node.tf_buffer),
            localization=summarize_localization(node.localization, now),
            cmd_vs_truth=summarize_cmd_vs_truth(node.cmd_speed, node.truth),
            window_s=round(now - started, 2),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        print(format_table(report))
    # 退出码按 Unix 习惯：0 = 全部正常，1 = 有异常项（便于 `tune_report.py || 排查`）
    return 0 if overall_ok(report) else 1


if __name__ == "__main__":
    sys.exit(main())
