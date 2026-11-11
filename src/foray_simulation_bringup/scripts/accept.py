#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
#
# 运行时验收脚本：把 README §1 的 A2/A3/A6 判据从「人肉看话题」变成「一条命令出回执」。
#
# 为什么放进包里（decision.md D18）：判据要能被 CI 之外的任何人重跑；仓根的 `scripts/ci/`
# 只放纯标准库的仓级门禁（tree.md 有约束），所以运行时验收脚本装在 `lib/<pkg>/` 下，
# 与 sim_up.sh 同一条安装路径。
#
# 三个子命令都只打印单行 key=value，便于贴进 plan.md / decision.md 的回执：
#   ready  等整链就绪：/livox/lidar 首帧、/localization/state 首次 pose_valid、
#          bt_navigator 与 controller_server 均 active
#   a2     定位精度保持（复刻 decision.md D13 的口径）
#   a3     Nav2 穿隧道可达：发 NavigateToPose 到隧道西侧，断言 SUCCEEDED 且轨迹采样点
#          落进隧道顶投影（decision.md D16 的判定口径）
#
# 真值取 gz 的 /sim/pose_info（world 系 = map 系，官方图 origin 就是 world 坐标）；
# 取不到真值时如实标 ground_truth=none 并把该项判据判失败，绝不放水（AGENTS.md）。

import argparse
import math
import sys
import time

import rclpy
from foray_localization_ros.msg import LocalizationState
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# 派生导航图里隧道开放段的投影（与 map/nav2 的派生口径、test/test_nav_map.py 一致）。
# 写死在这里是刻意的：验收脚本要独立于被测实现，才能判实现错没错。
TUNNEL_RECT = (3.325, 3.675, 7.06, 7.86)

# 仿真模型名（models/foray_sim_robot/model.sdf）
MODEL_NAME = "foray_sim_robot"

# 判据阈值（decision.md D13/D16）
A2_FIRST_OK_DEADLINE_S = 10.0
A2_MAX_XY_ERR_M = 0.30
A2_MAX_YAW_ERR_DEG = 5.0

# 轨迹采样率：5 Hz 足以在 0.8 m 宽、1.0 m/s 上限的隧道里取到多个点（穿过 0.35 m 金属顶
# 投影段约 0.35–0.5 s，至少 2 个采样点）
TRACE_HZ = 5.0


def log_kv(prefix="", **fields):
    """单行 key=value 输出。"""
    body = " ".join(f"{key}={value}" for key, value in fields.items())
    print(f"{prefix}{body}".strip(), flush=True)


def yaw_of(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def lifecycle_state_name(node, service_name):
    """查生命周期节点当前状态，返回小写名字（active / inactive / …）与原始 id。"""
    client = node.create_client(GetState, service_name)
    if not client.wait_for_service(timeout_sec=1.0):
        return None, None
    future = client.call_async(GetState.Request())
    deadline = time.monotonic() + 2.0
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        return None, None
    state = future.result().current_state
    names = {
        LifecycleState.PRIMARY_STATE_UNKNOWN: "unknown",
        LifecycleState.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
        LifecycleState.PRIMARY_STATE_INACTIVE: "inactive",
        LifecycleState.PRIMARY_STATE_ACTIVE: "active",
        LifecycleState.PRIMARY_STATE_FINALIZED: "finalized",
    }
    return names.get(state.id, f"id{state.id}"), state.id


class AcceptNode(Node):
    """一次采集/验收用的节点：只订阅，不发控制指令（a3 只发导航目标）。"""

    def __init__(self, args):
        super().__init__("accept")
        self.args = args
        self.lidar_frames = 0
        self.state_msg = None
        self.state_ok_seen = False
        self.last_state = None
        self.truth = None
        self.truth_frame = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PointCloud2, "/livox/lidar", self._on_lidar, sensor_qos
        )
        self.create_subscription(
            LocalizationState, "/localization/state", self._on_state, 10
        )
        self.create_subscription(TFMessage, "/sim/pose_info", self._on_pose_info, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_lidar(self, _msg):
        self.lidar_frames += 1

    def _on_state(self, msg):
        self.state_msg = msg
        names = {
            LocalizationState.STATE_INITIALIZING: "INITIALIZING",
            LocalizationState.STATE_OK: "OK",
            LocalizationState.STATE_DEGRADED: "DEGRADED",
            LocalizationState.STATE_LOST: "LOST",
        }
        self.last_state = names.get(msg.state, str(msg.state))
        if msg.state == LocalizationState.STATE_OK and msg.pose_valid:
            self.state_ok_seen = True

    def _on_pose_info(self, msg):
        """从 gz 的 pose/info 里挑机器人本体的位姿当真值。

        桥过来的是 TFMessage，child_frame_id 是 gz 里的实体名：模型是 `foray_sim_robot`，
        link 是 `foray_sim_robot/chassis` 之类。优先用 chassis（那是 base_footprint 的原点），
        没有就退回模型根位姿，两者都要与实际使用的一致，取到哪个都打印出来。
        """
        candidates = {
            transform.child_frame_id: transform.transform
            for transform in msg.transforms
            if transform.child_frame_id == f"{MODEL_NAME}/chassis"
            or transform.child_frame_id == MODEL_NAME
            or transform.child_frame_id.startswith(f"{MODEL_NAME}/")
        }
        for name in (f"{MODEL_NAME}/chassis", MODEL_NAME):
            if name in candidates:
                self.truth = candidates[name]
                self.truth_frame = name
                return
        if candidates:
            name = min(candidates)
            self.truth = candidates[name]
            self.truth_frame = name

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def tf_pose(self, target="base_footprint", source="map"):
        """查 map→base_footprint，返回 (x, y, yaw) 或 None。"""
        try:
            stamped = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), rclpy.time.Duration(seconds=0.1)
            )
        except Exception:  # noqa: BLE001 - 查不到 TF 是正常状态，不是错误分支
            return None
        translation = stamped.transform.translation
        return translation.x, translation.y, yaw_of(stamped.transform.rotation)

    def truth_pose(self):
        if self.truth is None:
            return None
        translation = self.truth.translation
        return translation.x, translation.y, yaw_of(self.truth.rotation)


def wait_ready(node, deadline_s):
    """等整链就绪，返回 (ok, 分段耗时 dict)。分段按事件首次发生的时间计。"""
    start = time.monotonic()
    marks = {}
    lifecycle_checked = {}
    while time.monotonic() - start < deadline_s:
        rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.monotonic() - start

        if "lidar_s" not in marks and node.lidar_frames > 0:
            marks["lidar_s"] = round(elapsed, 2)
        if "localization_s" not in marks and node.state_ok_seen:
            marks["localization_s"] = round(elapsed, 2)

        # 生命周期查询要等节点起来，别每一轮都打服务调用
        if "bt_navigator" not in marks and "localization_s" in marks:
            name, state_id = lifecycle_state_name(node, "/bt_navigator/get_state")
            lifecycle_checked["bt_navigator"] = name
            if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
                marks["bt_navigator"] = round(elapsed, 2)
        if "controller_server" not in marks and "localization_s" in marks:
            name, state_id = lifecycle_state_name(node, "/controller_server/get_state")
            lifecycle_checked["controller_server"] = name
            if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
                marks["controller_server"] = round(elapsed, 2)

        if len(marks) == 4:
            return True, marks, lifecycle_checked

    return False, marks, lifecycle_checked


def cmd_ready(node, args):
    ok, marks, lifecycle_checked = wait_ready(node, args.deadline)
    detail = {
        "ready_delay_s": marks.get("controller_server", "timeout"),
        "lidar_s": marks.get("lidar_s", "none"),
        "localization_s": marks.get("localization_s", "none"),
        "bt_navigator": lifecycle_checked.get("bt_navigator", "unknown"),
        "controller_server": lifecycle_checked.get("controller_server", "unknown"),
        "last_state": node.last_state or "none",
    }
    if ok:
        log_kv("ready=PASS ", **detail)
        return 0
    log_kv("ready=FAIL ", **detail)
    return 1


def cmd_a2(node, args):
    """A2：定位精度与健康状态保持（decision.md D13 的口径）。"""
    start = time.monotonic()
    first_ok_delay = None
    samples = 0
    lost = 0
    max_xy_err = 0.0
    max_yaw_err_deg = 0.0
    truth_used = 0
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (
            not node.state_ok_seen
            and first_ok_delay is None
            and node.state_msg is not None
            and node.state_msg.pose_valid
        ):
            first_ok_delay = time.monotonic() - start
        if node.last_state == "LOST":
            lost += 1
        message = node.state_msg
        if message is None or not message.pose_valid:
            continue
        samples += 1
        truth = node.truth_pose()
        if truth is None:
            continue
        truth_used += 1
        pose = message.pose.pose
        xy_err = math.hypot(pose.position.x - truth[0], pose.position.y - truth[1])
        yaw_err = abs(math.degrees(angle_diff(yaw_of(pose.orientation), truth[2])))
        max_xy_err = max(max_xy_err, xy_err)
        max_yaw_err_deg = max(max_yaw_err_deg, yaw_err)

    if first_ok_delay is None and node.state_ok_seen:
        first_ok_delay = 0.0
    detail = {
        "samples": samples,
        "first_ok_delay": (
            f"{first_ok_delay:.2f}" if first_ok_delay is not None else "none"
        ),
        "max_xy_err": f"{max_xy_err:.3f}" if truth_used else "none",
        "max_yaw": f"{max_yaw_err_deg:.3f}" if truth_used else "none",
        "last_state": node.last_state or "none",
        "lost": lost,
        "ground_truth": node.truth_frame or "none",
    }
    ok = (
        samples > 0
        and truth_used > 0
        and first_ok_delay is not None
        and first_ok_delay <= A2_FIRST_OK_DEADLINE_S
        and max_xy_err < A2_MAX_XY_ERR_M
        and max_yaw_err_deg < A2_MAX_YAW_ERR_DEG
        and lost == 0
    )
    detail["verdict"] = "PASS" if ok else "FAIL"
    log_kv("a2 ", **detail)
    return 0 if ok else 1


def cmd_a3(node, args):
    """A3：Nav2 必须穿隧道到达西侧目标点（decision.md D16 的判定口径）。"""
    ok, marks, _lifecycle_checked = wait_ready(node, args.deadline)
    if not ok:
        log_kv(
            "a3 ",
            result="NOT_READY",
            ready_delay_s=marks.get("controller_server", "timeout"),
            last_state=node.last_state or "none",
            verdict="FAIL",
        )
        return 1

    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    if not client.wait_for_server(timeout_sec=float(args.deadline)):
        log_kv("a3 ", result="NO_ACTION_SERVER", verdict="FAIL")
        return 1

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

    send_start = time.monotonic()
    goal_future = client.send_goal_async(goal)
    while not goal_future.done() and time.monotonic() - send_start < 20.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not goal_future.done():
        log_kv("a3 ", result="SEND_TIMEOUT", verdict="FAIL")
        return 1
    goal_handle = goal_future.result()
    if not goal_handle.accepted:
        log_kv("a3 ", result="REJECTED", verdict="FAIL")
        return 1

    result_future = goal_handle.get_result_async()
    trace = []
    max_xy_err = 0.0
    truth_used = 0
    next_sample = time.monotonic()
    while not result_future.done() and time.monotonic() - send_start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now < next_sample:
            continue
        next_sample = now + 1.0 / TRACE_HZ
        pose = node.tf_pose()
        if pose is not None:
            trace.append((now - send_start, pose[0], pose[1]))
        truth = node.truth_pose()
        message = node.state_msg
        if truth is not None and message is not None and message.pose_valid:
            truth_used += 1
            max_xy_err = max(
                max_xy_err,
                math.hypot(
                    message.pose.pose.position.x - truth[0],
                    message.pose.pose.position.y - truth[1],
                ),
            )

    timed_out = not result_future.done()
    if timed_out:
        result_name = "TIMEOUT"
    else:
        status = result_future.result().status
        result_name = {
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }.get(status, f"status{status}")

    path_len = sum(
        math.hypot(trace[i][1] - trace[i - 1][1], trace[i][2] - trace[i - 1][2])
        for i in range(1, len(trace))
    )
    cross_samples = sum(
        1
        for _t, x, y in trace
        if TUNNEL_RECT[0] <= x <= TUNNEL_RECT[1]
        and TUNNEL_RECT[2] <= y <= TUNNEL_RECT[3]
    )
    elapsed = (
        (trace[-1][0] if trace else 0.0) if timed_out else time.monotonic() - send_start
    )

    ok = result_name == "SUCCEEDED" and cross_samples >= 1
    detail = {
        "result": result_name,
        "time_s": f"{elapsed:.2f}",
        "path_len_m": f"{path_len:.3f}",
        "cross_samples": cross_samples,
        "trace_samples": len(trace),
        "max_xy_err": f"{max_xy_err:.3f}" if truth_used else "none",
        "ground_truth": node.truth_frame or "none",
        "verdict": "PASS" if ok else "FAIL",
    }
    log_kv("a3 ", **detail)
    return 0 if ok else 1


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(
        description="A2/A3/A6 运行时验收（判据见 README §1 与 decision.md D13/D16）"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    ready = subparsers.add_parser("ready", help="等整链就绪")
    ready.add_argument("--deadline", type=float, default=300.0, help="最长等到多少秒")

    a2 = subparsers.add_parser("a2", help="定位精度保持")
    a2.add_argument("--duration", type=float, default=70.0, help="采样多少秒")

    a3 = subparsers.add_parser("a3", help="导航穿隧道可达")
    a3.add_argument("--x", type=float, default=2.0, help="目标 x（隧道西侧）")
    a3.add_argument("--y", type=float, default=7.4, help="目标 y（隧道中线）")
    a3.add_argument("--yaw", type=float, default=math.pi, help="目标朝向（默认朝西）")
    a3.add_argument("--deadline", type=float, default=300.0, help="等就绪的最长秒数")
    a3.add_argument("--timeout", type=float, default=240.0, help="单次导航的最长秒数")

    args, ros_args = parser.parse_known_args(argv[1:])
    rclpy.init(args=remove_ros_args(ros_args))

    node = AcceptNode(args)
    try:
        if args.command == "ready":
            return cmd_ready(node, args)
        if args.command == "a2":
            return cmd_a2(node, args)
        return cmd_a3(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
