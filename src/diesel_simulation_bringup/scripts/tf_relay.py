#!/usr/bin/env python3
"""把全局 TF 中继进某个 ROS 命名空间（`/tf` → `<ns>/tf`，`/tf_static` → `<ns>/tf_static`）。

为什么需要：Nav2 各节点的 tf2 监听器按**相对名**订阅 `tf` / `tf_static`，一旦 Nav2 整栈
被 `PushRosNamespace` 推进 `/diesel/<robot>`，它们就落到 `/diesel/sentry/tf`，而 TF 的
发布者（`point_lio`、`sensor_scan_generation`、`diesel_localization`）都在根 `/tf` 上。
后果不是报错，而是代价地图每秒刷
`Timed out waiting for transform from base_footprint to odom … Invalid frame ID "odom"`、
`bt_navigator` 永远 `inactive`，整链 `accept.py ready` 超时（详见 decision.md D28）。

在 Nav2 的 include 上挂 `SetRemap(src="tf", dst="/tf")` 实测**不生效**（子进程命令行里
没有 `-r tf:=/tf`：`launch` 的 remap 不随 `IncludeLaunchDescription` 传播进被 include 的
描述），所以这里用中继：让命名空间里「多出一个发布者」，监听器就有帧可读了。

用法：
    ros2 run diesel_simulation_bringup tf_relay --ros-args -p target_namespace:=/diesel/sentry

参数：
    target_namespace  目标命名空间（不带尾斜杠）。为空表示中继到根本身——那会自己喂自己，
                      所以空值时直接退出，避免无限回环。
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    """把 `/tf`、`/tf_static` 原样转发到 `<target_namespace>/tf(_static)`。"""

    def __init__(self) -> None:
        super().__init__("tf_relay")

        self.declare_parameter("target_namespace", "")
        target = str(self.get_parameter("target_namespace").value).strip()

        if not target or target == "/":
            self.get_logger().error(
                "target_namespace 为空：中继到根本身会造成回环，拒绝启动"
            )
            raise SystemExit(2)

        self.target = target.rstrip("/")

        # QoS 踩过的坑：ROS 2 的兼容规则是「订阅端能接受发布端」——订阅端可靠时，
        # best_effort 发布端不兼容。nav2 的 tf2 监听器按 RELIABLE 订阅，所以这里
        # /tf 用 reliable + volatile（实测 best_effort 会把整条链拦住，日志只留
        # `New subscription discovered on topic '/diesel/sentry/tf', requesting
        #  incompatible QoS … Last incompatible policy: RELIABILITY`）；
        # /tf_static 与 tf2_ros 一致用 reliable + transient_local。
        # 入站订阅照抄源端的 QoS（/tf 在 tf2 里是 best_effort，也能收 reliable 发布端）。
        dynamic_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_tf = self.create_publisher(TFMessage, f"{self.target}/tf", dynamic_qos)
        self.pub_static = self.create_publisher(
            TFMessage, f"{self.target}/tf_static", static_qos
        )

        # 入站用 tf2 的默认（best_effort + volatile）：可靠订阅对 best_effort 发布端不兼容
        source_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, "/tf", self._forward_tf, source_qos)
        self.create_subscription(
            TFMessage, "/tf_static", self._forward_static, static_qos
        )

        self.get_logger().info(
            f"TF 中继已启动：/tf → {self.target}/tf，/tf_static → {self.target}/tf_static"
        )

    def _forward_tf(self, msg: TFMessage) -> None:
        self.pub_tf.publish(msg)

    def _forward_static(self, msg: TFMessage) -> None:
        self.pub_static.publish(msg)


def main() -> int:
    rclpy.init()
    try:
        node = TfRelay()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code or 0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
