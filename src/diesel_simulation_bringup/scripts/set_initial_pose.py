#!/usr/bin/env python3
# Copyright 2026 ZQU-Foray
#
# 一键把定位节点摆到已知初始位姿。
#
# 为什么需要：`diesel_localization` 的 GICP 初值只接受 0.5 m 以内的偏移
# （`align_max_correspondence_m`），而仿真机器人 spawn 在场地中央而不是 map 原点，
# 身份初值必然失败。这里在整链起来后调用 Relocalize 服务（MODE_GEOMETRIC）把
# map→base 直接摆到 spawn 位姿，避免「为了对齐地图而把机器人塞到原点」这种假验收。

import math

import rclpy
from diesel_localization_ros.srv import Relocalize
from geometry_msgs.msg import Pose, PoseWithCovariance
from rclpy.node import Node

DEFAULT_TIMEOUT_SEC = 60.0


class InitialPoseSetter(Node):
    def __init__(self) -> None:
        super().__init__("sim_set_initial_pose")
        # namespace 由 launch 注入（/diesel/<robot>）；service_name 非空时优先，便于单独调试。
        self.declare_parameter("robot_namespace", "/diesel/sentry")
        self.declare_parameter("service_name", "")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("startup_timeout_sec", DEFAULT_TIMEOUT_SEC)
        self.service_name = self.get_parameter("service_name").value or (
            f"{self.get_parameter('robot_namespace').value}/localization/relocalize"
        )
        self.client = self.create_client(Relocalize, self.service_name)

    def wait_and_call(self) -> bool:
        timeout = self.get_parameter("startup_timeout_sec").value
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"等待服务 {self.service_name} 超时（{timeout:.0f} s）"
            )
            return False

        request = Relocalize.Request()
        request.mode = Relocalize.Request.MODE_GEOMETRIC
        pose = Pose()
        pose.position.x = float(self.get_parameter("x").value)
        pose.position.y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        request.initial_pose = PoseWithCovariance()
        request.initial_pose.pose = pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            self.get_logger().error("重定位服务无响应")
            return False
        if not response.accepted:
            self.get_logger().error(f"重定位被拒绝：{response.message}")
            return False
        self.get_logger().info(
            f"已摆位到 ({pose.position.x:.2f}, {pose.position.y:.2f})：{response.message}"
        )
        return True


def main() -> int:
    rclpy.init()
    node = InitialPoseSetter()
    try:
        ok = node.wait_and_call()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
