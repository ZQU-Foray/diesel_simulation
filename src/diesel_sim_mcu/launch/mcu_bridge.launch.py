# Copyright 2026 ZQU-Foray
"""只起过渡桥（调试用）：链路两端的端点是参数，方便对着任意 PTY 或真设备试。

整链（`sim_up.sh --mcu-link`）里这个节点由 `simulation.launch.py` 代起，不必单独跑：
    ros2 launch diesel_sim_mcu mcu_bridge.launch.py
    ros2 launch diesel_sim_mcu mcu_bridge.launch.py port:=/dev/ttyDIESEL_HOST
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/tmp/diesel_host",
                description="上位机侧的 CDC 端点（virtual_cdc.py 的 --host-link）",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/diesel/sentry/cmd_vel",
                description="Nav2 侧的下行话题（D27 钉成绝对名）",
            ),
            DeclareLaunchArgument(
                "gz_topic",
                default_value="/model/diesel_sim_robot/cmd_vel",
                description="gz 车体的速度话题（与 bridge.yaml 的 id: cmd_vel 同一个）",
            ),
            Node(
                package="diesel_sim_mcu",
                executable="sim_mcu_bridge",
                name="sim_mcu_bridge",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "gz_topic": LaunchConfiguration("gz_topic"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
