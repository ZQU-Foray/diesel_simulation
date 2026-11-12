#!/usr/bin/env python3
"""RMUL 2027 仿真启动：场地世界 + 机器人 + ros_gz 桥。

只做「起仿真」这一件事：世界、机器人、桥接。定位（point_lio + diesel_localization）
与寻路（Nav2）分别在 localization_sim.launch.py / navigation_sim.launch.py 里，
这样单独调点云时不必拖起整链。

用法：
    ros2 launch diesel_simulation_bringup simulation.launch.py
    ros2 launch diesel_simulation_bringup simulation.launch.py gui:=true rviz:=true
    ros2 launch diesel_simulation_bringup simulation.launch.py headless:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "rmul_2027"
ROBOT_NAME = "diesel_sim_robot"


def gz_bridge_arguments(share_dir):
    """从 config/bridge.yaml 生成 parameter_bridge 的参数与重映射。

    方向符号：`[` = Gazebo→ROS，`]` = ROS→Gazebo。
    话题名在桥里统一用 Gazebo 侧名字，ROS 侧靠 remappings 改名，避免依赖
    ros_gz_bridge 的双侧命名语法（不同小版本行为不一致）。
    """
    config_path = os.path.join(share_dir, "config", "bridge.yaml")
    with open(config_path, encoding="utf-8") as stream:
        entries = yaml.safe_load(stream)

    arguments = []
    remappings = []
    for entry in entries:
        direction = entry.get("direction", "gz2ros")
        separator = {"gz2ros": "[", "ros2gz": "]"}.get(direction)
        if separator is None:
            raise ValueError(
                f"bridge.yaml 里的 direction 只能是 gz2ros/ros2gz：{entry}"
            )
        arguments.append(
            f"{entry['gz_topic']}@{entry['ros_type']}{separator}{entry['gz_type']}"
        )
        if entry.get("ros_topic") and entry["ros_topic"] != entry["gz_topic"]:
            remappings.append((entry["gz_topic"], entry["ros_topic"]))
    return arguments, remappings


def resolve_rviz_config(share_dir, value):
    """把 `rviz_config` 解析成 RViz 能直接加载的绝对路径。

    允许两种写法：`share/rviz/` 下的文件名（`sim.rviz` / `tune.rviz`），或绝对路径
    （临时改视图时不必往包里塞文件）。找不到就抛错并列出现有候选——RViz 用不存在的
    配置启动时只会开一片空窗，那比启动失败更难排查。
    """
    path = value if os.path.isabs(value) else os.path.join(share_dir, "rviz", value)
    if not os.path.isfile(path):
        rviz_dir = os.path.join(share_dir, "rviz")
        available = sorted(
            name
            for name in os.listdir(rviz_dir)
            if name.endswith(".rviz") and os.path.isfile(os.path.join(rviz_dir, name))
        )
        raise RuntimeError(
            f"rviz_config 指向的配置不存在：{path}；可用配置：{', '.join(available)}"
        )
    return path


def _rviz_node(context, share_dir):
    """按 `rviz` / `rviz_config` 参数决定是否起 RViz 以及加载哪份视图。"""
    if LaunchConfiguration("rviz").perform(context) != "true":
        return []
    config_path = resolve_rviz_config(
        share_dir, LaunchConfiguration("rviz_config").perform(context)
    )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", config_path],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    ]


def generate_launch_description():
    field_share = get_package_share_directory("diesel_rmul_2027_field")
    bringup_share = get_package_share_directory("diesel_simulation_bringup")

    world_sdf = os.path.join(field_share, "worlds", f"{WORLD_NAME}.sdf")
    robot_sdf = os.path.join(bringup_share, "models", ROBOT_NAME, "model.sdf")

    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    robot_x = LaunchConfiguration("x")
    robot_y = LaunchConfiguration("y")
    robot_yaw = LaunchConfiguration("yaw")

    arguments, remappings = gz_bridge_arguments(bringup_share)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        # -r 立刻开跑；gui:=false 时用 -s 无界面（服务器/CI 只跑物理与传感器）
        launch_arguments={
            "gz_args": [
                world_sdf,
                " -r -v 3",
                " --gui-config " + os.path.join(bringup_share, "config", "gui.config"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": [world_sdf, " -r -s -v 3 --headless-rendering"],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            WORLD_NAME,
            "-file",
            robot_sdf,
            "-name",
            ROBOT_NAME,
            "-x",
            robot_x,
            "-y",
            robot_y,
            "-z",
            "0.02",
            "-Y",
            robot_yaw,
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=arguments,
        remappings=remappings,
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[robot_sdf],
        parameters=[{"use_sim_time": True, "frame_prefix": ""}],
        output="screen",
    )

    rviz_node = OpaqueFunction(function=_rviz_node, kwargs={"share_dir": bringup_share})

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="false", description="开 Gazebo GUI"
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="无界面跑物理与传感器（服务器默认）",
            ),
            DeclareLaunchArgument("rviz", default_value="false", description="开 RViz"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="sim.rviz",
                description=(
                    "rviz/ 下的视图文件名（sim.rviz 验收最小集 / tune.rviz 调车全量视图），"
                    "或任意绝对路径；文件不存在时启动即报错并列出候选"
                ),
            ),
            DeclareLaunchArgument(
                "x", default_value="0.0", description="初始 x（场地坐标）"
            ),
            DeclareLaunchArgument(
                "y", default_value="0.0", description="初始 y（场地坐标）"
            ),
            DeclareLaunchArgument(
                "yaw", default_value="0.0", description="初始朝向（rad）"
            ),
            # 只跑无界面渲染时才需要软件 GL；有 GPU 时设 LIBGL_ALWAYS_SOFTWARE=0
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE", os.environ.get("LIBGL_ALWAYS_SOFTWARE", "1")
            ),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", os.path.join(field_share, "models")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(field_share, "models")
            ),
            gz_sim,
            gz_sim_headless,
            spawn_robot,
            bridge,
            robot_state_publisher,
            rviz_node,
        ]
    )
