#!/usr/bin/env python3
"""仿真启动：场地世界 + 机器人 + ros_gz 桥。

本文件只做「起仿真」这一件事：世界、机器人、桥接。定位（point_lio +
diesel_localization）与寻路（Nav2）分别在 localization_sim.launch.py /
navigation_sim.launch.py 里，这样单独调点云时不必拖起整链。

装配输入只有三份规格（decision.md D28）：`config/fields/<field>.yaml`（场地）、
`config/robots/<robot>.yaml`（机器人）、`config/bridge.yaml`（桥接契约）。
这里不书写世界路径、机器人路径或 gz 话题——它们全部来自规格，`field:=` / `robot:=`
只在目录里有多份规格时才需要显式给出。

用法：
    ros2 launch diesel_simulation_bringup simulation.launch.py
    ros2 launch diesel_simulation_bringup simulation.launch.py gui:=true rviz:=true
    ros2 launch diesel_simulation_bringup simulation.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from diesel_sim_tools import bridge as bridge_contract
from diesel_sim_tools import engine_gz
from diesel_sim_tools import profile as sim_profile


def gz_bridge_arguments(share_dir, context_values):
    """从 config/bridge.yaml 生成 parameter_bridge 的参数与重映射。

    解析与渲染都在 `diesel_sim_tools.bridge`：契约里的 gz 话题是模板，用场地与机器人
    规格的占位符展开；渲染后残留占位符会抛错并指出条目 `id`，而不是生成一条
    「桥建起来了但数据全断」的参数（D20）。
    """
    entries = bridge_contract.load(bridge_contract.default_path(share_dir))
    return bridge_contract.render_entries(entries, context_values)


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


def _assemble(context, share_dir):
    """按规格拼出「资源路径 + 世界 + 机器人 + 桥 + RViz」这一串动作。

    放在 OpaqueFunction 里是因为 `field:=` / `robot:=` 的取值只能在启动时确定，
    而节点参数必须在描述构建时给出最终字符串。
    """
    field = sim_profile.load_field_named(
        share_dir, LaunchConfiguration("field").perform(context)
    )
    robot = sim_profile.load_robot_named(
        share_dir, LaunchConfiguration("robot").perform(context)
    )
    gui = LaunchConfiguration("gui").perform(context) == "true"
    headless = LaunchConfiguration("headless").perform(context) == "true"

    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    actions = [
        # 世界 SDF 里的 model:// 只能靠资源路径解析；不给就是
        # `Error Code 13: Unable to find uri[model://<field>]`。
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", field.models_dir),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", field.models_dir),
    ]

    # -r 立刻开跑；headless 时用 -s 只跑服务器（服务器/CI 只跑物理与传感器）
    if gui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": engine_gz.world_args(
                        field.world,
                        headless=False,
                        gui_config=os.path.join(share_dir, "config", "gui.config"),
                    ),
                    "on_exit_shutdown": "true",
                }.items(),
            )
        )
    if headless:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": engine_gz.world_args(field.world, headless=True),
                    "on_exit_shutdown": "true",
                }.items(),
            )
        )

    actions.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=engine_gz.spawn_args(
                field.name,
                robot.sdf,
                robot.model,
                LaunchConfiguration("x"),
                LaunchConfiguration("y"),
                LaunchConfiguration("yaw"),
            ),
            output="screen",
        )
    )

    arguments, remappings = gz_bridge_arguments(
        share_dir, sim_profile.assembly_context(field, robot)
    )
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=arguments,
            remappings=remappings,
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=[robot.sdf],
            parameters=[{"use_sim_time": True, "frame_prefix": ""}],
            output="screen",
        )
    )
    actions.append(OpaqueFunction(function=_rviz_node, kwargs={"share_dir": share_dir}))
    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("diesel_simulation_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "field",
                default_value="",
                description=(
                    "config/fields/ 下的场地规格名（不带 .yaml）。留空表示用目录里唯一的"
                    "那份；目录里有多份时必须显式指定"
                ),
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="",
                description="config/robots/ 下的机器人规格名，留空规则同 field",
            ),
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
            OpaqueFunction(function=_assemble, kwargs={"share_dir": bringup_share}),
        ]
    )
