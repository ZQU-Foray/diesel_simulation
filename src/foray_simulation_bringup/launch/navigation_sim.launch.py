# Copyright 2026 ZQU-Foray
#
# 「整链」入口：场地仿真 + 机器人 + gz 桥 + 点云时序修复 + point_lio + foray_localization
#                + loam_interface/sensor_scan_generation（补 odom→base_footprint）+ Nav2。
#
# 为什么还要补 TF（decision.md D17）
# ---------------------------------
# foray_localization 只发 map→odom，point_lio 只发 camera_init→aft_mapped，谁都不发
# odom→base_footprint。Nav2 的代价地图和控制器都要 odom→base_footprint，所以用上游两件套：
#   point_lio /aft_mapped_to_init（camera_init→body）
#     └─ loam_interface ──> lidar_odometry（odom→front_mid360，不发 TF）
#                          registered_scan（odom 系点云）
#          └─ sensor_scan_generation ──> odom→base_footprint TF + odometry 话题
# 两者参数与上游 pb2025_nav_bringup 的仿真配置一致（lidar_frame=front_mid360）。
#
# 起停顺序
# --------
# Nav2 必须等 map→odom 存在后才能激活代价地图，所以整栈用 TimerAction 延后
# `nav2_delay` 秒。map_server 与它自己的 lifecycle_manager 由本仓起（Nav2 的 stock
# navigation_launch.py 不含 map_server，也不含 amcl——本仓定位已由 foray_localization 负责，
# 再起 amcl 会抢 map→odom）。

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 与上游 pb2025_nav_bringup 仿真参数一致：点云里程计在 odom 系、雷达帧 front_mid360。
# robot_base_frame 用 base_footprint（本车没有云台，上游这里是 gimbal_yaw）。
LOAM_INTERFACE_PARAMS = {
    "state_estimation_topic": "/aft_mapped_to_init",
    "registered_scan_topic": "/cloud_registered",
    "odom_frame": "odom",
    "base_frame": "base_footprint",
    "lidar_frame": "front_mid360",
}

SCAN_GENERATION_PARAMS = {
    "lidar_frame": "front_mid360",
    "base_frame": "base_footprint",
    "robot_base_frame": "base_footprint",
}


def generate_launch_description():
    bringup_share = FindPackageShare("foray_simulation_bringup")
    localization_launch = PathJoinSubstitution(
        [bringup_share, "launch", "localization_sim.launch.py"]
    )
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "nav2_params_sim.yaml"]
    )
    nav_map = PathJoinSubstitution([bringup_share, "map", "nav2", "rmul_2027_nav.yaml"])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    # 这两个节点只做帧变换与 TF 补全，起早一点无妨：拿不到里程计就先等着
    odom_bridge = [
        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[dict(LOAM_INTERFACE_PARAMS, use_sim_time=True)],
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            output="screen",
            # 依赖 base_footprint→front_mid360 静态 TF；TF 还没到时进程会退，靠 respawn 兜住
            respawn=True,
            respawn_delay=2.0,
            parameters=[dict(SCAN_GENERATION_PARAMS, use_sim_time=True)],
        ),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
                # 派生图（只打开隧道开放段）而不是定位侧的官方图，理由见 decision.md D16
                "yaml_filename": nav_map,
                "topic_name": "map",
            },
        ],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                # 仿真里 map_server 起得慢也不该让 manager 反复重连
                "bond_timeout": 0.0,
                "node_names": ["map_server"],
            }
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_nav2")),
    )

    # 调车面板组（I13）：脚本自己检查各 rqt 面板在不在，缺哪个少哪个，不影响整链
    rqt_tune = Node(
        package="foray_simulation_bringup",
        executable="tune_rqt.sh",
        name="tune_rqt",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rqt")),
    )

    nav2_stack = TimerAction(
        period=LaunchConfiguration("nav2_delay"),
        actions=[map_server, map_lifecycle, navigation],
        condition=IfCondition(LaunchConfiguration("start_nav2")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="sim.rviz",
                description="rviz/ 下的视图文件名（sim.rviz 验收最小集 / tune.rviz 调车全量视图）或绝对路径",
            ),
            # 起点默认在蓝方半场中央，A3 终点在隧道西侧 (2.0, 7.4)
            DeclareLaunchArgument("x", default_value="6.0"),
            DeclareLaunchArgument("y", default_value="4.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "rqt",
                default_value="false",
                description="开调车 rqt 面板组（graph / topic / plot / console / tf_tree / image_view）",
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            # 定位实测：point_lio 4 s 起、初值 6 s 注入、/localization/state 首次 OK 约 8 s；
            # 25 s 留足软件渲染下的余量（accept.py ready 会打印实际 ready_delay_s）
            DeclareLaunchArgument(
                "nav2_delay",
                default_value="25.0",
                description="延迟多少秒再起 Nav2（等 map→odom 与定位稳定）",
            ),
            localization,
            *odom_bridge,
            rqt_tune,
            nav2_stack,
        ]
    )
