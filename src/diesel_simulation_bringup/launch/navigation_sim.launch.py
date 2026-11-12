# Copyright 2026 ZQU-Foray
#
# 「整链」入口：场地仿真 + 机器人 + gz 桥 + 点云时序修复 + point_lio + diesel_localization
#                + loam_interface/sensor_scan_generation（补 odom→base_footprint）+ Nav2。
#
# 为什么还要补 TF（decision.md D17）
# ---------------------------------
# diesel_localization 只发 map→odom，point_lio 只发 camera_init→aft_mapped，谁都不发
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
# navigation_launch.py 不含 map_server，也不含 amcl——本仓定位已由 diesel_localization 负责，
# 再起 amcl 会抢 map→odom）。
#
# 参数文件、派生导航图与帧名同样来自规格（config/fields、config/robots，D28）：
# 换场地换的是 nav2 参数与派生图，换机器人换的是雷达帧与 base 帧，本文件不写这些值。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from diesel_sim_tools import profile as sim_profile


def _loam_params(robot):
    """loam_interface 的参数：与上游 pb2025_nav_bringup 仿真配置同口径。

    `robot_base_frame` 用 `base_footprint`（本车没有云台，上游这里是 gimbal_yaw）。
    """
    return {
        "state_estimation_topic": "/aft_mapped_to_init",
        "registered_scan_topic": "/cloud_registered",
        "odom_frame": "odom",
        "base_frame": robot.base_frame,
        "lidar_frame": robot.lidar.frame,
    }


def _scan_params(robot):
    """sensor_scan_generation 的参数：把里程计补成 odom→base_footprint。"""
    return {
        "lidar_frame": robot.lidar.frame,
        "base_frame": robot.base_frame,
        "robot_base_frame": robot.base_frame,
    }


def _assemble(context, share_dir):
    """按规格拼出整链的动作列表。"""
    field = sim_profile.load_field_named(
        share_dir, LaunchConfiguration("field").perform(context)
    )
    robot = sim_profile.load_robot_named(
        share_dir, LaunchConfiguration("robot").perform(context)
    )
    robot_namespace = LaunchConfiguration("robot_namespace")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "localization_sim.launch.py")
        ),
        launch_arguments={
            "field": LaunchConfiguration("field"),
            "robot": LaunchConfiguration("robot"),
            "headless": LaunchConfiguration("headless"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
            "robot_namespace": robot_namespace,
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
            parameters=[dict(_loam_params(robot), use_sim_time=True)],
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            output="screen",
            # 依赖 base_footprint→front_mid360 静态 TF；TF 还没到时进程会退，靠 respawn 兜住
            respawn=True,
            respawn_delay=2.0,
            parameters=[dict(_scan_params(robot), use_sim_time=True)],
        ),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            field.nav2_params,
            {
                "use_sim_time": True,
                # 派生图（只打开隧道开放段）而不是定位侧的官方图，理由见 decision.md D16
                "yaml_filename": field.nav_map,
                "topic_name": "map",
            },
        ],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        namespace=robot_namespace,
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

    # TF 中继（D28）：nav2 的 tf2 监听器按相对名订阅，进 namespace 后读
    # <ns>/tf(_static)，而发布者都在根 /tf ⇒ 必须有人在命名空间里「多一个发布者」。
    # 实机无命名空间时把 robot_namespace 置空并关掉这个开关（`tf_relay:=false`）。
    tf_relay = Node(
        package="diesel_simulation_bringup",
        executable="tf_relay.py",
        name="tf_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_namespace": LaunchConfiguration("robot_namespace"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("tf_relay")),
    )

    # Nav2 整栈进 /diesel/<robot>：humable 的 navigation_launch.py 只把 namespace 用于
    # 参数 YAML 的 root_key（参数键被改写成 /diesel/sentry/<节点>）和组合容器名，
    # **不会**给各 Node 加 namespace——所以必须在这里用 PushRosNamespace 包一层，
    # 否则参数键与节点名对不上，Nav2 退回默认参数（实测：controller_server
    # 报 "Couldn't load critics!"、frequency 变成默认 20 Hz）。
    # 跨边界的 /odometry、/cmd_vel、TF 帧名在参数与桥接里显式写成绝对名。
    navigation = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
            tf_relay,
            # Nav2 各节点的 tf2 监听器按**相对名** `tf`/`tf_static` 订阅，进 namespace 后
            # 落到 /diesel/sentry/tf（实测 5 个订阅、0 个发布，TF 全在根 /tf 上）⇒
            # 代价地图永远「Invalid frame ID "odom"」、bt_navigator 卡在 inactive。
            # 在这层挂 SetRemap 不生效（remap 不随 IncludeLaunchDescription 传播，
            # 子进程命令行没有 -r tf:=/tf），所以用中继喂命名空间（D28）。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("nav2_bringup"),
                        "launch",
                        "navigation_launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": field.nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                    # Nav2 自己会按 root_key 重写参数键，这里给同一个名字，避免两处
                    # namespace 口径不一致
                    "namespace": robot_namespace,
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("start_nav2")),
    )

    # 调车面板组（I13）：脚本自己检查各 rqt 面板在不在，缺哪个少哪个，不影响整链
    rqt_tune = Node(
        package="diesel_simulation_bringup",
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

    return [localization, *odom_bridge, rqt_tune, nav2_stack]


def generate_launch_description():
    bringup_share = get_package_share_directory("diesel_simulation_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "field",
                default_value="",
                description="config/fields/ 下的场地规格名；留空用目录里唯一的那份",
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="",
                description="config/robots/ 下的机器人规格名；留空用目录里唯一的那份",
            ),
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
            # ROS 运行时 namespace（foray_docs/diesel_naming_migration.md §1.5）：
            # 定位与 Nav2 同处一个 /diesel/<robot>；上游 /livox/*、/odometry、
            # /cmd_vel、TF 帧名仍是全局的——它们是跨边界契约，不由本仓拼前缀。
            DeclareLaunchArgument("robot_namespace", default_value="/diesel/sentry"),
            DeclareLaunchArgument(
                "tf_relay",
                default_value="true",
                description="把根 /tf、/tf_static 中继进命名空间（无命名空间时关掉）",
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            # 定位实测：point_lio 4 s 起、初值 6 s 注入、/localization/state 首次 OK 约 8 s；
            # 25 s 留足软件渲染下的余量（accept.py ready 会打印实际 ready_delay_s）
            DeclareLaunchArgument(
                "nav2_delay",
                default_value="25.0",
                description="延迟多少秒再起 Nav2（等 map→odom 与定位稳定）",
            ),
            OpaqueFunction(function=_assemble, kwargs={"share_dir": bringup_share}),
        ]
    )
