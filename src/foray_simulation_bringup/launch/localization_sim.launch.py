# Copyright 2026 ZQU-Foray
#
# 「点云 → 定位」整链：场地仿真 + 机器人 + gz 桥 + 点云时序修复 + point_lio + foray_localization。
#
# 数据链：
#   gpu_lidar ──桥──> /livox/lidar ──ign_sim_pointcloud_tool──> /velodyne_points ──> point_lio
#                                              └─ 补 time / ring（仿真点云没有逐点时间戳）
#   /aft_mapped_to_init（point_lio 里程计）──> foray_localization 的 odom
#   /livox/lidar（原始点云，帧 front_mid360）──> foray_localization 的 cloud
#
# 定位节点用 `foray_localization_bringup` 自带的兵种参数（sentry.yaml）当基线，
# 只覆盖仿真相关项，避免两处默认值漂移。


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 迁移包 parameters.json 的实测雷达安装高度（279.62 mm）
LIDAR_POSE = ("0.08", "0.0", "0.27962", "0.0", "0.0", "0.0")

# ign_sim_pointcloud_tool 按「垂直角 → 环号」重排点，参数必须与传感器视场一致：
# samples=32、-7°…+52°（= -0.12217304764…0.90757121104 rad）→ ang_res_y = 59/31
ANG_RES_Y = "1.903225806451613"
# 注意 ang_bottom 是「下沿角度的绝对值」，不是带符号角度：上游
# point_cloud_converter.cpp:66 的换算是 row_id = (vertical_angle + ang_bottom) / ang_res_y，
# 传 -7.0 会让 row_id 只在垂直角 ≥ +7° 时成立——正常回波全被丢，只剩 atan2(inf, inf)=+45° 的
# 无效点（见 decision.md D11）。上游默认值与 pb2025_nav_bringup 的仿真参数都用 7.0。
ANG_BOTTOM = "7.0"


def _initial_pose(context, *args, **kwargs):
    """整链起来后把定位摆到 spawn 位姿（GICP 只吃 0.5 m 内的初值）。"""
    if LaunchConfiguration("set_initial_pose").perform(context) != "true":
        return []
    return [
        TimerAction(
            period=float(LaunchConfiguration("initial_pose_delay").perform(context)),
            actions=[
                Node(
                    package="foray_simulation_bringup",
                    executable="set_initial_pose.py",
                    name="sim_set_initial_pose",
                    output="screen",
                    parameters=[
                        {
                            "x": LaunchConfiguration("x"),
                            "y": LaunchConfiguration("y"),
                            "yaw": LaunchConfiguration("yaw"),
                        }
                    ],
                )
            ],
        )
    ]


def generate_launch_description():
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("foray_simulation_bringup"), "launch", "simulation.launch.py"]
    )
    point_lio_params = PathJoinSubstitution(
        [FindPackageShare("foray_simulation_bringup"), "config", "point_lio_sim.yaml"]
    )
    map_manifest = PathJoinSubstitution(
        [
            FindPackageShare("foray_simulation_bringup"),
            "map",
            "rmul_2027",
            "manifest.yaml",
        ]
    )
    sentry_params = PathJoinSubstitution(
        [FindPackageShare("foray_localization_bringup"), "config", "sentry.yaml"]
    )

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
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

    # 传感器装在 chassis 上、TF 只由静态发布器给出（SDF 里没有 joint）
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_mid360_static_tf",
        arguments=[
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "front_mid360",
            "--x",
            LIDAR_POSE[0],
            "--y",
            LIDAR_POSE[1],
            "--z",
            LIDAR_POSE[2],
            "--roll",
            LIDAR_POSE[3],
            "--pitch",
            LIDAR_POSE[4],
            "--yaw",
            LIDAR_POSE[5],
        ],
        parameters=[{"use_sim_time": True}],
    )

    # point_lio 把机体帧硬编码成 `body`（laserMapping.cpp:228 点云 header、:270 odom 的
    # child_frame_id），而定位节点要把 `body` 换算到 `base_footprint` 才收得下点云；TF 树里
    # 没有这条父级时它一帧也吃不到，状态就冻在 INITIALIZING（实测 ready=FAIL、每 2 s 一条
    # `TF body->base_footprint 失败`）。父只能是 `base_footprint`：它的父已经是上游里程计，
    # 反向发 `body -> base_footprint` 会把树拆成两棵互不相连的树（tf2 单亲规则，decision.md D25）。
    # 六个分量给 0 是实测量到的口径（恒等后 ready=PASS、准静态 1218/1218 帧 state=OK）；
    # body 是 point_lio 自己的机体原点，实车若量出固定偏移，只改这一条参数即可。
    body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_static_tf",
        arguments=[
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "body",
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
        ],
        parameters=[{"use_sim_time": True}],
        # 默认发；`body_tf:=false` 用来复现 L1（定位节点一帧都吃不到、状态冻结在
        # INITIALIZING；补上 i2 之后应变成 LOST + reason=`tf unavailable: ...`）。
        condition=IfCondition(LaunchConfiguration("body_tf")),
    )

    cloud_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        parameters=[
            {
                "pcd_topic": "/livox/lidar",
                "n_scan": 32,
                "horizon_scan": 1875,
                "ang_bottom": float(ANG_BOTTOM),
                "ang_res_y": float(ANG_RES_Y),
            }
        ],
    )

    # point_lio 用开头的 IMU 数据做重力对齐（IMU_Processing::Set_init 用 mean_acc 定 gravity），
    # 而 gazebo 起机头几秒机器人还在沉降、IMU 上有大幅瞬时加速度；此时初始化会把重力方向
    # 永久带偏，里程计随后指数发散（实测 53 s 时已漂到 x≈-759 m、z≈-6506 m）。
    # 因此等仿真稳定后再启动点云里程计，同一份参数在仿真热起时漂移只有毫米级。
    point_lio = TimerAction(
        period=LaunchConfiguration("point_lio_delay"),
        actions=[
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                name="point_lio",
                output="screen",
                parameters=[point_lio_params],
            )
        ],
    )

    localization_params = [
        sentry_params,
        {
            "use_sim_time": True,
            "map_manifest": map_manifest,
            "verify_map_hashes": True,
            # 仿真里点云没有逐点时间戳，定位直接用原始桥出点云；里程计来自 point_lio
            "cloud_topic": "/livox/lidar",
            "odom_topic": "/aft_mapped_to_init",
            # 仿真里的雷达是全 360°（model.sdf 的 min/max_angle=±π），而 sentry.yaml 为实车
            # 30° 视场准备。这里显式改成 360°：留 30° 会让 60000 点先被裁到 ~5000 点、
            # 再过 0.10 m 体素塌到 64~70 点，GICP 直接报「无有效对应」（审计 L7）。
            # 这是「仿真口径」而不是算法参数，所以写在 launch 而不改兵种配置（D59）。
            "fov_horizontal_deg": 360.0,
            # 诊断开关（decision.md D25）：开启后每窗口打一行耗时与一行点数漏斗。
            # 提成 launch 参数是为了不必改 yaml 就能在仿真里拿到漏斗数据——L7 的
            # source=64~70 只能靠它看出来，且它不影响任何输出。
            "profile_enabled": LaunchConfiguration("profile_enabled"),
            # 以下三个时限是仿真侧的容差，不是定位算法的指标。理由（2026-10-30 实测）：
            # 1) point_lio 里程计约 7 Hz，但消息戳取 lidar_beg_time，比 /clock 落后 0.2–0.3 s，
            #    now-last_odom 常态顶到 0.47 s，偶发更高；
            # 2) 软件渲染的点云在负载下会从 7 Hz 掉到 4 Hz，帧间隔 p95≈0.6 s、p99≈1.0 s、
            #    最长 1.6 s（宿主机 12 核被其它进程占满时）；
            # 3) foray_localization 的 LOST 是粘滞状态，只有 Relocalize 能恢复，默认的
            #    0.5 s/0.2 s 阈值会让它被正常帧间隔误判成断流并永久 LOST
            #    （表现为初值注入后仍 pose_valid=false）。因此按实测最坏间隔留足余量。
            # 三个值是 launch 参数而不是硬编码（审计 L6）：同一套代码两套阈值时，
            # 现场必须能从命令行改、并在节点启动日志里看到最终生效值（D59）。
            "odom_timeout_sec": LaunchConfiguration("odom_timeout_sec"),
            "lidar_timeout_sec": LaunchConfiguration("lidar_timeout_sec"),
            "lost_timeout_sec": LaunchConfiguration("lost_timeout_sec"),
        },
    ]

    map_server = Node(
        package="foray_localization_ros",
        executable="map_server_node",
        name="foray_map_server",
        output="screen",
        parameters=localization_params,
        condition=IfCondition(LaunchConfiguration("start_map_server")),
    )

    localization = Node(
        package="foray_localization_ros",
        executable="localization_node",
        name="foray_localization",
        output="screen",
        parameters=localization_params,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="sim.rviz",
                description="rviz/ 下的视图文件名（sim.rviz / tune.rviz）或绝对路径",
            ),
            DeclareLaunchArgument("x", default_value="6.0"),
            DeclareLaunchArgument("y", default_value="4.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("set_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_delay", default_value="6.0"),
            # gazebo 沉降时间：小于该值时启动 point_lio 会因 IMU 初始化带偏重力而发散。
            DeclareLaunchArgument("point_lio_delay", default_value="4.0"),
            DeclareLaunchArgument("start_map_server", default_value="true"),
            # 仿真侧健康时限（审计 L6）：默认值就是本文档实测口径，可在命令行覆盖以便
            # 复现现场阈值（例如 --odom_timeout_sec 0.5 对照实车）。节点启动时会打印
            # 合并后的最终值，两套口径不会再有「无从得知当前是哪套」的情况（D59）。
            DeclareLaunchArgument("odom_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("lidar_timeout_sec", default_value="2.5"),
            DeclareLaunchArgument("lost_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("profile_enabled", default_value="false"),
            # 故障注入开关（审计 L1/L2）：关掉即可复现「仿真没有 body 帧」，
            # 用于验证定位侧把缺帧表达成 LOST 而不是无限 INITIALIZING。
            DeclareLaunchArgument("body_tf", default_value="true"),
            sim_include,
            lidar_tf,
            body_tf,
            cloud_tool,
            point_lio,
            map_server,
            localization,
            OpaqueFunction(function=_initial_pose),
        ]
    )
