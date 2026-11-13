# Copyright 2026 ZQU-Foray
#
# 「点云 → 定位」整链：场地仿真 + 机器人 + gz 桥 + 点云时序修复 + point_lio + diesel_localization。
#
# 数据链：
#   gpu_lidar ──桥──> /livox/lidar ──ign_sim_pointcloud_tool──> /velodyne_points ──> point_lio
#                                              └─ 补 time / ring（仿真点云没有逐点时间戳）
#   /aft_mapped_to_init（point_lio 里程计）──> diesel_localization 的 odom
#   /livox/lidar（原始点云，帧 front_mid360）──> diesel_localization 的 cloud
#
# 定位节点用 `diesel_localization_bringup` 自带的兵种参数（sentry.yaml）当基线，
# 只覆盖仿真相关项，避免两处默认值漂移。
#
# 雷达装位、线数、视场与地图目录全部来自规格（config/robots、config/fields，D29），
# 本文件不写这些数值：换机器人或换场地时只改规格。


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from diesel_sim_tools import profile as sim_profile


def _initial_pose(context):
    """整链起来后把定位摆到 spawn 位姿（GICP 只吃 0.5 m 内的初值）。"""
    if LaunchConfiguration("set_initial_pose").perform(context) != "true":
        return []
    return [
        TimerAction(
            period=float(LaunchConfiguration("initial_pose_delay").perform(context)),
            actions=[
                Node(
                    package="diesel_simulation_bringup",
                    executable="set_initial_pose.py",
                    name="sim_set_initial_pose",
                    output="screen",
                    parameters=[
                        {
                            "x": LaunchConfiguration("x"),
                            "y": LaunchConfiguration("y"),
                            "yaw": LaunchConfiguration("yaw"),
                            "robot_namespace": LaunchConfiguration("robot_namespace"),
                        }
                    ],
                )
            ],
        )
    ]


def _assemble(context, share_dir):
    """按规格拼出定位链的动作列表。"""
    field = sim_profile.load_field_named(
        share_dir, LaunchConfiguration("field").perform(context)
    )
    robot = sim_profile.load_robot_named(
        share_dir, LaunchConfiguration("robot").perform(context)
    )
    lidar = robot.lidar
    # 装位保持 profile 里的字符串口径（static_transform_publisher 按字符串吃参数）
    lidar_pose = [str(value) for value in lidar.pose]
    robot_namespace = LaunchConfiguration("robot_namespace")

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "simulation.launch.py")
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
            # 下位机链路（A10）：只做转发，具体动作在 simulation.launch.py 里
            "mcu_link": LaunchConfiguration("mcu_link"),
            "mcu_link_host_port": LaunchConfiguration("mcu_link_host_port"),
            "mcu_link_port": LaunchConfiguration("mcu_link_port"),
            "mcu_firmware": LaunchConfiguration("mcu_firmware"),
        }.items(),
    )

    # 传感器装在 chassis 上、TF 只由静态发布器给出（SDF 里没有 joint）
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_mid360_static_tf",
        arguments=[
            "--frame-id",
            robot.base_frame,
            "--child-frame-id",
            lidar.frame,
            "--x",
            lidar_pose[0],
            "--y",
            lidar_pose[1],
            "--z",
            lidar_pose[2],
            "--roll",
            lidar_pose[3],
            "--pitch",
            lidar_pose[4],
            "--yaw",
            lidar_pose[5],
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
            robot.base_frame,
            "--child-frame-id",
            robot.body_frame,
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

    # 环号重排参数必须与传感器视场一致：samples=线数、ang_res_y=垂直视场/(线数-1)、
    # ang_bottom 是下沿角度的绝对值（上游 point_cloud_converter.cpp:66 的换算，
    # 传负号会让正常回波全被丢掉，D11）。
    cloud_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        parameters=[
            {
                "pcd_topic": "/livox/lidar",
                "n_scan": lidar.channels,
                "horizon_scan": lidar.horizontal_samples,
                "ang_bottom": lidar.ang_bottom,
                "ang_res_y": lidar.ang_res_y,
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
                parameters=[
                    os.path.join(share_dir, "config", "point_lio_sim.yaml"),
                ],
            )
        ],
    )

    sentry_params = os.path.join(
        get_package_share_directory("diesel_localization_bringup"),
        "config",
        "sentry.yaml",
    )
    # 兵种配置的顶层键是**相对节点名**（`diesel_localization:` / `diesel_map_server:`）。
    # 节点一旦进了 /diesel/<robot> 命名空间，这两个键就匹配不上，ROS 2 会把整份参数
    # 静默丢弃——症状是启动行打印代码默认值（min_source_points=0 / max_unobservable_dofs=6）
    # 而不是兵种配置的 100/3，配准随之 initialization timeout。
    # 这里用 RewrittenYaml 把整份 YAML 挂到命名空间下，键变成 /diesel/sentry/<节点>；
    # root_key 解析为空串时不改写（见 RewrittenYaml.perform 的 `if root_key:`），
    # 所以实车（无 namespace）行为不变。
    sentry_params_ns = RewrittenYaml(
        source_file=sentry_params,
        param_rewrites={},
        root_key=robot_namespace,
    )

    localization_params = [
        sentry_params_ns,
        {
            "use_sim_time": True,
            "map_manifest": os.path.join(field.localization_dir, "manifest.yaml"),
            "verify_map_hashes": True,
            # 仿真里点云没有逐点时间戳，定位直接用原始桥出点云；里程计来自 point_lio
            "cloud_topic": "/livox/lidar",
            "odom_topic": "/aft_mapped_to_init",
            # 仿真里的雷达是全 360°（model.sdf 的 min/max_angle=±π），而 sentry.yaml 为实车
            # 30° 视场准备。这里显式改成 360°：留 30° 会让 60000 点先被裁到 ~5000 点、
            # 再过 0.10 m 体素塌到 64~70 点，GICP 直接报「无有效对应」（审计 L7）。
            # 这是「仿真口径」而不是算法参数，所以写在 launch 而不改兵种配置（D59）。
            "fov_horizontal_deg": lidar.horizontal_fov_deg,
            # 诊断开关（decision.md D25）：开启后每窗口打一行耗时与一行点数漏斗。
            # 提成 launch 参数是为了不必改 yaml 就能在仿真里拿到漏斗数据——L7 的
            # source=64~70 只能靠它看出来，且它不影响任何输出。
            "profile_enabled": LaunchConfiguration("profile_enabled"),
            # 以下三个时限是仿真侧的容差，不是定位算法的指标。理由（2026-10-30 实测）：
            # 1) point_lio 里程计约 7 Hz，但消息戳取 lidar_beg_time，比 /clock 落后 0.2–0.3 s，
            #    now-last_odom 常态顶到 0.47 s，偶发更高；
            # 2) 软件渲染的点云在负载下会从 7 Hz 掉到 4 Hz，帧间隔 p95≈0.6 s、p99≈1.0 s、
            #    最长 1.6 s（宿主机 12 核被其它进程占满时）；
            # 3) diesel_localization 的 LOST 是粘滞状态，只有 Relocalize 能恢复，默认的
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
        package="diesel_localization_ros",
        executable="map_server_node",
        name="diesel_map_server",
        namespace=robot_namespace,
        output="screen",
        parameters=localization_params,
        condition=IfCondition(LaunchConfiguration("start_map_server")),
    )

    localization = Node(
        package="diesel_localization_ros",
        executable="localization_node",
        name="diesel_localization",
        namespace=robot_namespace,
        output="screen",
        parameters=localization_params,
    )

    return [
        sim_include,
        lidar_tf,
        body_tf,
        cloud_tool,
        point_lio,
        map_server,
        localization,
        OpaqueFunction(function=_initial_pose),
    ]


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
                description="rviz/ 下的视图文件名（sim.rviz / tune.rviz）或绝对路径",
            ),
            DeclareLaunchArgument("x", default_value="6.0"),
            DeclareLaunchArgument("y", default_value="4.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("set_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_delay", default_value="6.0"),
            # gazebo 沉降时间：小于该值时启动 point_lio 会因 IMU 初始化带偏重力而发散。
            DeclareLaunchArgument("point_lio_delay", default_value="4.0"),
            # ROS 运行时 namespace（foray_docs/diesel_naming_migration.md §1.5）：
            # 定位节点自身话题是相对名，这里决定全名；TF 帧名不受 namespace 影响。
            DeclareLaunchArgument("robot_namespace", default_value="/diesel/sentry"),
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
            # 下位机链路（A10、D30）：默认关，打开后 /cmd_vel 经虚拟 USB CDC 交给固件
            DeclareLaunchArgument(
                "mcu_link",
                default_value="false",
                description="走虚拟 USB CDC + 下位机固件（A10），默认直连 gz",
            ),
            DeclareLaunchArgument(
                "mcu_link_host_port", default_value="/tmp/diesel_host"
            ),
            DeclareLaunchArgument("mcu_link_port", default_value="/tmp/diesel_mcu"),
            DeclareLaunchArgument(
                "mcu_firmware",
                default_value="",
                description="foray_mcu 的 firmware_sim 可执行文件路径；留空表示由外部自己起",
            ),
            OpaqueFunction(function=_assemble, kwargs={"share_dir": bringup_share}),
        ]
    )
