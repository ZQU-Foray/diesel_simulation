# Copyright 2026 ZQU-Foray
"""装配规格（profile）：场地与机器人各一份 YAML。

profile 是**数据**，不带任何 gz 或 nav2 的知识：它只说「这个场地用哪个世界文件、
官方地图与派生导航图在哪、验收目标点是多少」「这台机器人的机体 SDF 在哪、雷达装在
哪条链路、视场多大」。把数据翻成命令行与参数是 `engine_gz` 与 launch 的事
（decision.md D28 第 1 条）。

引用其他包的资源时写成 `{package: 包名, path: 包内相对路径}`，读的时候用
ament 索引解析成绝对路径；只写字符串时按「相对本包 share」解析。
"""

import os

import yaml

from .errors import AssemblyError

# 机体 SDF 的雷达装位由 `model.sdf` 决定；profile 里这条是**同一个值**的镜像，
# 供 static_transform_publisher 用（279.62 mm 是迁移包 parameters.json 的实测值，
# 见 localization_sim.launch.py 的原注释）。两处不一致会导致点云与地图错位，
# 所以校验时要求 6 个分量齐全、可解析成浮点。
_POSE_COMPONENTS = 6

# 雷达垂直角分辨率与视场的一致性容差：32 线、-7°…+52°（跨度 59°）对应 59/31。
_ANGLE_TOLERANCE = 1e-9


class Lidar:
    """一路激光雷达：链路名、传感器名、帧名、装位与视场。"""

    __slots__ = (
        "ang_bottom",
        "ang_res_y",
        "channels",
        "frame",
        "horizontal_fov_deg",
        "horizontal_samples",
        "imu_sensor",
        "link",
        "pose",
        "sensor",
        "vertical_fov_deg",
    )

    def __init__(self, **fields):
        for name in self.__slots__:
            setattr(self, name, fields[name])

    def __repr__(self):
        return (
            f"Lidar(link={self.link!r}, sensor={self.sensor!r}, frame={self.frame!r})"
        )


class Robot:
    """一台机器人：机体 SDF、gz 模型名、TF 帧名与它的雷达。"""

    __slots__ = ("base_frame", "body_frame", "lidar", "model", "name", "sdf")

    def __init__(self, **fields):
        for name in self.__slots__:
            setattr(self, name, fields[name])

    def __repr__(self):
        return f"Robot(name={self.name!r}, model={self.model!r})"


class Field:
    """一个场地：世界、模型目录、官方地图、派生导航图、nav2 参数与验收口径。"""

    __slots__ = (
        "goal",
        "localization_dir",
        "models_dir",
        "name",
        "nav2_params",
        "nav_map",
        "official_map",
        "tunnel_rect",
        "world",
    )

    def __init__(self, **fields):
        for name in self.__slots__:
            setattr(self, name, fields[name])

    def __repr__(self):
        return f"Field(name={self.name!r}, world={self.world!r})"


def _ament_share(package):
    """ament 索引查询。

    放在函数里而不是模块顶层：本库要被 launch、单测和 CI 的 lint 作业一起 import，
    顶层硬依赖 ament 会让「没 source 过 ROS 环境」的单跑直接 ImportError。
    """
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package)


def _require(mapping, key, where):
    value = mapping.get(key)
    if value is None or value == "":
        raise AssemblyError(f"{where} 缺少字段 `{key}`")
    return value


def _clean(mapping, where, allowed):
    if not isinstance(mapping, dict):
        raise AssemblyError(f"{where} 必须是映射，实际是 {type(mapping).__name__}")
    unknown = set(mapping) - set(allowed)
    if unknown:
        raise AssemblyError(f"{where} 有未知字段：{', '.join(sorted(unknown))}")
    return mapping


def _resolve(reference, where, share_lookup, base_share):
    """把 `{package, path}` 或裸字符串解析成绝对路径。"""
    if isinstance(reference, dict):
        _clean(reference, where, ("package", "path"))
        package = _require(reference, "package", where)
        relative = _require(reference, "path", where)
        try:
            share = share_lookup(package)
        except Exception as error:  # ament 索引查不到包时会抛 PackageNotFoundError
            raise AssemblyError(
                f"{where} 引用的包 `{package}` 不存在于 ament 索引：{error}"
            ) from error
        return os.path.join(share, relative)
    if isinstance(reference, str) and reference:
        return os.path.join(base_share, reference)
    raise AssemblyError(f"{where} 必须是 {{package, path}} 或相对本包 share 的路径")


def _load_yaml(path, where):
    if not os.path.isfile(path):
        raise AssemblyError(f"{where} 不存在：{path}")
    with open(path, encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise AssemblyError(f"{where} 必须是映射：{path}")
    return raw


def _base_share(config_path, levels):
    """从 `<share>/config/<kind>/<name>.yaml` 反推本包 share。"""
    share = config_path
    for _ in range(levels):
        share = os.path.dirname(share)
    return share


def _pose(values, where):
    if not isinstance(values, list) or len(values) != _POSE_COMPONENTS:
        raise AssemblyError(f"{where} 必须是 {_POSE_COMPONENTS} 个数的列表：{values!r}")
    pose = []
    for value in values:
        try:
            pose.append(float(value))
        except (TypeError, ValueError) as error:
            raise AssemblyError(f"{where} 含非数值分量：{value!r}") from error
    return tuple(pose)


def load_robot(path, *, share_lookup=None):
    """读机器人 profile。"""
    share_lookup = share_lookup or _ament_share
    where = f"机器人 profile {path}"
    raw = _clean(
        _load_yaml(path, where),
        where,
        ("name", "model", "sdf", "frames", "lidar"),
    )
    base_share = _base_share(path, levels=3)

    frames = _clean(
        _require(raw, "frames", where), f"{where} 的 frames", ("base", "body")
    )

    lidar_raw = _clean(
        _require(raw, "lidar", where),
        f"{where} 的 lidar",
        (
            "link",
            "sensor",
            "imu_sensor",
            "frame",
            "pose",
            "channels",
            "horizontal_samples",
            "horizontal_fov_deg",
            "vertical_fov_deg",
            "ang_res_y",
            "ang_bottom",
        ),
    )
    channels = int(_require(lidar_raw, "channels", f"{where} 的 lidar"))
    vertical_fov = float(_require(lidar_raw, "vertical_fov_deg", f"{where} 的 lidar"))
    ang_res_y = float(_require(lidar_raw, "ang_res_y", f"{where} 的 lidar"))
    if channels < 2:
        raise AssemblyError(f"{where} 的 lidar.channels 必须 ≥ 2：{channels}")
    expected = vertical_fov / (channels - 1)
    if abs(ang_res_y - expected) > _ANGLE_TOLERANCE:
        raise AssemblyError(
            f"{where} 的 lidar.ang_res_y={ang_res_y} 与 vertical_fov_deg/(channels-1)"
            f"={expected} 不一致；ign_sim_pointcloud_tool 的环号重排会因此错位"
        )

    lidar = Lidar(
        link=_require(lidar_raw, "link", f"{where} 的 lidar"),
        sensor=_require(lidar_raw, "sensor", f"{where} 的 lidar"),
        imu_sensor=_require(lidar_raw, "imu_sensor", f"{where} 的 lidar"),
        frame=_require(lidar_raw, "frame", f"{where} 的 lidar"),
        pose=_pose(
            _require(lidar_raw, "pose", f"{where} 的 lidar"), f"{where} 的 lidar.pose"
        ),
        channels=channels,
        horizontal_samples=int(
            _require(lidar_raw, "horizontal_samples", f"{where} 的 lidar")
        ),
        horizontal_fov_deg=float(
            _require(lidar_raw, "horizontal_fov_deg", f"{where} 的 lidar")
        ),
        vertical_fov_deg=vertical_fov,
        ang_res_y=ang_res_y,
        # ang_bottom 是「下沿角度的绝对值」而不是带符号角度：上游
        # point_cloud_converter.cpp:66 用 row_id = (vertical_angle + ang_bottom) /
        # ang_res_y，传负值会让正常回波全被丢掉（decision.md D11）。
        ang_bottom=abs(float(_require(lidar_raw, "ang_bottom", f"{where} 的 lidar"))),
    )

    return Robot(
        name=_require(raw, "name", where),
        model=_require(raw, "model", where),
        sdf=_resolve(
            _require(raw, "sdf", where), f"{where} 的 sdf", share_lookup, base_share
        ),
        base_frame=_require(frames, "base", f"{where} 的 frames"),
        body_frame=_require(frames, "body", f"{where} 的 frames"),
        lidar=lidar,
    )


def load_field(path, *, share_lookup=None):
    """读场地 profile。"""
    share_lookup = share_lookup or _ament_share
    where = f"场地 profile {path}"
    raw = _clean(
        _load_yaml(path, where),
        where,
        ("name", "world", "models", "maps", "nav2_params", "acceptance"),
    )
    base_share = _base_share(path, levels=3)

    maps = _clean(
        _require(raw, "maps", where),
        f"{where} 的 maps",
        ("official", "nav", "localization"),
    )
    acceptance = _clean(
        _require(raw, "acceptance", where),
        f"{where} 的 acceptance",
        ("goal", "tunnel_rect"),
    )
    goal_raw = _clean(
        _require(acceptance, "goal", f"{where} 的 acceptance"),
        f"{where} 的 goal",
        ("x", "y", "yaw"),
    )

    rect = _require(acceptance, "tunnel_rect", f"{where} 的 acceptance")
    if not isinstance(rect, list) or len(rect) != 4:
        raise AssemblyError(
            f"{where} 的 acceptance.tunnel_rect 必须是 [x0, x1, y0, y1]：{rect!r}"
        )

    return Field(
        name=_require(raw, "name", where),
        world=_resolve(
            _require(raw, "world", where), f"{where} 的 world", share_lookup, base_share
        ),
        models_dir=_resolve(
            _require(raw, "models", where),
            f"{where} 的 models",
            share_lookup,
            base_share,
        ),
        official_map=_resolve(
            _require(maps, "official", f"{where} 的 maps"),
            f"{where} 的 maps.official",
            share_lookup,
            base_share,
        ),
        nav_map=_resolve(
            _require(maps, "nav", f"{where} 的 maps"),
            f"{where} 的 maps.nav",
            share_lookup,
            base_share,
        ),
        localization_dir=_resolve(
            _require(maps, "localization", f"{where} 的 maps"),
            f"{where} 的 maps.localization",
            share_lookup,
            base_share,
        ),
        nav2_params=_resolve(
            _require(raw, "nav2_params", where),
            f"{where} 的 nav2_params",
            share_lookup,
            base_share,
        ),
        goal=(
            float(_require(goal_raw, "x", f"{where} 的 goal")),
            float(_require(goal_raw, "y", f"{where} 的 goal")),
            float(_require(goal_raw, "yaw", f"{where} 的 goal")),
        ),
        tunnel_rect=tuple(float(value) for value in rect),
    )


def assembly_context(field, robot):
    """桥接模板可用的占位符取值。

    只暴露「换场地/换机器人会变」的量：世界名、gz 模型名、雷达链路与传感器名。
    刻意不放 ROS 话题名——那是契约自己的事，进 context 就等于鼓励在 profile 里
    拼话题（D28 第 2 条）。
    """
    return {
        "world": field.name,
        "robot": robot.model,
        "lidar_link": robot.lidar.link,
        "lidar_sensor": robot.lidar.sensor,
        "lidar_imu_sensor": robot.lidar.imu_sensor,
    }
