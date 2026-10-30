# 三方来源与许可证

本仓包含**派生自三方作品**的资源。下列归属链必须随资源一起保留；删除或改写本文件
中的归属说明，等于同时失去使用这些资源的依据。

## 1. SMBU-PolarBear-Robotics-Team（Apache-2.0）

| 上游仓 | 本仓借鉴的内容 | 位置 |
|---|---|---|
| [`rmu_gazebo_simulator`](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator) | Gazebo 世界插件组合（physics / scene-broadcaster / user-commands / sensors + `ogre2`）、`ros_gz_bridge` 话题映射写法、机器人 spawn 流程 | `src/foray_simulation_bringup/` |
| [`pb2025_robot_description`](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description) | 机器人 SDF 宏的组织方式；Mid-360 的 `gpu_lidar` / `imu` 参数口径（32 线、`update_rate`、`samples`、range、噪声） | 通过 `xacro` 复用，未复制文件 |
| [`pb2025_sentry_nav`](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | `point_lio` 参数口径、`ign_sim_pointcloud_tool` 补 `time` 字段的必要性、Nav2 参数分节结构 | `src/foray_simulation_bringup/config/` |
| `rmoss_gz_base` / `rmoss_core` / `rmoss_interfaces` / `sdformat_tools` | 底盘驱动与 SDF→URDF 转换 | 通过依赖引用，未复制文件 |

上述仓以 **Apache License 2.0** 发布；本仓对其的引用不改变其许可证。复制片段处均在
文件头注明来源。

## 2. guodong_sentry_sim 场地迁移包

`src/foray_rmul_2027_field/` 下的网格、地图、场地复核参数与预览图来自迁移包
`RMUL2027_map_transfer_20261016_STEP_R1`。该包 `SOURCE_NOTICE.txt` 原文要点：

- 场地重建依据 **RoboMaster 2027 RMUL Rules V1.0.0（20261015）§3.2**；
- `official_top.png` 由用户提供的规则手册 PDF **Fig 3-1** 裁切；
- DJI / RoboMaster 相关图样版权归其所有者，**不暗示任何新许可证**；
- 官方 STEP 文件仅为**几何审计**目的收录。

迁移包的 `DEPENDENCIES.md` 声明的运行前提（ROS 2 Jazzy + Gazebo Harmonic）与本仓
（Humble + Fortress）不同代，本仓只取**资产**，启动链路按 Fortress 重写。

## 3. 未入库的资源

| 资源 | 未入库原因 | 获取方式 |
|---|---|---|
| `source_cad/RMUL2027_3V3_V1.0.0.stp`（1.99 MB，官方 STEP） | 版权归 RoboMaster 官方，仅用于几何审计 | 迁移包 `source_cad/`，或规则手册官方下载 |
| 上游 `pcd/simulation/*.pcd` 先验点云（`pb2025_sentry_nav`） | 上游本身不入库（网盘分发） | 本仓改为**由场地网格自行采样**，不依赖该资源 |

## 4. RoboMaster 社区 wiki

[RoboMaster 社区 wiki 仿真链路条目](https://bbs.robomaster.com/wiki/4577/714072) 仅作
**做法参考**，未复制其中代码或图片。

## 5. 本仓自身的许可

本仓自有代码与文档以 **Apache License 2.0** 发布，全文见仓根 [`LICENSE`](LICENSE)，
版权行：`Copyright 2026 ZQU-Foray`。两个 `package.xml` 的 `<license>` 字段同为
`Apache-2.0`，与 `LICENSE` 保持一致（`decision.md` D19）。

选用 Apache-2.0 的动机：上游 `pb2025_*`、`rmu_gazebo_simulator` 与 `rmoss_*` 均为
Apache-2.0，同一许可证可以逐文件搬用片段而不引入许可证混用；同时保留专利授权与
免责条款。

注意 §2 的场地迁移包与官方 STEP 文件**不在** Apache-2.0 覆盖范围内：本仓只对自有
代码与文档授权，DJI / RoboMaster 图样与规则手册内容的版权仍归其所有者。
