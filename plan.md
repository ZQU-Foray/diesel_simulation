# plan.md — 计划与进度

> 状态：`[x]` 已完成并有可追溯回执 · `[~]` 进行中 · `[ ]` 未开始。
> 任意时刻中断后从这里恢复。

## 当前断点（2026-11-11）

**I1–I6、I9 完成（A2 已在仿真里取证通过、仓级门禁已接入），I7 代码与文档完成但
运行时判定被外部阻塞，I8 / I10 未做；M5 调车可视化（I11–I16）里 I11 已实现
（`rviz/tune.rviz` + `rviz_config` 参数 + `tune_configs` 门禁），窗口开窗回执待容器补跑，
I12–I16 未开工。**
上述工作全部落在分支 `feat/i7-nav-a6`，`origin/dev` 仍停在 `05b6992`，**本分支尚未发起 PR**
（落后多少提交用 `git rev-list --count origin/dev..HEAD` 现查，不在文档里写死）。
I6 的三处根因与取证见 `decision.md` D11/D12/D13；D9 的「回调从未被调用」已更正。
I7（导航装配、派生栅格、验收脚本、`cmd_vel` 桥接修正）见 D16/D17/D18/D20；
I9（组织规则同步）见 D14/D15；许可证见 D19。
**A3/A6 的运行时判定被 `foray_localization` 的 GICP 后端阻塞**，
取证与待办见下方「阻塞与待确认」与 I10。

> **工作方式（2026-11-11 起）**：本仓任何编辑都先开功能分支（`feat/` `fix/` `docs/`），
> 再以 **PR 合入 `dev`**；不在 `dev` 上直接提交。`main` 只接受来自 `dev` 的稳定版本。

## M1 场地资产（A0 / A5 的一半）

- [x] **I1 迁移包落盘与自检**：解包 70 文件、`verify_package.py` → `PASS: 69 package files verified`、
      `tools/test_rmul_field.py` → 5 tests OK。
- [x] **I2 新仓骨架与文档**：`README.md` / `AGENTS.md` / `tree.md` / `plan.md` / `decision.md` /
      `THIRD_PARTY_NOTICES.md` / `.foray-layer`(META) / `.gitignore`。
- [x] **I3 场地资产移植**：`src/foray_rmul_2027_field` 成形；世界改为 Fortress 口径
      （`ignition-gazebo-*` 四插件 + `sensors` 带 `ogre2` + 自带 `directional` 光），容器内
      `ign model --list` 见 `rmul_2027`、话题 `/clock` 正常、日志无错误；
      `test/test_field_assets.py` 在 `colcon test` 里通过。

## M2 地图与定位契约（A4 / A5）

- [x] **I4 PCD + manifest**：
      `scripts/make_pcd.py` 三角面格点采样 0.08 m → 102433 点 / 1229370 字节；
      `scripts/make_manifest.py` 生成 `map/rmul_2027/manifest.yaml`（pcd sha256 `7915f37f…fdc775`、
      grid sha256 `cbc170a5…31801`）；
      `test/test_pcd_align.py` → **IoU 0.9331**（门槛 0.90）；
      `test/test_manifest.py` → PASS；
      容器内用真实 `foray_localization` 验证：正例 `地图版本 1.0.0，102433 点，栅格 512x352`，
      篡改 1 字节后 `栅格 sha256 不匹配`。

## M3 机器人 / 桥 / 点云时序（A1）

- [x] **I5 机器人模型、桥、点云时序**（代码与 A1 取证完成）：
      - `models/foray_sim_robot/model.sdf`：`base_footprint` + `chassis` + `front_mid360`，
        `gpu_lidar`（1875×32、-7°…+52°、`update_rate` 10）+ `imu`（200 Hz），
        平面运动用 `ignition-gazebo-velocity-control-system`（不建轮系）。
      - `config/bridge.yaml`：`/clock`、`/livox/lidar`、`/livox/imu`、`/cmd_vel`、`/sim/pose_info`。
      - `launch/simulation.launch.py`：`ros_gz_sim` + `create` + `parameter_bridge` +
        `robot_state_publisher` + 可选 rviz/gui；`scripts/sim_up.sh` 是它的薄封装。
      - A1 实测（`osrf/ros:humble-desktop-full`，headless，软件渲染）：话题全部出现；
        点云 `height=32`、`width=1875`、`point_step=32`、字段 `x/y/z/intensity/ring`；
        `/velodyne_points` 7.85 Hz、`/livox/lidar` 约 7 Hz，与 RTF ≈ 0.7 一致（见 D8）。

## M4 定位 / 寻路 / 一键（A2 / A3 / A6）

- [x] **I6 定位链路**（A2，见 D11/D12/D13）：`config/point_lio_sim.yaml`、
      `launch/localization_sim.launch.py`、`scripts/set_initial_pose.py` 打通。
      三处修复：`ang_bottom` 传 `7.0`（D11）、`point_lio` 用 `TimerAction` 延后 4 s 启动（D12）、
      仿真侧健康时限放宽为 `odom 2.0 / lidar 2.5 / lost 5.0`（D13）；另修 4 个脚本的可执行位。
      回执：默认参数 95 s 采样 4737 条状态 ⇒ `{INITIALIZING: 314, OK: 4423}`、零 DEGRADED/LOST；
      首次 OK 在仿真 3.78 s，位姿 `(6.0606, 4.0425)`，与真值 `(6.0, 4.0, 0.011)` 相差
      **0.083 m**，`yaw=0.012°`；`/aft_mapped_to_init` 7.1 Hz、静止漂移毫米级。
- [~] **I7 导航装配与整链（A3 / A6，见 D16/D17/D18）**：代码与文档已落盘，运行时判定待外部修复。
      已完成：`scripts/make_nav_map.py` + `map/nav2/rmul_2027_nav.{pgm,yaml}`（从官方图派生，
      只清隧道顶投影 462 格，`test_nav_map.py` 断言差异有界与通道宽度 ≥ 0.70 m）；
      `config/nav2_params_sim.yaml`（omni 控制器、`inflation_radius 0.20`、footprint 四点、
      res 0.025/0.05、只用静态层）；`launch/navigation_sim.launch.py`（`loam_interface` +
      `sensor_scan_generation` 补 `odom→base_footprint`，再 TimerAction 起 map_server 与
      `nav2_bringup/navigation_launch.py`）；`scripts/sim_up.sh` 默认起整链；
      `scripts/accept.py` 三条子命令（`ready` / `a2` / `a3`）；
      `config/bridge.yaml` 的 `cmd_vel` gz 侧话题改为 `/model/foray_sim_robot/cmd_vel`
      （D20：错接时桥接日志与 `ros2 topic pub` 都正常、车却不动；改后 0.5 m/s 发 9 s，
      真值 x 由 6.0 变 7.35）。
      回执（代码级，2026-10-30）：`colcon test` 3/3（`manifest_consistency` / `pcd_align` / `nav_map`）、
      `ros2 pkg executables` 含 `make_nav_map.py` 与 `accept.py`、`navigation_sim.launch.py --show-args` 通过、
      整链起后 `map_server` 成功加载派生图并激活、`lifecycle_manager_navigation` 把
      controller/planner/behavior/bt_navigator/waypoint_follower/velocity_smoother 全部激活、
      `/odometry` 2.2 Hz（说明 `loam_interface`→`sensor_scan_generation` 接线生效）。
      回执（2026-11-02，`foray_sim:humble-fortress` 容器内独立复跑）：`colcon build` 2 包通过、
      `colcon test` 4 个测试 0 failures（bringup 3 + field 的 `test_field_assets`）、
      `scripts/ci` 三门禁与 `selftest.py` 全绿、`ruff check` / `black --check` 0 待改、
      `tree.md` 与仓库 100 文件 / 31 目录双向一致。
      **未完成**：`accept.py ready` 仍是 `localization_s=none`，因为 `foray_localization`
      在本次会话期间切到 PCL GICP 后端且无法收敛（见 I10），A3/A6 的正式判定要等它修好。
- [x] **I9 组织规则同步（D14/D15）**：补齐 `CODEOWNERS`、`.clang-format`、`pyproject.toml`、
      `.github/workflows/ci.yml` 与 `scripts/ci/{check_style,check_tree_doc,check_layer}.py`；
      现有 14 个 `.py` 通过 ruff+black，导出素材行尾统一 LF；`tree.md` 重写为逐文件镜像
      （91 文件 / 30 目录）。回执：三条门禁全绿、`ruff check` 0 问题、`black --check` 0 待改。
      后续：`gate-selftest` 已在 I7 补上（D18）；`foray_docs` 的仓表与 scope 词表尚未收录本仓。
- [ ] **I10**：解除 A3/A6 判定阻塞（外部仓 GICP，见「阻塞与待确认」）。
- [ ] **I8**：跨仓收尾——`foray_ws/.repos` 增 `foray_simulation`，`foray_docs`
      §6.6 加 scope 词表、`repository_structure.md` 补仓列表，`foray_localization/decision.md`
      的 D17 一并提交。
- [ ] **I11–I16（M5 调车可视化）**：I11 代码与门禁完成（开窗回执待容器），I12–I16 未开工，
      逐项清单见下节
      `## M5 调车可视化`；其中 I14 会新增对外话题，须先由 `foray_interfaces` 定契约。

## M5 调车可视化（调试支撑，不改 A0–A7 验收口径）

> 目的：在容器里调车时每一环都有窗口可看，不必退回 `ros2 topic echo` 手工盯。
> 口径：本节是**工程支撑**（与 I9 同类），不是 `README.md` 的新需求；I14 会新增对外话题，
> 必须先由 `foray_interfaces` 定契约、再回 `README.md` 立需求。

现状（2026-11-11 勘察）：能给的窗口只有两个——Gazebo GUI（`config/gui.config` 里只有
`MinimalScene` + `GzSceneManager`，纯 3D 场景）和 RViz2（`rviz/sim.rviz` 只有 `TF` /
`/livox/lidar` / `/map`，俯视 `TopDownOrtho`）。`rqt` 在 launch、`package.xml`、
`docker/Dockerfile` 里都没有出现；`model.sdf` 只有 `gpu_lidar` + `imu`，没有相机。

- [ ] **I11 调车版 RViz 视图**（`rviz/tune.rviz` + `rviz_config:=` 启动参数）：验收最小集
      `sim.rviz` 保持不动，另出一份全量视图：
      - `RobotModel`（`/robot_description`，`robot_state_publisher` 已经起了）；
      - `TF` 打开 `Show Names`，一眼看出 `map→odom→base_footprint→front_mid360` 断在哪；
      - `/cloud_registered`（`loam_interface` 输出）与 `/lidar_odometry`；
      - `/map` 与派生导航栅格 `rmul_2027_nav` 叠看（D16 的清隧道要在图上可验）；
      - `local_costmap/costmap`、`global_costmap/costmap`、`/local_costmap/published_footprint`；
      - `/plan`（全局）与 `/local_plan`（局部），以及 `/odometry`（先只放朝向箭头）；
      - 轨迹：RViz2 的 `Odometry` display 能否保留历史位姿（`Keep`）需在容器里实测，
        能才开历史轨迹；否则轨迹交给 I15 的文本报告，不承诺 RViz 里的累积轨迹；
      - 工具：`SetInitialPose`（`/initialpose`）、`SetGoal`（`/goal_pose`）、
        `PublishPoint`（`/clicked_point`）、`Measure`（量隧道净空）；
      - 两套视图预设：`TopDownOrtho`（对图、发目标）与 `Orbit`（看隧道顶与场地高差）。
      - 状态（2026-11-11）：代码完成——`rviz/tune.rviz`、三层 `rviz_config` 透传、
        `sim_up.sh --tune`、门禁 `test/test_tune_configs.py`（话题白名单 + 最小集不许改）
        均已落在 `feat/i7-nav-a6`。**开窗、每项显示是否有数据、`/map` 是否双发布、
        `Odometry` 是否支持 `Keep` 仍待容器回执**，故本项暂不勾选。
- [ ] **I12 Gazebo GUI 补面板**（`config/gui.config`）：现在只有 `MinimalScene` +
      `GzSceneManager`。补 `WorldControl`（暂停/单步，调车时定住一帧）、`EntityTree`、
      `ComponentInspector`、`WorldStats`（可选再补 `TransformControl`、`ViewAngle`）；该文件由
      `simulation.launch.py` 用 `--gui-config` 传给 `gz sim`，改一份即对全链生效。
      插件类名以容器内 `ign gui -l` 的真实输出为准（候选名单按上游 `ign-gui6` 与
      `ign-gazebo6` 的插件目录核对得出），类名写错时 gz 会静默丢掉该面板、窗口里什么都不报。
      - 状态（2026-11-11）：代码完成——四个面板已写进 `config/gui.config`，各自带绝对
        `x/y`；`WorldControl` 显式 `start_paused=false`（否则世界启动即暂停，全链无数据）；
        `test/test_tune_configs.py` 断言「必需面板集合 / 浮窗必须有 x/y / start_paused 为
        false」，并做过注入 `start_paused=true` 的反向验证。**面板是否真的出现、布局是否
        压到机器人、`ign gui -l` 的名字是否与 `filename` 一致仍待容器回执**，故不勾选。
- [ ] **I13 rqt 面板组**（`rqt:=true` + `scripts/tune_rqt.sh` 启动器）：一次开出
      `rqt_graph`（谁连谁）、`rqt_topic`（关键话题频率）、`rqt_plot`（`/cmd_vel` 与
      `/odometry` 速度对照，调控制器先看这张）、`rqt_console`（过滤 `ros_gz_bridge` 噪声）、
      `rqt_tf_tree`、`rqt_image_view`（等 I14）。启动器逐个检查面板是否可用，缺哪个就打印
      对应的 `ros-humble-rqt-*` 包名再继续拉起其余面板（不整组失败），Ctrl-C 统一收摊。
      - 状态（2026-11-11）：代码完成——`scripts/tune_rqt.sh`（缺面板打印 `ros-humble-rqt-*`
        包名后继续，一个都没有才非零退出，`trap` 统一收摊）、`navigation_sim.launch.py` 的
        `rqt:=true`、`sim_up.sh --rqt`、门禁 `test/test_tune_rqt.py`（语法 / 面板清单 /
        缺包行为 / 透传与 `--no-nav` 收敛）均已落地；`package.xml` 按 D23 不加 rqt 依赖。
        **面板真实弹出与曲线有数据仍待容器回执**，故不勾选。
      2026-11-11 勘察修正：**不再盲写 `config/rqt/tune.perspective`**——perspective 是 Qt 的
      ini 布局快照、靠 GUI 保存产生，凭空手写极可能开出一片空窗；只有在容器里人工保存的
      perspective 被验证可用后，才可选入库。基础镜像是 `osrf/ros:humble-desktop-full`，
      rqt 大概率自带；缺则把 `ros-humble-rqt*` 补进 `docker/Dockerfile` 并在回执写清。
- [ ] **I14 相机 / 图像链路（接口决策点，不阻塞 I11–I13）**：要在窗口里看点云以外的画面，
      需 `model.sdf` 加 `camera` 传感器、`bridge.yaml` 加 `/camera/image_raw` 桥、RViz 加
      `Image` 显示。这是**新增对外话题**，超出 `README.md` 现有 A0–A7，按「README 是唯一
      需求源」必须先由 `foray_interfaces` 定契约（话题名、编码、`camera_info` 口径）再回
      README 立需求；在此之前只规划、不动手。
- [ ] **I15 话题健康快照**（`scripts/tune_report.py`）：把调车时反复手打的命令收成一个——
      关键话题实测频率与帧间隔、`/localization/state` 的健康字段（`accept.py` 已订阅该契约）、
      TF 链完整性、`/cmd_vel` 指令与 `/sim/pose_info` 真值对照；默认人读表格，`--json` 给机器。
      不依赖仿真也能跑（空数据路径自检），并纳入 `colcon test` 回归。
      - 状态（2026-11-11）：代码完成——统一窗口采样六路话题 + TF 链 + `/cmd_vel` vs 真值，
        `--json` 顶层键固定，顶层不 import rclpy；门禁 `test/test_tune_report.py`
        （10 Hz 合成 / 2.0 s 边界判 stale / 空序列不补零 / JSON 契约）已接入 `colcon test`。
        修正一处早期误判：`/sim/pose_info` 是 `tf2_msgs/TFMessage`、
        `LocalizationState` 用 `state` 常量而非 `status` 字段。
        **真实频率数值与真值偏差量级仍待容器回执**，故不勾选。
- [ ] **I16 浏览器路线（可选，无 X11 时的替代）**：`foxglove_bridge`（Humble 有
      `ros-humble-foxglove-bridge`）→ Foxglove Studio 网页看 3D / 曲线 / 图像，还能顺手录
      bag 复看。与 `use_sim_time` 的兼容性、网页端 `PointCloud2` 带宽都需实测；真要用就在
      `docker/Dockerfile` 装包，并把「rqt 还是 foxglove」的取舍写进 `decision.md`。

外部参考（2026-11-11 勘察）：

- 上游 `rmu_gazebo_simulator` 的 `rviz/visualize.rviz`：`RobotModel` + `TF` + `LaserScan` +
  `PointCloud2`(`/livox/lidar`) + `Camera`，工具位带 `SetInitialPose` / `SetGoal` /
  `PublishPoint`——I11 的工具集照它对齐：
  <https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator>
- `pb2025_sentry_nav` 的 `pb2025_nav_bringup/rviz/nav2_default_view.rviz`：Nav2 官方视图，
  costmap / plan 的显示项照它抄：
  <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>
- `rmu_gazebo_simulator/scripts/player_web/`：Flask 网页面板（`ros_handler.py` 发布底盘 /
  云台 / 发射指令），I16 网页路线的同类实现；
- `foxglove_bridge` 与 `rqt_graph` 在 Humble 的可用性已核对：
  <https://index.ros.org/p/foxglove_bridge/>、<https://index.ros.org/p/rqt_graph/>。

## 2026-11-11 完成度审计

审计基线：`README.md` §1 的 A0–A7（唯一需求源）+ 本文件 I1–I16（计划）。`dev`（`05b6992`）
与本分支各复跑一次，结论如下。

| 层面 | 结论 |
|---|---|
| 需求 A0–A7 | **6/8 有回执**：A0/A1/A2/A4/A5/A7 通过；**A3/A6 无 PASS 回执**（I10 未解） |
| 计划 I1–I10 | **完成 7、进行中 1、未做 2**：I1–I6、I9 完成；I7 代码完成、判定阻塞；I8/I10 未做 |
| 计划 I11–I16（M5） | **I11 已实现**（`tune.rviz` / `rviz_config` / `tune_configs` 门禁，
宿主回执齐全；开窗与 `/map` 发布者待容器核）；I12–I16 未开工 |
| `dev` 分支 | 落后本分支若干提交（`git rev-list --count origin/dev..HEAD` 现查），且其 `plan.md` 仍写 I7 未开始 ⇒ 从 `dev` 恢复会重做 I7；本分支以 PR 对齐 |

本机复跑（`colcon` + 标准库脚本，不依赖容器）：

- `colcon build` 2 包通过；`colcon test` → `dev` 3 tests、本分支 4 tests，均 0 failures；
- `scripts/ci/{check_style,check_tree_doc,check_layer}.py` 三门全绿（本分支 100 文件 / 31 目录）；
- `scripts/ci/selftest.py` → 5 条注入全部被拦下，干净副本三门全绿；
- `test_pcd_align.py` 复算 **IoU 0.9331**（召回 0.9777 / 精确 0.9534）、`test_manifest.py` PASS；
- `ros2 pkg executables foray_simulation_bringup` → 6 个可执行体（含 `accept.py`、`make_nav_map.py`）。

**本轮复核不了的项**（宿主未装 `docker`；`/opt/ros/humble` 下无 `nav2*` / `ros_gz*` /
`point_lio`；`pip` 缺失）：

- A0/A1/A2/A3/A6 的运行时回执只能在 `foray_sim:humble-fortress` 容器里重跑；
- `ruff check` / `black --check` 的回执本轮未能独立复验，仍由 CI 的 `lint-python` 作业守。

## 阻塞与待确认

- **I6 已解除阻塞**（2026-10-30）：根因是 `ang_bottom` 符号（D11）、`point_lio` 起步撞上
  Gazebo 沉降（D12）、健康时限贴着阈值抖动（D13），与 D9 推测的 executor/回调问题无关。
- **环境依赖提醒**：软件渲染下点云帧率随宿主机负载从 7 Hz 掉到 4 Hz，D13 的时限是按
  实测最坏帧间隔（p99 1.01 s、最长 1.60 s）留的余量；若换到更快的机器可再收紧。
- **A2 整链门有已知脆弱点**：残留的上一轮 `ign_sim_pointcloud_tool` 进程 + 宿主高负载
  会让它误判成 LOST（D15 有实测）。重跑前先 `ps` 确认容器干净，必要时 `docker restart`。
- **I10（新，阻塞 A3/A6 判定）**：2026-10-30 12:03 起 `foray_localization` 分支
  `feat/pcl-gicp-aligner` 的提交 `5234023` 把配准后端换成 PCL GICP，容器内镜像也已被
  并行会话重建。实测该后端在本场地**不收敛**：
  - 真值位姿 (6.0, 4.0) 下对 0.25 m 体素先验图的最近邻残差 `mean=0.1069 m`、
    `inlier(<0.10)=0.437`，而 `ok_mean_residual_m=0.10`、`ok_inlier_ratio=0.55`
    ⇒ 健康门先天不满足（阈值与地图粒度不自洽）；
  - 运行时把 `ok_mean_residual_m` 放宽到 0.30、`ok_inlier_ratio` 放宽到 0.05、
    `odom_timeout_sec`/`lidar_timeout_sec` 放宽到 5 s、`align_rate_hz` 降到 2 Hz
    后，`/localization/state` 仍是 `state: 3`（LOST）、`reason: initialization timeout`；
  - 同条件下 `point_lio` 只有 1.3 Hz（`/livox/lidar` 6.7 Hz），GICP 常驻 41% CPU，
    宿主 load 41 / 12 核导致 RTF 塌陷，也会单独触发 `odom_timeout_sec`。
  结论：**这不是本仓能修的问题**（本仓只装配），需 `foray_localization` 侧把 GICP 的
  质量口径与 0.25 m 体素地图对齐、或恢复可用后端；修好后在本仓执行
  `accept.py ready --deadline 300` + `accept.py a3 --x 2.0 --y 7.4` 即可补上 A3/A6 回执。
- **A1 的 IMU 口径被改动且无留痕（待确认）**：本分支 `README.md` §1 把 A1 里
  「`/livox/imu` 200 Hz 生效」改成「配置 `update_rate: 200`，实测随 RTF 缩放（软件渲染约
  156 Hz）」。`README.md` 是唯一需求源，改口径属需求变更，但 `decision.md` 没有对应条目、
  仓内也找不到 156 Hz 的实测回执，本机（无仿真环境）无法复核。处理方式二选一：
  补一条 decision 条目 + 容器内 `ros2 topic hz /livox/imu` 回执，或恢复原文措辞。
- **开发副本路径（2026-11-11 更正）**：日常开发副本是 **`~/Projects/foray_simulation`**
  （不是此前写的 `~/Projects/RoboMaster/foray_simulation`），`origin` 为 SSH
  `git@github.com:ZQU-Foray/foray_simulation.git`；宿主未装 `gh`，PR 需在网页发起。
  `foray_localization/local/staging/` 下的旧副本不再使用。
- **I14（相机 / 图像链路）是新增对外接口，需先立需求**：`model.sdf` 目前只有
  `gpu_lidar` + `imu`，`config/bridge.yaml` 只桥 5 条话题。加相机就会多出
  `/camera/image_raw`、`/camera/camera_info` 这类对外话题，`README.md` 的 A0–A7
  没有对应条目——按「`README.md` 是唯一需求源」，得先由 `foray_interfaces` 定契约
  （话题名 / 编码 / `camera_info` 口径），再回 `README.md` 立需求，然后才动 `model.sdf`
  与 `bridge.yaml`。在此之前 I11–I13 只用现有话题即可完成。
- **可视化窗口本机开不出来（取证限制）**：宿主没有 `docker` / `rviz2` / `rqt` / `gz`，
  I11–I13、I16 的「窗口真的开出来、显示项确实有数据」这类回执只能在
  `foray_sim:humble-fortress` 容器里、把 X11 递进去补；本轮只做到配置文件与
  `--show-args` 一级的核对。同理 I15 的频率/延迟数据也要在容器内才有意义。

## 复现命令

```bash
# 单元级（不需要仿真环境，宿主即可跑）
colcon build --packages-select foray_rmul_2027_field foray_simulation_bringup
colcon test  --packages-select foray_rmul_2027_field foray_simulation_bringup
colcon test-result --all      # 期望 0 errors 0 failures

# 仓级门禁（纯标准库）
python3 scripts/ci/check_style.py && python3 scripts/ci/check_tree_doc.py \
    && python3 scripts/ci/check_layer.py && python3 scripts/ci/selftest.py

# 仿真（需要派生镜像；宿主缺 docker / nav2 / ros_gz / point_lio 时只能跳过这两步）
docker build -t foray_sim:humble-fortress docker/
ros2 launch foray_simulation_bringup simulation.launch.py headless:=true

# 调车窗口：最小集（rviz/sim.rviz）
ros2 launch foray_simulation_bringup navigation_sim.launch.py gui:=true rviz:=true

# 调车窗口：全量视图（I11，已实现；开窗回执待容器）
ros2 launch foray_simulation_bringup navigation_sim.launch.py rviz:=true rviz_config:=tune.rviz
src/foray_simulation_bringup/scripts/sim_up.sh --gui --tune   # 等价简写

# 调车窗口：其余入口要等 M5 后续阶段，下面几条现在跑会报参数/文件不存在
# ros2 launch foray_simulation_bringup navigation_sim.launch.py rqt:=true                          # I13
# python3 scripts/tune_report.py                                                                   # I15
# ros2 run foxglove_bridge foxglove_bridge   # I16，浏览器开 https://studio.foxglove.dev 连接
```

窗口只能在 `foray_sim:humble-fortress` 容器里开（宿主无 `rviz2` / `rqt` / `gz`），
需要把 X11 递进容器（`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`）。
