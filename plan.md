# plan.md — 计划与进度

> 状态：`[x]` 已完成并有可追溯回执 · `[~]` 进行中 · `[ ]` 未开始。
> 任意时刻中断后从这里恢复。

## 当前断点（2026-11-11）

## M12 ROS 运行时 namespace 统一（2026-11-12，分支 `feat/nav2-namespace`）

上游决策见 `diesel_localization` D61 与 `decision.md` D27。

- [x] 定位侧在本仓落地：`robot_namespace` launch 参数（默认 `/diesel/sentry`），
      `diesel_localization`、`diesel_map_server` 加 `namespace=`。
- [x] Nav2 整栈进同一命名空间：`PushRosNamespace` 包住 include，节点实测
      `/diesel/sentry/{bt_navigator,controller_server,velocity_smoother,...}`，
      `controller_server` 打印 `Activating controller: FollowPath of type
      regulated_pure_pursuit_controller::OmniPidPursuitController`（参数命中）。
- [x] 参数 YAML 在命名空间下重写：`RewrittenYaml(root_key=...)`，修复「兵种参数
      被静默丢弃」——修后启动行打印 `min_source_points=100 max_unobservable_dofs=3
      observability_ratio=0.050`（此前是代码默认的 `0/6/0.000`）。
- [x] 跨边界话题显式绝对名：`nav2_params_sim.yaml` 的 `odom_topic` 三处改
      `/odometry`，`bridge.yaml` 的 `ros_topic` 改 `/diesel/sentry/cmd_vel`；
      桥的 `gz_topic` 仍是 `/model/diesel_sim_robot/cmd_vel`。
- [x] `accept.py` 支持命名空间：生命周期探针与 `NavigateToPose` 走 `<ns>/...`。
- [x] TF 喂进命名空间（D28）：Nav2 节点的 tf2 监听器按相对名订阅，进 namespace 后
      落到 `/diesel/sentry/tf`（实测 5 订阅 / 0 发布），代价地图永远等不到 `odom`、
      `bt_navigator` 卡 `inactive`。修法：新增 `scripts/tf_relay.py`，把根 `/tf`、
      `/tf_static` 中继到 `<robot_namespace>/tf(_static)`，随 `PushRosNamespace` 一起
      放进 Nav2 组（`tf_relay:=false` 可关）；`/tf` 用 best_effort + volatile、
      `/tf_static` 用 reliable + transient_local，与发布端一致。
      曾试过 `SetRemap(src="tf", dst="/tf")`，**无效**——`launch` 的 remap 不随
      `IncludeLaunchDescription` 传播，子进程命令行 `grep -c "tf:=/tf"` = 0（见 D28）。
      回归用例 `test/test_navigation_sim_tf_remap.py` 注册为 ctest
      `navigation_sim_tf_remap`，断言同一 `GroupAction` 里既有 `PushRosNamespace`
      又有 `tf_relay`，且不得残留 `SetRemap`。
- [x] 起链回执（2026-11-13，宿主直跑 `navigation_sim.launch.py headless:=true`，
      `ROS_DOMAIN_ID=85`）：`ready=PASS ready_delay_s=0.94 lidar_s=0.1 localization_s=0.94
      bt_navigator=active controller_server=active last_state=OK state_reason="warmup complete"`；
      `a2 samples=5280 max_xy_err=0.034 lost=0 verdict=PASS`。
      修前一模一样的命令是 `ready=FAIL ready_delay_s=timeout bt_navigator=inactive` +
      346 条 `Invalid frame ID "odom"`。顺带说明 D27「已知未决」里 `4/6 不可观 → LOST`
      这次没有复现（`last_state=OK`、`lost=0`），点云稠密度随起链时序波动，仍需跟踪。

## M13 装配层解耦（2026-11-12，分支 `feat/sim-assembly-decoupling`）

需求见 README §1 的 A8 / A9 与 §3.1；关键取舍见 `decision.md` D29。
基线：`dev` 的 `95cb331`（含 PR #9 命名空间统一与 PR #10 TF 中继）。原分支基线 `2639d93`
未进 `dev`，故本分支 rebase 到 `dev` 后重取基线。
基线回执（S0，改写前）：`a2 max_xy_err=0.033 lost=0 verdict=PASS`；S0 时 `ready` 卡 lifecycle
（`Timed out waiting for transform` 1280 条）——该缺陷已由 PR #10 的 `tf_relay` 修掉，
修后 `ready=PASS ready_delay_s=0.01 bt_navigator=active controller_server=active`（见 M12 回执）。

- [x] I17 `diesel_sim_tools` 库包落地：`engine_gz.py`（资源路径 / 世界参数 / spawn 参数 /
      真值话题）、`bridge.py`（契约渲染）、`profile.py`（场地与机器人规格读取）+ 单测。
- [x] I18 桥接契约化：`bridge.yaml` 加 `id`、`{world}` / `{robot}` 占位、`expect`；
      渲染后残留占位符即失败并指出 `id`。
- [x] I19 profile 落盘：`config/fields/rmul_2027.yaml`、`config/robots/sentry.yaml`。
- [x] I20 `simulation.launch.py` 接装配层：世界 SDF、机器人 SDF、spawn 位姿、资源路径不再写字面量。
- [x] I21 `localization_sim.launch.py` 接装配层：雷达装位、线数、视场、帧名、manifest 目录来自 profile。
- [x] I22 `navigation_sim.launch.py` 接装配层：nav2 参数文件与派生导航图路径来自 field profile。
- [x] I23 `accept.py` 读 profile 的验收目标点与隧道窗口；`ready` 增加 A9 桥接契约自检。
- [x] I24 收尾：`tree.md` / `README` / 门禁全绿（`check_style`、`check_tree_doc`、`check_layer`、`selftest`、ctest）。

回执 A（2026-11-12，改写当期，分支基线 `2639d93`，宿主直跑）：

- 静态门禁：`check_style.py` 文件卫生通过；`check_tree_doc.py` 双向一致；`check_layer.py`
  依赖方向通过（layer=META）；`selftest.py` 5 条注入全被拦下；
  `ruff check src` All checks passed；`black --check src` unchanged。
- 单元/资产测试：`colcon test --packages-select diesel_sim_tools` 3/3；
  `diesel_simulation_bringup` 7/7（manifest / pcd_align IoU=0.9331 / nav_map / tune_* / tf）。
- 起链（`navigation_sim.launch.py headless:=true`，ROS_DOMAIN_ID=149 / GZ_PARTITION=i24c）：
  RTF `0.9996`、`/livox/lidar` 9.104 Hz、height=32 width=1875、`bridge_missing=none`（A9 通过）、
  `[Err]` 0；`ready=FAIL`（卡 lifecycle，与 S0 同一失败点）；`a2 max_xy_err=0.040 lost=0 PASS`；
  `a3` 在 `wait_ready` 处异常退出。

回执 B（2026-11-13，rebase 到 `dev` `95cb331` 之后复测，`ROS_DOMAIN_ID=155` /
`GZ_PARTITION=asm2`，宿主 load≈5 / 24 核）：

- 构建与测试：三包 `colcon build --symlink-install` 全通过；`colcon test` **12 tests /
  0 failures**（bringup 8 个含 PR #10 的 `navigation_sim_tf_remap`、field 1、tools 3）；
  `python3 -m pytest src/diesel_sim_tools/test -q` → **19 passed**；四道 CI 门禁全绿；
  `ruff` / `black` 全绿。
- `/livox/lidar` 9.992–9.996 Hz；`/diesel/sentry/tf` 的中继由 `tf_relay` 提供
  （日志 `TF 中继已启动：/tf → /diesel/sentry/tf`）；launch 日志 `[Err]` 0、
  `Invalid frame ID` 仅 4 条（中继起来之前的瞬时窗口）。
- `ready=PASS ready_delay_s=0.64 lidar_s=0.09 localization_s=0.64 bt_navigator=active
  controller_server=active last_state=OK state_reason="warmup complete" bridge_missing=none`
  ——**A9 桥接契约自检在整链上通过，且 PR #10 的 TF 修复与本次装配改写兼容**。
- `a2 samples=6789 first_ok_delay=0.00 max_xy_err=0.041 max_yaw=0.214 last_state=OK lost=0
  ground_truth=diesel_sim_robot verdict=PASS`（S0 基线 0.033，同量级）。
- `a3 result=ABORTED time_s=154.66 path_len_m=69.720 cross_samples=0 trace_samples=763
  max_xy_err=10.508 ground_truth=diesel_sim_robot verdict=FAIL`：TF 修好后 A3 首次真正
  跑到导航阶段，失败点回到**运动态定位发散**（误差 10.5 m、`Robot is out of bounds of the
  costmap!` ×21、`GridBased failed` ×12），与 2026-11 审计结论一致——属 `diesel_localization`
  / `point_lio` 跨仓问题，不在本仓职责内。

等价性判据（D29）：重写前后以「A0–A2 回执 + A3 失败指纹」比对。A2 判据相同、误差同量级
（0.041 对 0.033）；数据链、话题名、点云元数据（32×1875）、`/sim/pose_info` 真值帧名均一致；
`ready` 由 FAIL 变 PASS 是因为 rebase 带入了 PR #10 的 TF 修复，不是本改写引入的行为差异
（改写后 A9 自检与导航参数命中均可复核）。

已知阻塞（**已解决，留档**）：命名空间化后 Nav2 各节点订阅 `<ns>/tf`、`<ns>/tf_static`，
而 TF 由 `robot_state_publisher` / `body_static_tf` / `sensor_scan_generation` 发在全局
`/tf`，`controller_server` 报 `Timed out waiting for transform from base_footprint to odom ...
Invalid frame ID "odom"`，costmap 永不激活 ⇒ `ready` 卡在 lifecycle。
**该缺陷已由 PR #10（`fix/nav2-tf-relay`，commit `6bd7daa`，决策 D28）用 `tf_relay` 中继修掉**，
修后 `ready=PASS`、`a2` PASS（回执见 M12）。本重构分支 rebase 到 `dev` 后已带上该修复；
原先另开的任务 `nav2-ns-tf-remap` 随之作废（需求记录已标注作废原因）。

## M14 虚拟下位机链路（2026-11-13，分支 `feat/mcu-link-sim`）

需求见 README §1 的 A10 / A11 与 §3.2；关键取舍见 `decision.md` D30。
基线：`dev` 的 `9aba6e8`（含 PR #11 文档修正与 PR #13 技术栈归属表）。

目标：`mcu_link:=true` 时 `/cmd_vel` 经虚拟 USB CDC 交给**下位机固件源码**（`foray_mcu` 的
`host_sim` 后端），跑通 HELLO 握手、心跳、状态帧回读与 500 ms 判活，底盘速度再由过渡桥
送回 gz。默认 `false`，不影响既有验收路径。

- [ ] I25 `virtual_cdc.py`：PTY 对 + 双向中继 + 退出清理，`--echo-log` 与字节计数。
- [ ] I26 `foray_mcu` 的 `platform/host_sim/`（`hal::UsbCdc` 落到 PTY、`hal::os` 用
      `steady_clock`、其余外设复用 `platform/mock/`）+ `config/profiles/sim_dm_mc02/`
      + `firmware_sim_dm_mc02` 目标（`host-tests` 预设内）。
- [ ] I27 `simulation.launch.py` 的 `mcu_link` / `mcu_link_port` 参数与 `virtual_cdc` 装配。
- [ ] I28 `diesel_sim_mcu` 过渡桥：从机侧会话 → `/diesel/<robot>/cmd_vel`；
      `/sim/pose_info` → `GIMBAL_ATTITUDE` / `SYSTEM_STATUS` 回写；>500 ms 无帧归零。
- [ ] I29 `accept.py link` 子命令：`link_state` / `hello_rtt_ms` / `status_hz` / `frame_errors`，
      握手未完成或失联未在 500 ms 内触发即非零退出。
- [ ] I30 收尾：`tree.md` / README / 门禁全绿；整链回执（ready、a2、link、失联归零）。

回执：待补（本小节在代码落地后回填，含每轮 `ROS_DOMAIN_ID` / `GZ_PARTITION` 与原始输出）。

## M11 联调缺陷整改（2026-11-12，分支 `fix/m11-body-tf`）

`diesel_localization/plan.md` M11 的仿真侧三项已落地并取证，本 PR（#6 → `dev`）：

- **L1/L2**：`localization_sim.launch.py` 补 `base_footprint → body` 恒等静态 TF（D25），
  父方向不可反；新增静态 AST 用例 `test/test_localization_sim_launch.py` 并注册进 ctest，
  `body_tf:=false` 保留为故障注入开关。
- **L6/L7**：三项时限提成 `DeclareLaunchArgument`（默认 2.0/2.5/5.0）；`fov_horizontal_deg`
  覆盖 360°（实车 30° 会把 60000 点裁到 ~5000，再经 0.10 m 体素塌到 64~70 点）；`profile_enabled` 提成参数。
- **L8**：`accept.py` 输出 `state_reason`。

回执（宿主直跑，本仓自带 install）：`ready=PASS ready_delay_s=0.93 last_state=OK state_reason="warmup complete"`、
`a2 lost=0 max_xy_err=0.039 verdict=PASS`（整改前 `lost=5571 verdict=FAIL`）、
漏斗 `raw=18818 crop=18818 voxel=3197 align_source=1697 align_target=4137`、
`body_tf:=false` 时 `last_state=LOST reason="tf unavailable: ..."`。

**I1–I6、I9 完成（A2 已在仿真里取证通过、仓级门禁已接入），I7 代码与文档完成但
运行时判定被外部阻塞，I8 / I10 未做。M5 调车可视化（I11–I16）里 I11 / I12 / I13 / I15
已完成代码与门禁（`tune.rviz` + `rviz_config` 参数、Gazebo GUI 四面板、`tune_rqt.sh`
面板组、`tune_report.py` 健康快照），**开窗与真实频率回执仍待容器补跑**；
I16 可选未做，I14 阻塞在 `diesel_interfaces` 的相机契约。**
上述工作全部落在 `feat/i7-nav-a6`，并已**合并进 `dev`**：**PR #3 已 MERGED**
（合并提交 `99a07ae`，CI 八项全绿、`MERGEABLE/CLEAN`，base 按 AGENTS.md 为 `dev`，
原本直指 `main` 已纠正）。
`dev` 合并后做过一次 **clean rebuild 回执**（`rm -rf build install log` 后重来）：
`colcon build` 2 包通过、`colcon test-result --all` → 7 tests 0 failures、
`check_style` / `check_tree_doc`（106 文件、31 目录）/ `check_layer` 三门全绿、
`selftest.py` 5 条注入全部被拦下。
同批的 PR #4（`fix/i6-pointlio-localization` → `main`）已关闭：其提交 `19d3d0d` 早已随 PR #1
合入 `dev`，与 `dev` 零差异，属重复 PR。
`gh` 已登录（classic `repo` scope，token 存 keyring），可直接查 PR / CI / 合并。
后续改动一律从 `dev` 切新分支，不再往 `feat/i7-nav-a6` 上叠。
I6 的三处根因与取证见 `decision.md` D11/D12/D13；D9 的「回调从未被调用」已更正。
I7（导航装配、派生栅格、验收脚本、`cmd_vel` 桥接修正）见 D16/D17/D18/D20；
I9（组织规则同步）见 D14/D15；许可证见 D19。
**A3/A6 的运行时判定被 `diesel_localization` 的 GICP 后端阻塞**，
取证与待办见下方「阻塞与待确认」与 I10。

> **工作方式（2026-11-11 起）**：本仓任何编辑都先开功能分支（`feat/` `fix/` `docs/`），
> 再以 **PR 合入 `dev`**；不在 `dev` 上直接提交。`main` 只接受来自 `dev` 的稳定版本。

## M12 仿真点云稠密度整改（2026-11-12，分支 `fix/m12-cloud-density`）

承接 M11 的 L7「配准无有效对应」：`source` 侧点数不足是主因，改 `point_lio_sim.yaml`
两处下采样口径（D26）：

- `point_filter_num: 8 → 3`、`filter_size_surf: 0.2 → 0.1`，`filter_size_map` 保持 `0.2`。

回执（本机 GPU、headless、干净 `ROS_DOMAIN_ID`）：

| 指标 | 整改前 | 整改后 |
|---|---|---|
| `/cloud_registered` | 572 点 | 1352 点 |
| `GICP 无有效对应` | 每 2 s 一条 | 0 条 |
| 配准退化 | 5/6 方向不可观，condition≈1e10 | 2/6，condition≈3.3e3 |
| `accept.py ready` | FAIL / LOST | PASS / `warmup complete` |
| `accept.py a2` | FAIL `lost=5571` | **PASS** `max_xy_err=0.035`、`lost=0` |

**未解**：`accept.py a3` 仍 `ABORTED`（`path_len_m=356`、`max_xy_err=262`），
期间 `/aft_mapped_to_init` 跑到 `x=511 y=-639` —— 运动状态下 point_lio 发散；
另有一次冷启动 `52` 条 `GICP 无有效对应` 后不自恢复（首次配准失败无重试路径）。
两项均未在本分支处理。

> **验证纪律（2026-11-12 补）**：一次只跑一条链，每轮换新的 `ROS_DOMAIN_ID`。
> `pkill` 在新的 PID 命名空间里杀不到上一轮进程，旧链会与新链同时发 `/clock`，
> 把 RTF 量成 2.4（真值 1.000），据此得出的「积压/飞车」结论全是伪影。

## M1 场地资产（A0 / A5 的一半）

- [x] **I1 迁移包落盘与自检**：解包 70 文件、`verify_package.py` → `PASS: 69 package files verified`、
      `tools/test_rmul_field.py` → 5 tests OK。
- [x] **I2 新仓骨架与文档**：`README.md` / `AGENTS.md` / `tree.md` / `plan.md` / `decision.md` /
      `THIRD_PARTY_NOTICES.md` / `.diesel-layer`(META) / `.gitignore`。
- [x] **I3 场地资产移植**：`src/diesel_rmul_2027_field` 成形；世界改为 Fortress 口径
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
      容器内用真实 `diesel_localization` 验证：正例 `地图版本 1.0.0，102433 点，栅格 512x352`，
      篡改 1 字节后 `栅格 sha256 不匹配`。

## M3 机器人 / 桥 / 点云时序（A1）

- [x] **I5 机器人模型、桥、点云时序**（代码与 A1 取证完成）：
      - `models/diesel_sim_robot/model.sdf`：`base_footprint` + `chassis` + `front_mid360`，
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
      `config/bridge.yaml` 的 `cmd_vel` gz 侧话题改为 `/model/diesel_sim_robot/cmd_vel`
      （D20：错接时桥接日志与 `ros2 topic pub` 都正常、车却不动；改后 0.5 m/s 发 9 s，
      真值 x 由 6.0 变 7.35）。
      回执（代码级，2026-10-30）：`colcon test` 3/3（`manifest_consistency` / `pcd_align` / `nav_map`）、
      `ros2 pkg executables` 含 `make_nav_map.py` 与 `accept.py`、`navigation_sim.launch.py --show-args` 通过、
      整链起后 `map_server` 成功加载派生图并激活、`lifecycle_manager_navigation` 把
      controller/planner/behavior/bt_navigator/waypoint_follower/velocity_smoother 全部激活、
      `/odometry` 2.2 Hz（说明 `loam_interface`→`sensor_scan_generation` 接线生效）。
      回执（2026-11-02，`diesel_sim:humble-fortress` 容器内独立复跑）：`colcon build` 2 包通过、
      `colcon test` 4 个测试 0 failures（bringup 3 + field 的 `test_field_assets`）、
      `scripts/ci` 三门禁与 `selftest.py` 全绿、`ruff check` / `black --check` 0 待改、
      `tree.md` 与仓库 100 文件 / 31 目录双向一致。
      **未完成**：`accept.py ready` 仍是 `localization_s=none`，因为 `diesel_localization`
      在本次会话期间切到 PCL GICP 后端且无法收敛（见 I10），A3/A6 的正式判定要等它修好。
- [x] **I9 组织规则同步（D14/D15）**：补齐 `CODEOWNERS`、`.clang-format`、`pyproject.toml`、
      `.github/workflows/ci.yml` 与 `scripts/ci/{check_style,check_tree_doc,check_layer}.py`；
      现有 14 个 `.py` 通过 ruff+black，导出素材行尾统一 LF；`tree.md` 重写为逐文件镜像
      （91 文件 / 30 目录）。回执：三条门禁全绿、`ruff check` 0 问题、`black --check` 0 待改。
      后续：`gate-selftest` 已在 I7 补上（D18）；`foray_docs` 的仓表与 scope 词表尚未收录本仓。
- [ ] **I10**：解除 A3/A6 判定阻塞（外部仓 GICP，见「阻塞与待确认」）。
- [ ] **I8**：跨仓收尾——`diesel_ws/.repos` 增 `diesel_simulation`，`foray_docs`
      §6.6 加 scope 词表、`repository_structure.md` 补仓列表，`diesel_localization/decision.md`
      的 D17 一并提交。
- [ ] **I11–I16（M5 调车可视化）**：I11 / I12 / I13 / I15 代码与门禁完成（开窗与真实频率回执
        待容器），I16 可选未做、I14 阻塞在相机契约，
      逐项清单见下节
      `## M5 调车可视化`；其中 I14 会新增对外话题，须先由 `diesel_interfaces` 定契约。

## M5 调车可视化（调试支撑，不改 A0–A7 验收口径）

> 目的：在容器里调车时每一环都有窗口可看，不必退回 `ros2 topic echo` 手工盯。
> 口径：本节是**工程支撑**（与 I9 同类），不是 `README.md` 的新需求；I14 会新增对外话题，
> 必须先由 `diesel_interfaces` 定契约、再回 `README.md` 立需求。

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
      需求源」必须先由 `diesel_interfaces` 定契约（话题名、编码、`camera_info` 口径）再回
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
- `ros2 pkg executables diesel_simulation_bringup` → 6 个可执行体（含 `accept.py`、`make_nav_map.py`）。

**本轮复核不了的项**（宿主未装 `docker`；`/opt/ros/humble` 下无 `nav2*` / `ros_gz*` /
`point_lio`；`pip` 缺失）：

- A0/A1/A2/A3/A6 的运行时回执只能在 `diesel_sim:humble-fortress` 容器里重跑；
- `ruff check` / `black --check` 的回执本轮未能独立复验，仍由 CI 的 `lint-python` 作业守。

## 阻塞与待确认

- **I6 已解除阻塞**（2026-10-30）：根因是 `ang_bottom` 符号（D11）、`point_lio` 起步撞上
  Gazebo 沉降（D12）、健康时限贴着阈值抖动（D13），与 D9 推测的 executor/回调问题无关。
- **环境依赖提醒**：软件渲染下点云帧率随宿主机负载从 7 Hz 掉到 4 Hz，D13 的时限是按
  实测最坏帧间隔（p99 1.01 s、最长 1.60 s）留的余量；若换到更快的机器可再收紧。
- **A2 整链门有已知脆弱点**：残留的上一轮 `ign_sim_pointcloud_tool` 进程 + 宿主高负载
  会让它误判成 LOST（D15 有实测）。重跑前先 `ps` 确认容器干净，必要时 `docker restart`。
- **I10（新，阻塞 A3/A6 判定）**：2026-10-30 12:03 起 `diesel_localization` 分支
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
  结论：**这不是本仓能修的问题**（本仓只装配），需 `diesel_localization` 侧把 GICP 的
  质量口径与 0.25 m 体素地图对齐、或恢复可用后端；修好后在本仓执行
  `accept.py ready --deadline 300` + `accept.py a3 --x 2.0 --y 7.4` 即可补上 A3/A6 回执。
- **A1 的 IMU 口径被改动且无留痕（待确认）**：本分支 `README.md` §1 把 A1 里
  「`/livox/imu` 200 Hz 生效」改成「配置 `update_rate: 200`，实测随 RTF 缩放（软件渲染约
  156 Hz）」。`README.md` 是唯一需求源，改口径属需求变更，但 `decision.md` 没有对应条目、
  仓内也找不到 156 Hz 的实测回执，本机（无仿真环境）无法复核。处理方式二选一：
  补一条 decision 条目 + 容器内 `ros2 topic hz /livox/imu` 回执，或恢复原文措辞。
- **开发副本路径（2026-11-11 更正）**：日常开发副本是 **`~/Projects/diesel_simulation`**
  （不是此前写的 `~/Projects/RoboMaster/diesel_simulation`），`origin` 为 SSH
  `git@github.com:ZQU-Foray/diesel_simulation.git`；宿主未装 `gh`，PR 需在网页发起。
  `diesel_localization/local/staging/` 下的旧副本不再使用。
- **I14（相机 / 图像链路）是新增对外接口，需先立需求**：`model.sdf` 目前只有
  `gpu_lidar` + `imu`，`config/bridge.yaml` 只桥 5 条话题。加相机就会多出
  `/camera/image_raw`、`/camera/camera_info` 这类对外话题，`README.md` 的 A0–A7
  没有对应条目——按「`README.md` 是唯一需求源」，得先由 `diesel_interfaces` 定契约
  （话题名 / 编码 / `camera_info` 口径），再回 `README.md` 立需求，然后才动 `model.sdf`
  与 `bridge.yaml`。在此之前 I11–I13 只用现有话题即可完成。
- **可视化窗口本机开不出来（取证限制）**：宿主没有 `docker` / `rviz2` / `rqt` / `gz`，
  I11–I13、I16 的「窗口真的开出来、显示项确实有数据」这类回执只能在
  `diesel_sim:humble-fortress` 容器里、把 X11 递进去补；本轮只做到配置文件与
  `--show-args` 一级的核对。同理 I15 的频率/延迟数据也要在容器内才有意义。

## 复现命令

```bash
# 单元级（不需要仿真环境，宿主即可跑）
colcon build --packages-select diesel_rmul_2027_field diesel_simulation_bringup
colcon test  --packages-select diesel_rmul_2027_field diesel_simulation_bringup
colcon test-result --all      # 期望 0 errors 0 failures

# 仓级门禁（纯标准库）
python3 scripts/ci/check_style.py && python3 scripts/ci/check_tree_doc.py \
    && python3 scripts/ci/check_layer.py && python3 scripts/ci/selftest.py

# 仿真（需要派生镜像；宿主缺 docker / nav2 / ros_gz / point_lio 时只能跳过这两步）
docker build -t diesel_sim:humble-fortress docker/
ros2 launch diesel_simulation_bringup simulation.launch.py headless:=true

# 调车窗口：最小集（rviz/sim.rviz）
ros2 launch diesel_simulation_bringup navigation_sim.launch.py gui:=true rviz:=true

# 调车窗口：全量视图（I11，已实现；开窗回执待容器）
ros2 launch diesel_simulation_bringup navigation_sim.launch.py rviz:=true rviz_config:=tune.rviz
src/diesel_simulation_bringup/scripts/sim_up.sh --gui --tune   # 等价简写

# 调车窗口：其余入口要等 M5 后续阶段，下面几条现在跑会报参数/文件不存在
# ros2 launch diesel_simulation_bringup navigation_sim.launch.py rqt:=true                          # I13
# python3 scripts/tune_report.py                                                                   # I15
# ros2 run foxglove_bridge foxglove_bridge   # I16，浏览器开 https://studio.foxglove.dev 连接
```

窗口只能在 `diesel_sim:humble-fortress` 容器里开（宿主无 `rviz2` / `rqt` / `gz`），
需要把 X11 递进容器（`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`）。
