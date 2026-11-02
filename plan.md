# plan.md — 计划与进度

> 状态：`[x]` 已完成并有可追溯回执 · `[~]` 进行中 · `[ ]` 未开始。
> 任意时刻中断后从这里恢复。

## 当前断点（2026-11-02）

**I1–I6、I9 完成（A2 已在仿真里取证通过、仓级门禁已接入），I7 代码与文档完成但
运行时判定被外部阻塞，I8 / I10 未做。** 上述工作全部落在分支 `feat/i7-nav-a6`
（相对 `origin/dev` 共 8 个提交），本地 `dev` 已快进到 `origin/dev`（`05b6992`）。
I6 的三处根因与取证见 `decision.md` D11/D12/D13；D9 的「回调从未被调用」已更正。
I7（导航装配、派生栅格、验收脚本、`cmd_vel` 桥接修正）见 D16/D17/D18/D20；
I9（组织规则同步）见 D14/D15；许可证见 D19。
**A3/A6 的运行时判定被 `foray_localization` 的 GICP 后端阻塞**，
取证与待办见下方「阻塞与待确认」与 I10。

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
- **仓已迁至 `~/Projects/RoboMaster/foray_simulation`**（2026-10-29 起为日常开发副本，
  `remote` 用 https + gh CLI 凭据；`foray_localization/local/staging/` 下的旧副本不再使用）。

## 复现命令

```bash
# 单元级（不需要仿真环境）
colcon build --packages-select foray_rmul_2027_field foray_simulation_bringup
colcon test  --packages-select foray_rmul_2027_field foray_simulation_bringup
colcon test-result --all      # 期望 0 errors 0 failures

# 仿真（需要派生镜像）
docker build -t foray_sim:humble-fortress docker/
ros2 launch foray_simulation_bringup simulation.launch.py headless:=true
```
