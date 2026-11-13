# decision.md — 关键工程决策

> 只记「改变了实现方向」的决策。每条都要有当时的理由，避免后人重复讨论。

## D1 仿真器口径：Ignition Gazebo Fortress（gz-sim 6），不是 Harmonic

- 参考的 SMBU 系仿真仓（`rmu_gazebo_simulator`）就是 **Humble + Fortress + `gz_version: 6`**，
  上游现成的世界插件组合、`ros_gz_bridge` 话题映射可直接照抄。
- 容器环境实测：`osrf/ros:humble-desktop-full` 自带 ignition-gazebo 6.18、`ros_gz_bridge`、
  `ros_gz_sim`，**无需自建镜像**即可起仿真。
- guodong 迁移包声明的是 Jazzy + Harmonic，**本仓只取它的资产**（网格/地图/复核参数），
  世界文件按 Fortress 重写（插件名前缀 `ignition-gazebo-*`，SDF 1.9 在 sdformat12 下可加载，
  已实测）。
- 代价：Fortress 的 `gpu_lidar` 需要显式加载 `ignition-gazebo-sensors-system`（带 `ogre2`），
  否则**静默不发数据**；世界必须自带 `<light>` 与地面，镜像里没有 `model://sun`。

## D2 官方 STEP 不入库

`source_cad/RMUL2027_3V3_V1.0.0.stp`（1.99 MB）版权归 RoboMaster 官方、仅用于几何审计，
体积也不适合进仓。只保留由它派生的**分件 DAE** 与复核参数（`maps/rmul_2027/*.json`），
获取方式写进 `THIRD_PARTY_NOTICES.md`。

## D3 先验点云由网格自行采样，不用上游网盘 PCD

`pb2025_sentry_nav` 的先验 PCD 不在 git（网盘分发），且是别的赛季场地。本仓改为
`make_pcd.py` 从同包网格表面采样：

- 只用顶点会得到 **166 点**（每个分件 36 个顶点 = 12 个三角形），GICP 完全不够用
  → 必须做**三角面内格点采样**；
- 采样间距 **0.08 m**：5 cm 时 260898 点 / 3.1 MB 过于臃肿，8 cm 得 102433 点 / 1.2 MB，
  且在 2.5 cm 栅格上仍能覆盖围挡全表面；
- 输出为**定长头 + binary** PCD，字段按字典序排列，保证同输入逐字节同哈希（sha256 可复现）。

## D4 A5 的 IoU 判定口径

两条数据链必须互相独立，断言才不是自证：占据图取官方 PGM，点云由网格采样。

- **点云占据**：栅格内存在 `z > 0.02 m` 的点。2 cm 是底盘可通行分界（中央控制区 10 mm、
  补给垫 3 mm 可通行；围挡 420 mm、高地 200 mm、工作台 403 mm、隧道顶 270 mm 记占据）。
- **比较范围**：外场矩形 `(-0.3, -0.3) – (12.3, 8.3)`（内场 12×8 + 0.3 m 围挡）。官方 PGM
  把场外 0.1 m 余量也标成占据，点云不含该区域，不排除会系统性拉低约 12% IoU。
- **表面膨胀**：点投影按采样半径 `spacing/2 = 0.04 m` 膨胀，否则 0.3 m 厚围挡上只留 7 行点，
  IoU 会掉到 0.22（首轮实测）。
- 结果：**IoU 0.9331 / 召回 0.9777 / 精确 0.9534**，门槛 0.90。

## D5 manifest 与栅格/PCD 同目录

`diesel_localization` 的地图加载按 `manifest 目录 / 字段值` 定位文件，`pcd_file` / `grid_file`
必须是相对路径且不含 `..`。因此地图目录固定为 `map/rmul_2027/`，`make_manifest.py` 只负责
读 nav2 yaml 的 `image/resolution/origin`、读 PGM 尺寸、算两个 sha256，**不猜、不手填**。

## D6 只交付一个 canonical 世界

迁移包提供 `rmul_2027.sdf` / `_preview` / `_native` / `_native_preview` 四个变体。调试变体
（内联模型、preview 机器人）不入库：世界只有一份，差异靠 launch 参数与 `visual_quality`
表达，避免四份世界各自漂移。

## D7 本仓层级为 META

仿真属于工具链，不在算法六层里。`.diesel-layer` 声明 `layer: META`、`depends_on_layers: []`，
与 `diesel_ws` 一致；它**运行时**依赖 `diesel_localization` 的可执行文件，但不链接其代码，
因此不构成层级依赖。

## D8 点云与 IMU 的仿真口径

- **线数与视场**：`<vertical><samples>32</samples>-0.12217304764…0.90757121104</vertical>`
  （-7°…+52°）、`<horizontal><samples>1875</samples>±π</horizontal>`，取自
  `pb2025_robot_description` 的 `resource/models/mid360/model.sdf.xmacro`（上游 pin 注解见
  `THIRD_PARTY_NOTICES.md`）。Fortress 的 `gpu_lidar` 支持 32 线，实测 `height=32`、
  `width=1875`、`point_step=32`，字段 `x/y/z/intensity(FLOAT32)/ring(UINT16)`。
- **nominal 10 Hz 而非 20 Hz**：Livox MID-360 默认出点率 10 Hz；`update_rate=20` 只会让渲染
  线程更吃力而不增加信息量。软件渲染（`LIBGL_ALWAYS_SOFTWARE=1`）下实测 6.8–7.2 Hz，
  与 RTF ≈ 0.7 一致；GPU 直通环境应达 10 Hz。因此 A1 的门槛按 normal 10 Hz 写，
  CI 只保证 ≥ 6.5 Hz。
- **IMU 必须靠 `imu-system`**：世界只加载 `sensors-system` 时 `gpu_lidar` 正常、`imu` 话题
  却根本不出现（连 `ign topic -l` 都查不到，`ros_gz_bridge` 也不报错——它按需懒订阅）。
  世界必须同时加载 `ignition-gazebo-imu-system`。`test_field_assets.py` 已把这条写成断言。
- **机器人本体不建轮系**：用 `ignition-gazebo-velocity-control` 平面速度控制，Nav2 直接给
  `/cmd_vel`；真实底盘的运动学差异不在「点云→定位→寻路」这条链路的验证范围内。

## D9 I6 暂停：上游 point_lio 在本仿真里收不到数据

**现象**：`point_lio` 进程存活、`ros2 topic info` 显示它订阅了 `/velodyne_points`（BEST_EFFORT）
与 `/livox/imu`、也是 `/aft_mapped_to_init` 的发布者，但插桩后 `sync_packages`
一秒不停打印 `lidar_buf=0 imu_deque=0 imu_en=1 last_imu=-1.000`——**两个回调从未被调用**，
`/aft_mapped_to_init` 一帧都没有，`diesel_localization` 因此 `pose_valid=false`、A2 无从取证。
同一时刻同一个容器里，`diesel_localization` 的 `map_server_node` 能正常加载地图、
python 探针能收到同两条话题的数据。

**已排除**（每条都实测过，回执见 `local/evidence-sim/i6-point-lio-blocker.txt`）：
- 节点名未 remap → 直接 segfault：`ros2 run point_lio pointlio_mapping` 时 YAML 的
  `point_lio:` 段不生效，`prior_pcd.enable` 回默认真值、`prior_pcd_map_path` 为空字符串，
  加载空路径崩溃。**跑 point_lio 必须 `-r __node:=point_lio`**（或走 launch）。
- 参数未生效：`ros2 param get` 逐项确认 `common.lid_topic=velodyne_points`、
  `common.imu_topic=livox/imu`、`preprocess.lidar_type=2`、`use_sim_time=True`、
  `prior_pcd.enable=False`。
- 点云里的 `inf`：仿真顶棚开放，约 79% 回波是 ±inf；用一个只保留有限点的调试节点过滤后
  再喂给它，**仍然没有回调**。
- 执行器：把 `executor.spin_some()` 换成 `rclcpp::spin_some(nh)` 报
  `Node '/point_lio' has already been added to an executor`（证明节点确实在 executor 里），
  换成「专用线程 `executor.spin()`」也仍然是零回调。

**三条备选路线**（下一步从这里挑一条，不在本次范围内）：
1. 继续查上游（可能是 RMW 事件队列 / `spin_some` 在 Humble 多线程 executor 上的行为，
   或需要打成 component 交给容器 spin）。
2. 换里程计源：`rmoss_gz_base` 或 `ignition-gazebo-odometry-publisher-system` 出 odom。
   代价是 A2 退化成「真值输入 + GICP 微调」，验收力度变弱。
3. 路线 2 + 一个几十行的漂移注入节点，让 GICP 真的需要纠偏，A2 仍有效。

**本次决定**：先停在这里，把 I1–I5 交付干净（资产 / PCD / manifest / 机器人 / 桥 / 点云时序），
I6–I8 留待下一轮；`localization_sim.launch.py` 与 `point_lio_sim.yaml` 保留在树里，
它们是下一个接手者直接可用的起点。

## D10 摆位必须用 MODE_GEOMETRIC

`diesel_localization` 的 `/localization/relocalize` 服务有两个模式：`MODE_GEOMETRIC=0`、
`MODE_PLACE=1`。本仓 `scripts/set_initial_pose.py` 必须用 **MODE_GEOMETRIC**：用 MODE_PLACE
会被拒（`place 后端默认关闭（D10）；请使用 MODE_GEOMETRIC 给定初值`），仿真里没有 place 后端。
服务名是**绝对名** `/localization/relocalize`（不是 `/diesel_localization/relocalize`）。

## D11 `ign_sim_pointcloud_tool` 的 `ang_bottom` 必须传正数（下沿角绝对值）

**现象**：I6 卡在「`/velodyne_points` 有数据、`point_lio` 回调也不报错，但
`lidar_buffer` 恒空」。插桩显示回调其实每秒 8 次正常进入（`lidar_cbk #421`），
`imu_deque` 只进不出，`/velodyne_points` 有 50015 点却**全部是 `inf`**，
`velodyne_handler` 出来 `pl_surf=0`，于是 `sync_packages` 永远凑不齐一帧。

**根因**：上游 `pb2025_sentry_nav/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp`
用 `row_id = int((vertical_angle + ang_bottom_) / ang_res_y_)` 反算线号，`ang_bottom_` 的语义是
**下沿角的绝对值**——上游默认 `declare_parameter<float>("ang_bottom", 7.0)`，
`pb2025_nav_bringup/config/simulation/nav2_params.yaml` 也传 `7.0`。本仓原先传 `-7.0`，
算成 `row_id = (v - 7)/1.903`：真实回波（-7°…+52°）全被判 `continue` 丢掉，只剩
`atan2(inf, inf) = +45°` 那些点恰好落在 `row_id=19`，输出就变成「50015 个 `inf` 点」。

**决定**：`localization_sim.launch.py` 里用常量 `ANG_BOTTOM = "7.0"` 并就地注释语义，
不再允许传负值；`ANG_RES_Y` 保持 `1.903225806451613`（与 32 线、竖直视场 59° 的换算一致）。

**回执**：修后 `/velodyne_points` 恢复 `width=60000`，插桩 `velo_out pl_surf=1249`，
`/aft_mapped_to_init` 7.3 Hz。

## D12 `point_lio` 必须等 Gazebo 沉降结束再启动

**现象**：D11 修好后里程计能出，但**一启动就发散**——静止机器人在 40 s 时
`/aft_mapped_to_init` 已经跑到 `x=310 m / z=-1201 m`，53 s 到 `x=-759 m / z=-6506 m`，
70 s 量级到了 12 km；`diesel_localization` 用它做先验运动的 guess，ICP 直接 0 对应
（插桩 `guess_t=(152449, 344122, -3070468)`），看起来像「配准坏了」，其实是先验炸了。

**定位**：把同一份仿真**热着**（Gazebo 已经在跑）只重启 `point_lio`，`use_imu_as_input`
取 `true` / `false` 都稳定在毫米级漂移 ⇒ 与 `use_imu_as_input` 无关。差别只在
**`point_lio` 是否在 Gazebo 刚起、机器人还在落地的那几秒里启动**：它会在 `IMU_init`
用最初若干帧 `mean_acc` 定重力方向（`IMU_Processing.cpp` 的 `Set_init` / `IMU_init`），
沉降期的瞬时加速度把重力对齐带偏，之后积分再也不会回到正确姿态。

**决定**：`localization_sim.launch.py` 用 `TimerAction` 把 `point_lio` 延后启动，
新增 `DeclareLaunchArgument("point_lio_delay", default_value="4.0")`；仿真侧不 fork、
不打补丁上游 `point_lio`，`config/point_lio_sim.yaml` 保持上游默认值
（`use_imu_as_input: False`）。

**回执**：默认参数干净重启，52/64/76 s 时 `/aft_mapped_to_init` 为
`(-0.0011, 0.0007, 0.0005)` / `(-0.0016, 0.0005, 0.0007)` / `(-0.0022, 0.0001, 0.0004)`。

## D13 仿真侧健康时限：`odom 2.0 s` / `lidar 2.5 s` / `lost 5.0 s`

**现象**：里程计与配准都正常（插桩 `inlier=1.000 resid=0.0943 conv=1 iters=7`，
`probe_found=1688/1688`），`/localization/state` 却一直 `pose_valid=false`，
`age_sec` 只增不减。

**根因**：`diesel_localization` 的健康判定默认阈值是
`odom_timeout_sec 0.5` / `lidar_timeout_sec 0.2` / `lost_timeout_sec 1.0`，
而 **LOST 是粘滞状态，只能靠 Relocalize 恢复**。本仿真里两个量都贴着阈值：
- 里程计消息戳取 `lidar_beg_time`，比 `/clock` 落后约 0.17 s，`now-last_odom` 常态顶到 0.47 s，
  发布间隔最大 0.32 s；
- 软件渲染的点云在宿主机 12 核被占满时只有 4 Hz，帧间隔 p50 0.20 s、p95 0.60 s、
  p99 1.01 s、最长 1.60 s（`/tmp/gap.py` 45 s 采样：`over0.5=21`、`over1.0=2`）。

于是一次正常抖动就喂进一个「断流」样本，累计过 `lost_timeout_sec` 后永久 LOST。

**决定**：只在本仓 launch 的 `localization_params` 里放宽为
`odom_timeout_sec: 2.0`、`lidar_timeout_sec: 2.5`、`lost_timeout_sec: 5.0`，并声明这是
**仿真侧容差、不是定位算法指标**——真机口径由 `diesel_localization` 自己定，本仓不改它的算法。
配套把 `initial_pose_delay` 默认从 12 s 收到 6 s（`point_lio` 已在 4 s 起，摆位不必再等 12 s）。

**回执**：默认参数 95 s 采样 4737 条状态 ⇒ `{INITIALIZING: 314, OK: 4423}`，
**0 条 DEGRADED、0 条 LOST**；首次 OK 出现在仿真 3.78 s，位姿 `(6.0606, 4.0425)`，
与真值 `/sim/pose_info` `(6.0, 4.0, 0.011)` 的水平误差 0.083 m，`yaw=0.012°`。
A2 的三条判据（10 s 内 OK、位置 < 0.30 m、朝向 < 5°）全部满足。

> **D9 更正**：D9 里「point_lio 的两个回调从未被调用」是**误判**——插桩证明
> `lidar_cbk` 每秒 8 次、`imu_cbk` 每秒约 195 次都在正常进入，真正断在
> `ign_sim_pointcloud_tool` 的 `ang_bottom` 符号（D11）。D9 的路线 1（继续查上游）
> 已在本轮闭环，**不需要**降级到路线 2/3 的真值 odom。

## D14 仓级规范文件与 CI 门禁：把组织约束变成可执行断言

**背景**：`foray_docs/repository_structure.md` §10 要求每个仓含
`CODEOWNERS` 与 `.github/workflows/ci.yml`，§6.4/§6.5/§6.6/§7.3 给了具体口径。
本仓 2026-10 新建时只带了 AGENTS.md 与 `.diesel-layer`，缺这两个文件——同一份 AGENTS.md
（md5 `b4f2d108…`）在 interfaces/localization/navigation/sim 四个仓里逐字相同，
但只有本仓没有任何门禁去执行它，等于规矩只写在墙上。

**决定**：

1. `CODEOWNERS` 放仓根（§7.3 明确组织 `.github` 里那份不生效），正文与兄弟仓一致
   `* @ZQU-Foray/算法组`；owner 角色先记「集成」，因 §2 的 10 仓表尚未收录本仓
   （收录另走 `foray_docs` 的 PR）。
2. `.clang-format` 直接取 interfaces/platform 的 canonical 文件
   （md5 `631c338f79fd5d3fb2cd729a9576fdd2`）——本仓当前无 C/C++，但 tree.md 已声明它存在，
   且 2 空格回退是 clang-format 的默认陷阱，先放好过等有代码了再补。
3. `pyproject.toml` 只声明 `[tool.ruff] target-version = "py310"`，与 humble 的 Python 对齐。
4. `.github/workflows/ci.yml` 从组织模板裁剪为 7 个作业：`dir-layout` · `file-hygiene` ·
   `doc-sync` · `layer-check` · `lint-python` · `lint-c`（无 C/C++ 时自动跳过） · `build-ros2`。
   门禁实现放 `scripts/ci/`，只用标准库，本地与 CI 跑同一份代码。

**刻意不引入的三个作业**（照抄他仓会让它们恒跳过，形成「假绿灯」）：

- `param-contract`：本仓不发布 msg/srv/action，参数清单在 README §4；
- `contract-lock`：同上，没有接口契约面；
- `gate-selftest`：门禁自检需要注入缺陷的沙箱，本仓先立门禁，自检留作 D15 的欠账。

**代价（一次性）**：现有 14 个 `.py` 按 black 重排、ruff 修 27 项（其中 5 项手工：
`dll` 未用变量改 `_dll_handle`、`subprocess.run` 补 `check=False`、顶层兜底 `except Exception`
加 `noqa: BLE001` 并注明理由、`dict()` 改字面量、`int(round(x))` 去多余 `int()`）；
5 个 CAD/STEP 导出 JSON、`model.config`、两个 SDF 的行尾由 CRLF 统一为 LF，两个 SDF 补末尾换行。
素材只动行尾与末尾换行：JSON 逐个 `json.loads`、XML 逐个 `minidom.parse` 复验通过。

**回执**：`ruff check .` → `All checks passed!`；`black --check .` → `17 files would be left unchanged`；
`python3 scripts/ci/check_style.py` → 通过；`check_tree_doc.py` → 「91 个文件、30 个目录与仓库双向一致」；
`check_layer.py` → 「layer=META，depends_on_layers=[]」。

## D15 CI 不跑整链仿真；门禁自检留作欠账

**决定**：`build-ros2` 作业只做 `colcon build` + `ros2 pkg executables` + `launch --show-args` +
`colcon test`，**不在 GitHub runner 上起 Gazebo**。

**理由**：
- 本仓两个包都是 `project(... NONE)` 的纯资源/装配包，构建期不需要上游运行时；
  `point_lio` / `ign_sim_pointcloud_tool` / `diesel_localization_ros` 等都是 `exec_depend`，
  rosdep 用 `-r` 跳过即可。
- 整链验收（A0–A7）依赖软件渲染的 Gazebo，帧率随宿主机负载在 4–7 Hz 之间漂
  （见 D13 的实测），在共享 runner 上既不稳也不快；它已经由 `docker/` 镜像 + 本地容器回归覆盖。
- 镜像构建要拉 `pb2025_sentry_nav` 与 Livox-SDK2，耗时远超 CI 合理预算。

**欠账**：`gate-selftest`（向仓库注入已知缺陷，证明每道门禁真的会红）尚未做。
localization 的 D15/D18 已经证明「恒跳过的门禁」是本组织最真实的失败模式，
本仓下一轮补上，届时同步更新 `decision.md` 与 ci.yml 的作业集合。

**实证（A2 整链门对宿主状态敏感）**：把 A2 判据固化成门脚本（清理 → 起 launch → 70 s 采样 → 判据）
之后，2026-10-30 连跑两次出现过一次误判：第二次输出
`samples=3451 first_ok_delay=5.01 s max_xy_err=2.261 max_yaw=4.220 last_state=3 lost=True -> FAIL`。
诊断：上一轮 launch 的 `ign_sim_pointcloud_tool` 进程没被清掉，容器内同时有两个实例在转点云，
叠加宿主 `loadavg 14.6`（12 核）⇒ 帧率塌陷触发 LOST。`docker restart` 清空后同一套用例
`8 passed`。所以**整链门只当本地人工回归用，判红前先确认容器内没有残留 gz/ign 进程**；
这也是它不进 CI 的又一条理由（runner 上无法保证这种干净度）。

## D16 Nav2 用**派生**导航栅格，官方 PGM 只给定位

**决定**：定位继续吃 `src/diesel_rmul_2027_field/maps/rmul_2027/rmul_2027.pgm`（A4/A5 的
哈希契约不动），Nav2 另起一张**派生图** `src/diesel_simulation_bringup/map/nav2/rmul_2027_nav.pgm`，
由 `scripts/make_nav_map.py` 从官方图逐像素复制后，只把隧道顶投影矩形
（x 3.325–3.675 / y 7.06–7.86，共 462 格）由全占据改成全自由，其余 res/origin/thresholds 逐字保持。

**理由**：
- 官方图把高地与隧道顶的**投影**都画成占据，对平面底盘偏保守；`parameters.json` 的
  `limitations` 自己写着「The supplied PGM conservatively blocks highlands and tunnel roof
  footprints for this planar robot」「A lidar-only occupancy slice differs from a chassis
  traversability map」。也就是说，官方图不是「底盘能不能走」的图，直接喂 Nav2 必然判隧道不可通。
- 反过来，从 PCD 重建栅格不成立：`map/rmul_2027/rmul_2027.pcd` 是 8 cm 采样（102433 点），
  投到 0.025 m 栅格只点亮 1510 格，中间有洞，Nav2 会当成零星障碍物。
- 隧道金属顶在 z 0.25–0.27，底盘包络 z 0–0.15，垂直净空 0.2495 m ⇒ 底盘物理上能过，
  「占据」只对雷达投影成立。派生图清掉的正是这一块不该算给底盘的格子。

**代价与护栏**：多一张图就必须防它被手改。`test/test_nav_map.py` 固化了三条断言：
差异必须**只有**这 462 格且全部落在投影内（不许新增占据）、矩形内必须全自由、
x=2.6/3.0/3.3375/3.5/3.6625/3.9/4.3 各列连续自由带 ≥ 0.70 m。生成脚本幂等
（固定 PGM 头 + 只改 yaml 的 `image:` 行），仓库里的 sha256 是
`00f3d2f775d7fbb784a0d2aa8153df60dbdc193082ac15e6cd9c62cbbc6eeb1b`。
隧道顶投影的边界从 `parameters.json` 的 `highland_southwest` / `metal_strip_west_offset` /
`metal_strip_width` / `highland_north_y` / `tunnel_clear_span` 现算，不在脚本里写死数字。

## D17 `odom→base_footprint` 由上游 `sensor_scan_generation` 补，Nav2 只做装配

**决定**：不自己写 TF 广播节点，用上游两件套补链条：
`loam_interface`（`/aft_mapped_to_init` → `/lidar_odometry` + `/cloud_registered`，只发
`odom→雷达` 的里程计消息，不发 TF）+ `sensor_scan_generation`（订阅这两条，发
`odom→base_footprint` TF 与 `/odometry` 话题）。Nav2 本体用
`IncludeLaunchDescription(nav2_bringup/launch/navigation_launch.py)` 装配，不手抄 8 个节点；
`map_server` 与它的 lifecycle_manager 由本仓单独起，**不用 amcl**。

**理由**：
- `diesel_localization` 只发 `map→odom`，`robot_state_publisher` 吃的是无 joint 的 SDF，
  所以 `odom→base_footprint` 这一段在本仓原本是空的——Nav2 的 costmap 一查就
  `Extrapolation Error ... from frame [base_footprint] to frame [map]`。
- `sensor_scan_generation` 恰好就是干这件事的上游实现（`publishTransform(tf_odom_to_chassis,
  odometry->header.frame_id, base_frame_)`），且 `robot_base_frame` 可配；我们没有云台，
  三个 frame 参数都填 `base_footprint`。
- 不启 amcl：amcl 会自己发 `map→odom`，和 `diesel_localization` 抢同一条边，定位会跳。

**代价**：`sensor_scan_generation` 依赖 `lidar→base` 静态 TF，那条边由
`localization_sim.launch.py` 发，所以 `navigation_sim.launch.py` 必须 include 它而不是
另起仿真；两个节点都用 `respawn=True` 兜住启动顺序。

## D18 验收脚本入库（A2/A3/A6），门禁自检也入库

**决定**：把运行时判据固化成 `scripts/accept.py`（`ready` / `a2` / `a3` 三条子命令，
单行 `key=value` + `verdict=PASS|FAIL`），并把 `scripts/ci/selftest.py` 接进 CI。
D15 里「A2 判据门脚本没入库」的欠账一并还清。

**理由**：
- 判据写在会话里、结论写在 `plan.md` 里，等于不可复现：D15 自己就记了一次误判
  （残留进程 + 高负载 ⇒ 假 LOST）。判据只有变成能重跑的命令，才谈得上「取证」。
- 脚本取不到真值时打印 `ground_truth=none` 并让判据失败，而不是悄悄用过默认起点——
  假通过比判红危险。
- `gate-selftest` 的理由同 D15 的欠账段落：一个被改坏的门禁会安静地一直通过。
  `selftest.py` 复制仓库到临时目录、`git init` 后注入五类缺陷
  （CRLF / tree 少一行 / 多一个未登记文件 / 删掉已登记文件 / `.diesel-layer` 反向依赖），
  断言对应门禁非零退出，最后跑干净副本必须三门全绿。它不需要 Docker，所以能进 CI。

## D19 本仓采用 Apache-2.0

**决定**：仓根 `LICENSE` 用 Apache License 2.0 全文（202 行，取自 Debian
`/usr/share/common-licenses/Apache-2.0`，逐字节未经改写）；两个 `package.xml` 的
`<license>` 由 `TODO` 改为 `Apache-2.0`；`THIRD_PARTY_NOTICES.md` §5 记版权行
`Copyright 2026 ZQU-Foray` 与授权范围。

**理由**：上游 `pb2025_*`、`rmu_gazebo_simulator`、`rmoss_*` 都是 Apache-2.0，同许可证
才能逐文件搬用片段而不引入混用；同时保留专利授权与免责条款。场地迁移包与官方 STEP
不在授权范围内（§2 已声明），本次不改它们的归属说明。

## D20 `cmd_vel` 桥接的 gz 侧话题必须带 `/model/` 前缀

**决定**：`src/diesel_simulation_bringup/config/bridge.yaml` 的 `cmd_vel` 条目的
`gz_topic` 由 `/diesel_sim_robot/cmd_vel` 改为 `/model/diesel_sim_robot/cmd_vel`。

**理由（实测证据）**：`ignition-gazebo-velocity-control-system` 在模型命名空间下订阅，
启动日志原话是 `VelocityControl subscribing to twist messages on
[/model/diesel_sim_robot/cmd_vel]`。原来的桥接把 ROS `/cmd_vel` 转发到
`/diesel_sim_robot/cmd_vel`，桥接日志照样打印「Creating ROS->GZ Bridge」、ROS 侧
`ros2 topic pub` 也照常成功，**但车永远不动**——是一个只靠 `ign topic -l` 看不出、
只有对比真值位姿才会暴露的静默错接。

- 错接时：`ros2 topic pub -r 20 /cmd_vel ... "{linear: {x: 0.5}}"` 持续 10 s，
  `/sim/pose_info` 里 `diesel_sim_robot` 一直停在 (6.0, 4.0)。
- 改对后：同样命令 9 s，真值 x 由 6.0 变 7.35（墙钟 9 s、RTF ≈ 0.3，与
  `0.5 m/s × 仿真时间` 相符）。

**影响**：A3（Nav2 穿隧道）在修复前不可能通过——`controller_server` 只会反复
`Failed to make progress`，因为速度从未进到物理引擎。A1/A2/A4/A5 不涉及运动，
所以这个错接从迁移起一直没被发现。凡是以后改模型或桥接，必须用「发速度 + 读真值」
验证闭环，不能只看话题列表。

## D21 `rviz_config` 只收视图名或绝对路径，找不到就启动失败；调车视图另开 `tune.rviz`

**决定**：`launch/simulation.launch.py` 新增 `rviz_config` 参数（默认 `sim.rviz`），
并在 `localization_sim.launch.py`、`navigation_sim.launch.py` 逐层透传；
`scripts/sim_up.sh` 增 `--rviz-config F` 与 `--tune` 简写。解析规则：含路径分隔的绝对
路径直接用，否则拼 `share/rviz/<名字>`；文件不存在时抛错并列出现有候选，不静默空窗。
调车视图另立 `rviz/tune.rviz`，`rviz/sim.rviz` 一字不改，仍作 A0/A2 取证的最小集。

**理由**：
- RViz 用不存在的 `-d` 配置启动时窗口照开、只是一片空场，调车时最容易被误判成
  「话题没数据」。把失败前移到 launch 阶段，一行报错胜过半小时 echo。
- 视图是调试物，不该随着调车需求反复改验收基线：`sim.rviz` 的显示项被
  `test/test_tune_configs.py` 锁成 `{TF, /livox/lidar, /map}`，谁改谁红。
- 手写 `.rviz` 的话题名打错时 RViz 只会显示空数据，所以话题进白名单
  （每条注明来源：`bridge.yaml` / nav2 参数 / launch 常量），新话题必须显式登记。

**已知限制（待容器回执）**：宿主没有 `rviz2`/`gz`，本轮只做到 YAML 结构、
话题白名单与 launch 参数解析一级；「窗口真开出来且每项有数据」、
`Odometry` display 是否支持 `Keep`（累积轨迹）、`/map` 是否被
`diesel_map_server` 与 nav2 `map_server` 双发布，都要在
`diesel_sim:humble-fortress` 容器里补证。

## D22 gz GUI 面板用绝对 x/y 摆放，`WorldControl` 必须显式 `start_paused=false`

**决定**：`config/gui.config` 在原有 `MinimalScene` + `GzSceneManager` 之后追加
`WorldControl`、`WorldStats`、`EntityTree`、`ComponentInspector` 四个面板，每个用
`<ignition-gui><property key="x"/"y">` 给绝对像素位置（左上两块、右侧两块），
`WorldControl` 显式写 `<start_paused>false</start_paused>`，
`test/test_tune_configs.py` 把「必需面板集合」「浮窗必须有 x/y」「start_paused 必须是
false」三条写成断言。

**理由**：
- 面板不给 x/y 时全部叠在窗口左上（3D 视图正中），调车时等于没开。
- `start_paused` 若写成 `true`，世界一启动就停在暂停态：`/clock` 不走、`/livox/lidar`
  一帧不推，A2/A3 会表现为「全链没数据」，而 GUI 侧看起来完全正常。
- `WorldControl` 的 `<service>` 与 `WorldStats` 的 `<topic>` 故意留空：不写才按当前
  世界名推导（`/world/rmul_2027/...`），写死会在换世界后静默失效——和 D20 同一类坑。

**待补回执**：宿主无 `gz`，插件注册名尚未在容器内用 `ign gui -l` 逐条核对；布局也只
按经验值摆放（1400 px 宽窗口）。容器里首次开 GUI 时需确认：四个面板都出现、
没有压在机器人身上、暂停/单步按钮可用。若某面板类名与实际注册名不符，
`gz sim` 会在终端打印 `Failed to load plugin`，此时改 `gui.config` 的 `filename`
并同步 `KNOWN_GZ_GUI_PLUGINS`。

## D23 调车 rqt 用启动器按需拉面板，且不写进 `package.xml`

**决定**：新增 `scripts/tune_rqt.sh`（装到 `lib/diesel_simulation_bringup`），由
`navigation_sim.launch.py` 的 `rqt:=true` 以普通 `Node` 拉起；`sim_up.sh` 加 `--rqt`
（`--no-nav` 时自动收掉，因为 `localization_sim.launch.py` 没有该参数）。
脚本逐个 `command -v` 检查 `rqt_graph`/`rqt_topic`/`rqt_plot`/`rqt_console`/
`rqt_tf_tree`/`rqt_image_view`，缺哪个就打印 `ros-<distro>-rqt-*` 包名再继续，
一个都没有才非零退出；`trap` 统一收摊。**`package.xml` 不声明 rqt 依赖。**

**理由**：
- 不用 `rqt --perspective-file`：perspective 是 Qt 的二进制布局，手写必然与安装的 Qt
  版本不符，加载失败时只得到一片空白窗口，比没开更误导。按需拉独立面板则降级自然。
- 缺面板不整组失败：调控制器时最要紧的是 `rqt_plot` 的「下发 vs 实际速度」，
  不能因为没装 `rqt_graph` 就全丢。
- 不写 `package.xml`：rqt 只是可选调试工具，写进 `exec_depend` 会让 CI 镜像与
  `rosdep` 安装链条变重，而 CI 从不打开任何窗口。代价是容器里要手动装，
  脚本已经把该执行的命令打出来了。
- `test/test_tune_rqt.py` 把三条行为锁住：语法、面板清单、缺 rqt 时的非零退出与包名提示；
  另加 `sim_up.sh --rqt` 的透传与 `--no-nav` 收敛断言。

**未覆盖**：面板真实弹出、`rqt_plot` 三条曲线真的有数据，需在
`diesel_sim:humble-fortress` 容器 + X11 下人工确认。

## D24 调车快照用统一采样窗口，缺数据标 `missing` 而不是 0 Hz

**决定**：新增 `scripts/tune_report.py`（装到 `lib/diesel_simulation_bringup`），用一个
`--window` 窗口同时采样 `/livox/lidar`、`/cloud_registered`、`/aft_mapped_to_init`、
`/odometry`、`/localization/state`、`/cmd_vel`、`/sim/pose_info`，并按
`map→odom→base_footprint→front_mid360` 逐段查 TF，最后输出人读表格或 `--json`
（顶层键 `generated_at`/`window_s`/`topics`/`tf`/`localization`/`cmd_vs_truth`）。
失效阈值直接复用 D13 的 `ODOM_TIMEOUT_SEC=2.0`、`LIDAR_TIMEOUT_SEC=2.5`、
`LOST_TIMEOUT_SEC=5.0`；顶层不 import rclpy，判断逻辑全在纯函数里。

**理由**：
- 调车时「车不动」至少有四种成因（速度没进物理引擎、点云断流、TF 断链、定位 LOST），
  逐条 `ros2 topic hz` 采样窗口各不相同，结论对不上；统一窗口才能互相印证。
- 取不到就标 `missing`、`hz` 留 `None`：把「没数据」显示成 `0 Hz` 会让人误判成
  「发布者存在但极慢」，而实际是根本没发布者。
- 顶层不 import rclpy，宿主就能跑 `test/test_tune_report.py`（10 Hz 合成序列、
  2.0 s 边界判 stale、空序列判 missing、`--json` 键契约），不必等容器。

**已踩过的坑（写进代码注释）**：`/sim/pose_info` 桥过来是 `tf2_msgs/TFMessage`
（gz 的 `pose/info`），不是位姿消息本体；要按 `child_frame_id` 取
`diesel_sim_robot/chassis`，退回模型根位姿。`LocalizationState` 用
`state` 常量 + `pose_valid`，没有 `status` 字段——初版按 `status` 写是错的。

**未覆盖**：真实话题上的频率数值、TF 查询在 `use_sim_time` 下的表现、
`/cmd_vel` 与真值位移的偏差量级，都要在容器里跑一次才有回执。

## D25 仿真链补 `base_footprint → body` 静态 TF，父方向不可反

**决定**：`launch/localization_sim.launch.py` 增一条 `body_static_tf`
（`tf2_ros/static_transform_publisher`：父 `base_footprint`、子 `body`、六个分量全 `0.0`），
并新增 `test/test_localization_sim_launch.py`（ctest 名 `localization_sim_tf`）做结构性断言：
父帧必须是 `base_footprint`、子帧必须是 `body`、`body_tf` 必须出现在 `LaunchDescription` 里、
且没有第二条静态 TF 以 `body` 为父。

**理由**：
- `point_lio` 把机体帧硬编码成 `body`（`laserMapping.cpp:228` 点云 header、
  `:270` odom 的 `child_frame_id`），而它**从不发布 `body` 的 TF**——它广播的那条在
  `:280-292`（`publish.tf_send_en` 默认 `true`），是 `camera_init → aft_mapped`。
  定位节点只在 TF 查得到 `body → base_footprint` 时才收点云，缺这条就一帧不吃、
  `publish_state()` 四支成因全不命中，状态冻在 `INITIALIZING`（实测 `ready=FAIL`、
  每 2 s 一条 `TF body->base_footprint 失败`）。
- 方向只能是 `base_footprint → body`：`base_footprint` 的父已经是上游里程计（D17），
  再给它挂一个父会把 TF 树拆成两棵互不相连的树（tf2 单亲规则），实测日志
  `Could not find a connection between 'base_footprint' and 'body'`；改成这个方向后
  `ready=PASS`、准静态探针 1218/1218 帧 `state=OK`。
- 六个分量取 `0.0` 是实测口径，不是几何推导：`body` 是 `point_lio` 自己的机体原点，
  与 `base_footprint` 的偏差会同时进里程计与雷达外参。实车若量出固定偏移，只改这一条参数。
- 断言用静态 AST、不 `import` ROS：`colcon test` 不该为一条帧契约拉起 gazebo；
  负向自检已确认它拦得住——把方向改成 `body → base_footprint` 时报出两条问题
  （父帧、子帧各一条）。

**未覆盖**：实车链同样缺这一层（`point_lio` 两个仓都不发 `body` TF），实车 launch 要单独落实；
本仓只有仿真 launch。另 `camera_init → aft_mapped` 是一对孤立的两节点树，当前没有订阅者
跨树查询，所以留它不做改动。

## D26 仿真点云下采样按「够配准」定，不照抄上游的 32 线机械雷达口径

- **背景**：审计 L7 报 `GICP 无有效对应（source=64~70 target=4137 iterations=0）`，
  localization 长期 `LOST`，A2 掉锁 5571 帧。查下来 `source` 是配准源点数，
  一帧 `/livox/lidar` 有 60000 条射线，但只有 **9985（16.6%）** 有回波——
  `model.sdf` 里 MID-360 的垂直视场是 `-7° ~ +52°`，场地 `rmul_2027` 只有约 7 m 见方
  且没有高处结构，朝上的射线基本打空。剩下的有效点再经 `ign_sim_pointcloud_tool`
  按 `row_id` 过滤（`ring` 全为 0 的点都在第 0 行）、`point_filter_num: 8` 抽稀、
  `filter_size_surf: 0.2` 体素化，最终只剩 500 出头。
- **决策**：`config/point_lio_sim.yaml` 取 `point_filter_num: 3`、`filter_size_surf: 0.1`，
  `filter_size_map` 保持 `0.2`。
- **实测（本机 GPU，headless，干净 `ROS_DOMAIN_ID`）**：
  - `/cloud_registered` 572 → **1352 点**；
  - `GICP 无有效对应` 每 2 s 一条 → **0 条**；
  - 退化告警 5/6 方向不可观（condition≈1e10、点面约束 64~74）→ **2/6（condition≈3.3e3、点面约束≈780）**；
  - `accept.py ready` FAIL → **PASS**（`ready_delay_s=0.3`、`last_state=OK`）；
  - `accept.py a2 --duration 60` FAIL（`lost=5571`）→ **PASS**（`samples=12122`、
    `max_xy_err=0.035 m`、`lost=0`）。
- **不取 `point_filter_num: 1`**：全量入 ikd-tree 后单帧负担翻 8 倍，实测那一轮
  `/aft_mapped_to_init` 跑到 1e4 量级（配准与下游一起垮）。先按「够用」推进。
- **`filter_size_map` 不动**：它决定全局地图密度，改小对内存与单帧耗时的影响远大于
  `filter_size_surf`，而配准源点数由前者决定。
- **已知未解**：`accept.py a3` 仍 `ABORTED`（`path_len_m=356`、`max_xy_err=262`），
  期间 `/aft_mapped_to_init` 跑到 `x=511 y=-639`——**运动状态下 point_lio 会发散**，
  与参数无关（静止时位姿稳定、A2 原地旋转 PASS）。另有一次冷启动 `52` 条
  `GICP 无有效对应`、`state_reason="no valid registration"` 后不再自恢复，
  说明「首次配准失败」没有重试路径。两者都待单独处理。
- **测量注意**：链一次只能有一条。`pkill` 在新的 PID 命名空间里杀不到上一轮进程，
  旧链会和新的同时发 `/clock`，把 RTF 量成 2.4（真值 1.000）。每次验证换新的
  `ROS_DOMAIN_ID`。

## D27 Nav2 整栈进 `/diesel/<robot>`：三处必须显式做的事

<!-- lint: allow-long-lines -->
Context：定位侧的话题改相对名后（`diesel_localization` D61），导航侧也要落到同一个
`/diesel/<robot>`，否则多机同场时 `bt_navigator`、`costmap`、`cmd_vel` 会互相抢。

Decision：

1. **用 `PushRosNamespace` 包住 nav2 include**。humble 的
   `nav2_bringup/launch/navigation_launch.py` 里 `namespace` 只用于参数 YAML 的
   `root_key` 与组合容器名，非组合模式下**不给各 Node 加 namespace**。只传
   `namespace:=` 的结果是参数键被改写成 `/diesel/sentry/<节点>`、节点名仍是全局的，
   于是 Nav2 退回默认参数——实测日志 `Couldn't load critics!`、
   `Controller frequency set to 20.0000Hz`（本仓配置是 10.0）。
2. **参数 YAML 顶层键是相对节点名，进命名空间后必须重写**。`nav2_params_sim.yaml`
   一直是相对键（`bt_navigator:` 等），`sentry.yaml` 也是（`diesel_localization:`、
   `diesel_map_server:`）。节点进了命名空间，这些键匹配不上，ROS 2 会**静默丢弃**
   整份参数——症状是启动行打印的是代码默认值而不是兵种配置。
   对策：仿真侧两个 include 都在 launch 里用 `nav2_common.launch.RewrittenYaml`
   以 `root_key=LaunchConfiguration("robot_namespace")` 包一层；`root_key` 解析为空串
   时 `RewrittenYaml.perform()` 不改写，实车（无 namespace）行为不变。
3. **跨边界话题写绝对名，TF 帧名不进 namespace**。`/odometry`（Nav2 的
   `odom_topic`）与 `/cmd_vel`（gz 桥的 `ros_topic`）在两侧都被写死成绝对名，
   否则相对名会解析成 `/diesel/sentry/odometry`、`/diesel/sentry/cmd_vel`，
   桥建得起来、日志不报错，车永远不动。TF 帧（`map` / `odom` / `base_footprint` /
   `body`）是全局契约，不随命名空间变化。

Reason：`/cmd_vel` 与 `map` 是跨机器人共享资源的唯一入口，一旦进 namespace，
gz 桥、`nav2_map_server` 与上游里程计就各自指向不同的话题，多机同场必然互相覆盖。

Consequences：`accept.py` 的生命周期探针与 `NavigateToPose` action 客户端改为
`<ns>/bt_navigator/get_state`、`<ns>/controller_server/get_state`、
`<ns>/navigate_to_pose`；`robot_namespace` 提成 `navigation_sim.launch.py` 的
launch 参数（默认 `/diesel/sentry`），实机换机器人只改这一个值。

已知未决（2026-11-12 记，未修）：本机自编栈起链时 `align_source≈1223`、
degeneracy `4/6 个方向不可观`，跨过 `max_unobservable_dofs: 3` 就会一路 LOST；
M11 时代的容器栈是 `align_source≈1697`、`2/6`。点云为何变稀（`voxel=2171` 对
M11 的 `3197`）尚未定位，不动兵种阈值——那是安全边界，不能为了仿真好看放宽。

## D28 Nav2 进 namespace 后必须用 `tf_relay` 把全局 TF 喂进去（`SetRemap` 无效）

<!-- lint: allow-long-lines -->
Context：D27 把 Nav2 整栈推进 `/diesel/sentry` 之后，`accept.py ready` 一直超时：
节点全在、参数也对，但代价地图每秒刷 `Timed out waiting for transform from
base_footprint to odom to become available, tf error: Invalid frame ID "odom" …
frame does not exist`，`bt_navigator` 停在 `inactive`，`controller_server` 连
`activating` 都没完成。定位侧同时是健康的（`state: OK`、`a2 verdict=PASS`）。

实测证据（`navigation_sim.launch.py headless:=true`，t≈40 s）：

```
/diesel/sentry/tf          Publisher count: 0   Subscription count: 5
/tf                        Publisher count: 4   Subscription count: 3
```

Nav2 各节点的 tf2 监听器按**相对名**订阅 `tf`/`tf_static`，`PushRosNamespace`
之后落到 `/diesel/sentry/tf`；而 TF 发布者（`point_lio`、`sensor_scan_generation`、
`diesel_localization`）都在根 `/tf` 上。于是监听器永远收不到帧——**D27 第 3 条只说了
「TF 帧名不进 namespace」，漏了「tf 话题本身会被 namespace 吃掉」这一层**。

Alternative（已试，**无效**）：在 Nav2 那组 `GroupAction` 里 `PushRosNamespace` 之后加
`SetRemap(src="tf", dst="/tf")` + `SetRemap(src="tf_static", dst="/tf_static")`。
重跑后错误一条不少（仍 346 条），查子进程命令行 `grep -c "tf:=/tf"` = **0** ——
`launch` 的 remap 不随 `IncludeLaunchDescription` 传播进被 include 的 nav2 描述，
子进程压根没拿到 remap 参数。这条修法「看着对」，验收时只看 launch 文件也会通过，
所以必须留痕。

Decision：新增 `scripts/tf_relay.py`（`tf_relay` 节点），把根 `/tf`、`/tf_static`
中继到 `<robot_namespace>/tf`、`<robot_namespace>/tf_static`，随 `PushRosNamespace`
一起放进 Nav2 组，用 `tf_relay:=false` 可关。QoS 必须与发布端一致：
`/tf` 用 `best_effort + volatile`，`/tf_static` 用 `reliable + transient_local`
（QoS 不匹配时 ROS 2 不报错、只静默不投递）。

Reason：TF 是全局契约（帧名全局唯一），而 nav2 各节点的 tf2 监听器写死了相对名
`tf`/`tf_static`、拿不到 remap；在「不能改上游」的前提下，唯一干净的做法是让命名空间里
多出一个 TF 发布者。上游实机只有一个命名空间，掩盖了这件事；仿真里 Nav2 在 namespace、
发布者在根，问题才暴露。

Consequences：`test/test_navigation_sim_tf_remap.py` 静态断言「同一个 `GroupAction` 里
既有 `PushRosNamespace`、又引用了 `tf_relay` 节点、其 `target_namespace` 引用
`robot_namespace`，且没有 `SetRemap` 残留」，注册为 ctest `navigation_sim_tf_remap`
（`colcon test` 即跑，不用起仿真）。
代价：`tf` 在命名空间里会被复制一份（带宽翻倍，仿真可接受）；实机把整栈放根命名空间时
需 `tf_relay:=false`，否则中继节点的空 `target_namespace` 保护会直接退出（`SystemExit(2)`）。

## D29 装配层解耦：引擎调用 / 资产选择 / 桥接契约都是声明式输入

Context：三条 launch 把场次名（`simulation.launch.py:31` `WORLD_NAME = "rmul_2027"`）、
机器人名（`:32` `ROBOT_NAME = "diesel_sim_robot"`）、gz 话题全名（`config/bridge.yaml:8`
`/world/rmul_2027/model/diesel_sim_robot/link/front_mid360/sensor/...`）、验收目标点与隧道窗口
（`scripts/accept.py:40` `TUNNEL_RECT`）分别写死在 Python、YAML 和脚本里。换个场次名或换个
机器人名时桥接**照建、不报错、数据全断**（与 D20 同一类静默错接），换一套场地做几何与算法
的交叉验证也无从下手。而 `simulation.launch.py:34-60` 的 `gz_bridge_arguments()` 已经是
「从 `bridge.yaml` 生成 `parameter_bridge` 参数与 remappings」的生成器——解耦方向本来就在，
只是停在半路：生成器只处理桥，场地/机器人/验收仍是字面量。

Decision：

1. **四层，launch 退化为解释器**。L0 资产层（`diesel_rmul_2027_field`，纯数据无节点）；
   L1 引擎适配层（`diesel_sim_tools/engine_gz.py`：资源路径、世界启动参数、spawn 参数、
   真值话题 是 gz 的唯一出口）；L2 桥接契约层（`config/bridge.yaml` + `bridge.render()`）；
   L3 装配层（`config/fields/*.yaml` + `config/robots/*.yaml` 决定用哪个世界、哪台机器人、
   哪些桥接件）；L4 验收层读 profile 的目标点与窗口。
2. **桥接走「模板 + 占位符」，不做结构化派生**。gz 话题写成
   `/world/{world}/model/{robot}/link/front_mid360/sensor/front_mid360_lidar/scan/points`，
   不做 `kind/link/sensor` 三段拼装——拼装等于把 gz 的命名规则编码进 Python，换引擎或换
   传感器就要改代码；模板让「gz 侧怎么写」留在声明里，reviewer 直接可读。占位符渲染后
   **残留未替换即启动失败并指出条目 `id`**：这个话题写错不会报错、只会静默断流（D20）。
3. **不拆资产包**。场地资产已独立成包；机器人只有一份 `model.sdf`，拆包不带来收益，却要
   同步 `tree.md` 双向校验、CI 的 `dir-layout`、`docker/Dockerfile` 三处。
4. **不做多引擎插件化、不做资产热切换、不把 nav2/定位参数 profile 化**。前者只有 Fortress
   一个引擎（D1），此时抽象引擎接口是空壳，差异收敛到一个文件就够；资产热切换超出需求；
   nav2 与定位参数属下游仓职责（README §8）。

Reason：切换维度横跨 SDF 世界、gz 话题名、地图契约、验收目标点四个平面，只加一个
`world:=` 参数会得到「能启动、没数据」的空链。声明式化的收益是**新增一路桥接件不改 Python**，
而不是「支持多引擎」。

Consequences：新增 A8（装配可配置）与 A9（桥接契约自检）两条需求，见 README §1；
`accept.py ready` 承担契约自检，`bridge.yaml` 每条要写 `expect`（发布者/订阅者 + 频率）。
等价性判据不是 A3 通过（A3 现为 FAIL），而是 A0–A2 回执与 **A3 失败指纹**（`path_len_m` /
`max_xy_err` / `cross_samples` 量级、报错链顺序）与重构前一致。

回退：库包与 YAML 读取是按提交切的等价替换，逐提交 `git revert` 即可；单条桥接件改回
字面量话题只会让 A9 断言失败，不影响运行。

## D30 下位机链路在仿真里用「固件源码 + 虚拟 CDC」，不用 QEMU/Renode

Context：`/cmd_vel` 目前由 `config/bridge.yaml` 的 `id: cmd_vel` 直接桥进 gz
（`/model/{robot}/cmd_vel`），协议层被完全绕过——仿真验不到「上位机发一帧、下位机收一帧」
这段最容易出错的接口。真车侧的栈是 RoboLink v0.2 over USB CDC：PC 侧在
`diesel_platform::link`（`usb_cdc_port.hpp` / `robolink_frame_parser.hpp` /
`robolink_host_link.hpp`，500 ms 判活），契约在 `diesel_interfaces`（X2），固件侧在
`foray_mcu`（`src/hal/usb_cdc.hpp`、`src/middleware/robolink/session.hpp`、`app/pc_link`）。
其中 PC 侧只有离线单测（192/192），固件侧收到快照只记账不执行，两边都没实机对账。

Decision：

1. **固件侧新增 host 后端，跑真源码**。`foray_mcu` 的 `platform/host_sim/` 实现
   `hal::UsbCdc`（PTY 上做非阻塞读写、`MAX_WRITE_LENGTH=512` 照契约），`hal::os` 用
   `steady_clock`，can/uart/spi/gpio/pwm 复用既有 `platform/mock/`。跑道是**同一份**
   `hal::UsbCdc` + `robolink::Session` + `app/pc_link`，所以切帧、CRC、握手、判活都是真代码。
2. **虚拟 USB CDC 用 PTY 对，不用 usbip / tty0tty / 内核 gadget**。`virtual_cdc.py` 建一对
   PTY，两端 `raw`、`VMIN=0 VTIME=0`，`select` 双向中继，符号链接给两个稳定路径。
   usbip 需要内核模块与真实设备、tty0tty 要编内核模块，都超出「本地可复现」的成本线；
   PTY 没有设备权限和加载模块的前置。
3. **不做 QEMU/Renode 全系统仿真**。QEMU 没有 STM32H7 整机模型更没有 USB OTG device 模型；
   Renode 有 `stm32h7.repl`，但 STM32 的 USB CDC 至今未支持（renode#722、#533 均 open）——
   真固件能在里面跑，`hal::UsbCdc` 却没有外设，链路永远连不上。等这两个上游补上再评估。
4. **底盘执行暂由仿真侧过渡桥顶替**。固件 `app/` 下没有 chassis 能力，若坚持「只走真固件」，
   A10 就只能验到「快照被收下」，验不到车动。因此 `diesel_sim_mcu` 作为从机侧会话的对手方，
   把 `chassis_vx/vy/wz` 转成 gz 车体速度；电控组补上 chassis + CAN 电机后，这一层退化为
   「CAN 电机替身」，**协议侧不再有替身**。
5. **只覆盖底盘平面速度**。云台/发射机构一律不驱动——协议未冻结，PC 侧文档原文即
   「不得据此驱动云台或发射机构」。

Reason：这条链路的验证价值在「接口」而非「物理」，而接口的真伪取决于**跑的是不是同一份
编解码与状态机**。host 后端让真源码进链路，成本是一个后端文件加一个 profile；QEMU/Renode
的成本是等上游补齐 USB CDC，且即使补齐也只能验固件内部逻辑，验不到 PC 侧与固件的字节对账
（那本来要在实机上做）。

Consequences：新增 A10（下位机链路可用）与 A11（链路自检），见 README §1；
`mcu_link:=true` 时 `/diesel/<robot>/cmd_vel` 归过渡桥独占，Nav2 同时发布即启动失败；
PTY 无法检测对端是否打开，host 后端以「端口打开即 connected」近似，实机的连接语义留到上机验。
跨仓改动（`foray_mcu`）owner 是电控组，落地前需协调；不可动时的替代是仓内验证夹具 + PR 草案。

回退：`mcu_link` 默认 `false`，链路整段不装配，回归到 D29 的直连桥；固件侧 `platform/host_sim/`
与 `config/profiles/sim_dm_mc02/` 是纯新增，删除不影响 ARM 构建与既有 profile。
