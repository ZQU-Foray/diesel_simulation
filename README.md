# diesel_simulation

ZQU-Foray 的 RMUL 2027 3v3 **场地仿真仓**：在 Gazebo 里搭出赛季场地，跑通
「点云 → 定位 → 寻路」整链，为实车调试提供可复现的对照环境。

> 本仓**只做仿真**：不实现定位算法（属 `diesel_localization`）、不实现导航算法
> （属 `diesel_navigation`）、不定义接口（属 `diesel_interfaces`）。仿真里暴露给
> 下游的话题名与 `diesel_ws` 实车链路保持同名，算法侧一份配置两处复用。

## 1. 需求（本仓的验收基线）

| 编号 | 需求 | 验收口径 |
|---|---|---|
| A0 | 场地资产可用 | `diesel_rmul_2027_field` 的世界能在 Gazebo 启动、无报错加载 |
| A1 | 32 线点云 | `/livox/lidar` 上 PointCloud2 为 **32 线**（`height=32`）、`width=1875`、nominal **10 Hz**（MID-360 实际出点率）；软件渲染 CI 实测 ≥ 6.5 Hz、GPU 直通达 10 Hz；`/livox/imu` 配置 `update_rate: 200`，实测随 RTF 缩放（软件渲染约 156 Hz） |
| A2 | 定位收敛 | `diesel_localization` 在仿真里 10 s 内状态 OK，位置误差 < 0.30 m、朝向误差 < 5° |
| A3 | 寻路可达 | Nav2 把机器人从起点送到隧道对侧（净空 0.2495 m 的场地几何）`SUCCEEDED`，且轨迹采样点（≥ 5 Hz）至少一个落在隧道顶投影 x 3.325–3.675 / y 7.06–7.86 内 |
| A4 | 地图契约 | manifest 哈希与 `diesel_localization` 一致，篡改 1 字节必须拒绝启动 |
| A5 | 地图同源 | 由网格采样的 PCD 与官方地图的占据 IoU ≥ 0.90 |
| A6 | 一键启动 | `sim_up.sh` 300 s 内整链起来（仿真 + 桥 + 定位 + Nav2 全部 active） |
| A7 | 致谢完整 | 三方来源的归属链写进 `THIRD_PARTY_NOTICES.md` |
| A8 | 装配可配置 | 场地、机器人、桥接三份规格（`config/fields/<field>.yaml`、`config/robots/<robot>.yaml`、`config/bridge.yaml`）是装配的唯一输入：三条 launch 不直接书写场次名 / 机器人名 / gz 话题字面量；桥接规格里的占位符（`{world}` / `{robot}`）在启动前必须全部替换，残留即启动失败并指出条目 `id` |
| A9 | 桥接契约自检 | `accept.py ready` 对 `bridge.yaml` 每条 `expect` 断言对应 ROS 话题按方向存在发布者 / 订阅者；缺失时按条目 `id` 报错并非零退出（D20 类静默错接必须在这里被拦下） |
| A10 | 下位机链路可用 | `mcu_link:=true` 时 `/cmd_vel` 经虚拟 USB CDC 交给**下位机固件源码**（`foray_mcu` 的 `host_sim` 后端），上位机 `RoboLinkHostLink` 状态稳定为 `Active`，固件侧 `robolink::Session` 进 `Ready/Active`；停掉任一端后 **≤500 ms** 双方判活失败，`/cmd_vel` 归零。链路只覆盖底盘平面速度，云台/发射机构不驱动（协议未冻结，见 D30） |
| A11 | 链路自检 | `accept.py link` 打印 `link_state` / `hello_rtt_ms` / `status_hz` / `frame_errors` / `drop_detect_ms`，并在「握手未完成、状态帧断流（持续超过 500 ms）、失联未在 500 ms 内触发」时非零退出；加 `--expect-drop` 时额外要求「链路确实断过，且从最后一帧到判定断开 ≤ 540 ms」——由人停一端触发，停不下来就是 FAIL，不把「一直没断」当成「失联检测正常」 |

进度与阶段划分见 `plan.md`；关键取舍见 `decision.md`。

## 2. 技术栈

下表是**整链依赖**，不是本仓实现：本仓只交付配置、话题契约与验收脚本（见第 6-9 行的边界）。

| 组件 | 归属 | 本仓职责 |
|---|---|---|
| ROS 2 **Humble** | 整链基线 | 无，与 `diesel_ws` 及本地容器工具链保持一致 |
| **Ignition Gazebo Fortress（gz-sim 6.18）** | 仿真器（本仓选择） | 装配与资产，见 `decision.md` D1 |
| `ros_gz_bridge` / `ros_gz_sim` | 桥（本仓选择） | `config/bridge.yaml` 的桥接契约与渲染 |
| `point_lio` | 上游算法，由 `diesel_localization` 依赖 | 仿真点云补 `time` 字段后可直接喂 |
| `diesel_localization` | 上游仓 | 只提供 manifest 与地图（A4） |
| `nav2_bringup` + `pb_omni_pid_pursuit_controller` | 下游仓 `diesel_navigation` | 提供场地与参数标定环境，参数从仿真实测标定 |

## 3. 包结构

```
diesel_simulation/
├── src/
│   ├── diesel_rmul_2027_field/        # 场地资源包（纯资产 + 资源断言测试）
│   │   ├── models/rmul_2027/         # 分件 DAE 网格 + 贴图
│   │   ├── worlds/rmul_2027.sdf      # 唯一交付的世界（调试变体不入库）
│   │   ├── maps/rmul_2027/           # 官方 PGM/YAML + STEP 复核参数
│   │   ├── docs/                     # 场地规格与 STEP 审计
│   │   ├── previews/                 # 场地预览图
│   │   └── tools/                    # 上游审计/生成脚本（只读参考）
│   ├── diesel_sim_tools/              # 装配库（纯 Python：引擎适配 / 桥接渲染 / profile）
│   ├── diesel_sim_mcu/                # 过渡桥（RoboLink 从机侧 ↔ gz 车体速度，D30）
│   └── diesel_simulation_bringup/     # 装配层（launch / profile / 桥接契约 / 地图 / 脚本）
│       ├── map/rmul_2027/            # PCD + PGM + manifest.yaml（定位侧输入）
│       ├── map/nav2/                 # 派生导航栅格（只清隧道顶投影，见 D16）
│       ├── launch/                   # simulation / localization_sim / navigation_sim
│       ├── config/                   # bridge.yaml + fields/（场地）+ robots/（机器人）+ 调参
│       ├── scripts/                  # make_pcd / make_manifest / make_nav_map / sim_up / accept / virtual_cdc
│       └── test/                     # A4 / A5 / 导航栅格回归
├── README.md  AGENTS.md  tree.md  plan.md  decision.md
└── THIRD_PARTY_NOTICES.md
```

### 3.1 装配层：profile 与桥接契约

三条 launch 不写死场地与机器人，装配输入只有三份规格：

| 规格 | 管什么 | 内容 |
|---|---|---|
| `config/fields/<field>.yaml` | 场地 | 世界 SDF、模型资源目录、官方地图、派生导航图、验收目标点与隧道窗口 |
| `config/robots/<robot>.yaml` | 机器人 | 机体 SDF、`model_name`、雷达链路与装位、线数与视场、帧名 |
| `config/bridge.yaml` | 桥接 | 每条桥接件的 `id` / gz 话题模板 / ROS 话题与类型 / 方向 / `expect` |

- **占位符**：gz 话题里用 `{world}` / `{robot}` 引用 profile 的值。启动前渲染，**残留未替换的占位符即启动失败**并指出条目 `id`——这类话题写错不会报错、只会静默断流（D20），必须靠前置校验拦住。
- **扩展一路桥接件**：在 `bridge.yaml` 加一条声明、在 `model.sdf` 加对应传感器片段、必要时在 robot profile 里登记装位，不改 Python 代码。
- **边界**：本轮只做单引擎（Fortress）的声明式装配；多引擎插件化、资产热切换都不做，见 `decision.md` D29。

### 3.2 虚拟下位机链路（可选装配）

默认可跑通不需要它；`mcu_link:=true` 时 `/cmd_vel` 不再直接桥进 gz，而是走真车同构的下行链路：

```
上位机（Nav2 / 遥控）──► /diesel/<robot>/cmd_vel
      └─ diesel_sim_mcu（过渡桥，上位机侧 RoboLinkHostLink）
            └─ 虚拟 USB CDC：/tmp/diesel_host ═PTY 对═ /tmp/diesel_mcu
                  └─ 下位机固件 host_sim（foray_mcu 源码）─ hal::UsbCdc ─ robolink::Session
                        └─ 回读系统状态（link_state / last_rx_control_seq）
      └─ 固件确认收到控制快照后，桥才把 chassis_vx / vy / wz 写进 gz 车体
```

- **虚拟 CDC**：`virtual_cdc.py` 用一对 PTY 顶替 USB CDC，两端各自 `raw` 无缓冲，双向字节中继，退出时清理符号链接；`socat` 可作旁证手段但不是依赖。
- **固件是真的**：链路里跑的是 `foray_mcu` 的同一份源码（`hal::UsbCdc` / `robolink` / `app/pc_link`），只是外设后端换成 PC 的 PTY。QEMU 与 Renode 都不行——两者都没有 STM32 的 USB CDC 模型（`decision.md` D30）。
- **过渡桥的定位**：固件目前**没有 chassis 能力**（收到快照只记账），所以底盘执行由 `diesel_sim_mcu` 顶替；等电控组补上 chassis 与 CAN 电机后，这一层退化成「CAN 电机替身」，协议侧不再是替身。
- **固件在环才算数**：桥在 `RoboLinkHostLink` 处于 `Active`、且固件回报 `link_state=ACTIVE` 与
  `last_rx_control_seq > 0` 之后，才把这段速度写进 gz；判据只取固件自己报的字段——
  固件 `control_msg_age_ms` 第一版恒为 `0xFFFF`（`app/pc_link` 不再更新它），不能拿来当「收到了」。
- **独占约束**：`mcu_link:=true` 时 launch 把 `config/bridge.yaml` 的 `id: cmd_vel` 从桥接参数里
  剔除（`skip_ids=("cmd_vel",)`），gz 侧只有过渡桥一个写入者；ROS 侧 `/diesel/<robot>/cmd_vel`
  仍可能被写，所以**量底盘速度要用 `start_nav2:=false`**——Nav2 空闲时 `behavior_server` 也在发零速，
  与本仓的测试指令互相覆盖（实测 `ros2 topic info -v` 数到 6 个发布者）。

## 4. 数据链

```
gz-sim 场地世界 (rmul_2027.sdf)
   └─ gpu_lidar + imu ─ ros_gz_bridge ─► /livox/lidar  /livox/imu
                                            │
                     ign_sim_pointcloud_tool（补 time 字段）
                                            ▼
                                      point_lio ──► /aft_mapped_to_init（camera_init→body）
                                            │
                        loam_interface ──► /lidar_odometry + /cloud_registered
                                            │
                   sensor_scan_generation ──► odom→base_footprint TF + /odometry
                                            │
             先验地图 manifest.yaml ◄── diesel_localization（重定位）
                                            ▼
                                        map→odom TF
                                            │
                                        Nav2 规划/控制 ──► /cmd_vel
                                                             │
                     mcu_link:=false ─────────────────────────┤（直连：桥接契约 id: cmd_vel）
                     mcu_link:=true  ──► diesel_sim_mcu ─虚拟 USB CDC─► 固件 host_sim（A10）
                                                             ▼
                                              gz /model/<robot>/cmd_vel ─► 车体
```

下行有两档：默认档把 `/cmd_vel` 直接桥进 gz，用来跑验收；`mcu_link:=true` 时接上虚拟下位机链路（§3.2），用来验证「上位机 ↔ 下位机」这段接口在仿真里也是真的。

先验地图与栅格**同源**：两者都从场地网格派生，因此定位与 Nav2 不会各说各话。
不过两者的**判据不同**：定位吃官方 PGM 的稠密占据面，Nav2 需要底盘可通行栅格，
由 `make_nav_map.py` 从官方图派生并只打开隧道顶投影（`decision.md` D16）。

## 5. 复现

```bash
# 1) 由网格采样先验点云（8 cm 采样，102433 点）
python3 src/diesel_simulation_bringup/scripts/make_pcd.py \
    --field-package src/diesel_rmul_2027_field \
    --out src/diesel_simulation_bringup/map/rmul_2027/rmul_2027.pcd

# 2) 生成地图清单（sha256 由脚本计算，勿手改）
python3 src/diesel_simulation_bringup/scripts/make_manifest.py \
    --map-dir src/diesel_simulation_bringup/map/rmul_2027 \
    --out src/diesel_simulation_bringup/map/rmul_2027/manifest.yaml

# 3) 跑回归（A4 清单契约 / A5 地图同源 / 场地资源断言）
colcon test --packages-select diesel_rmul_2027_field diesel_simulation_bringup

# 4) 派生 Nav2 可通行栅格（只清隧道顶投影，幂等）
python3 src/diesel_simulation_bringup/scripts/make_nav_map.py \
    --official-pgm src/diesel_rmul_2027_field/maps/rmul_2027/rmul_2027.pgm \
    --official-yaml src/diesel_rmul_2027_field/maps/rmul_2027/rmul_2027.yaml \
    --out-pgm src/diesel_simulation_bringup/map/nav2/rmul_2027_nav.pgm \
    --out-yaml src/diesel_simulation_bringup/map/nav2/rmul_2027_nav.yaml

# 5) 起整链（仿真 + 桥 + 定位 + Nav2；`--no-nav` 只起定位）
src/diesel_simulation_bringup/scripts/sim_up.sh

# 6) 运行时验收（另开终端；A6 / A2 / A3 各一条）
ros2 run diesel_simulation_bringup accept.py ready --deadline 300
ros2 run diesel_simulation_bringup accept.py a2 --duration 70
ros2 run diesel_simulation_bringup accept.py a3 --x 2.0 --y 7.4
```

`accept.py` 每条子命令只打印单行 `key=value` 并给出 `verdict=PASS|FAIL`，
判据与 `decision.md` D13（A2）、D18（验收脚本入库）一致；没有真值时它会打印
`ground_truth=none` 而不是假装通过。

## 6. 地图判定口径（勿改而不留痕）

- **点云占据**：栅格内存在 `z > 0.02 m` 的点。2 cm 是「底盘可通行」分界——中央控制区
  顶面 10 mm、补给区垫面 3 mm 属可通行，不记占据；围挡 420 mm、高地 200 mm、工作台
  403 mm、隧道顶 270 mm 记占据。
- **比较范围**：外场矩形（内场 12×8 m + 0.3 m 围挡），官方 PGM 的场外 0.1 m 余量不参与
  比较，否则 IoU 会被系统性拉低约 12%。
- **manifest 同目录契约**：`pcd_file` / `grid_file` 是相对 manifest 目录的路径，
  `diesel_localization` 的 `verify_map_hashes:=true` 依赖这一点。

## 7. 致谢

本仓站在以下开源工作上，完整归属与许可证信息见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)：

- **SMBU-PolarBear-Robotics-Team**：`rmu_gazebo_simulator`、`pb2025_robot_description`、
  `pb2025_sentry_nav`、`rmoss_gazebo` / `rmoss_core`；机器人模型、Gazebo 插件与桥接
  配置的写法直接借鉴自这些仓。
- **guodong_sentry_sim 场地迁移包**（RMUL2027_map_transfer_20261016_STEP_R1）：
  `src/diesel_rmul_2027_field` 的网格、地图与场地复核参数来自该包。
- **RoboMaster 社区 wiki**（仿真链路做法参考）。
- 场地几何源自 **RoboMaster 2027 RMUL Rules V1.0.0（20261015）§3.2**；`official_top.png`
  由用户提供的规则手册 PDF Fig 3-1 裁切。DJI / RoboMaster 相关图样版权归其所有者，
  本仓不为其主张任何新许可证。

## 8. 上下游

| 方向 | 仓 | 关系 |
|---|---|---|
| 上游 | `diesel_localization` | 消费本仓的 `manifest.yaml` + PCD/PGM |
| 上游 | `diesel_ws` | 提供同名的实车话题与依赖版本 |
| 下游 | `diesel_navigation` | 本仓提供仿真场地与 Nav2 参数标定环境 |
