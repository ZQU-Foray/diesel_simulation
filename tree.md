# tree.md — 目录结构与职责

仓根即 colcon 空间根，包一律在 `src/` 下。

> 本文件与仓库**双向一致**，由 `scripts/ci/check_tree_doc.py` 在 CI 的 `doc-sync` 作业里强制校验：
> 这里列出的每个文件必须真实存在，每个被跟踪文件也必须在这里出现。
> 新增 / 删除 / 重命名文件的提交，必须在同一次提交内更新本文件。

```
foray_simulation/
├── README.md                                  # 本仓唯一需求来源（A0–A7 验收表）
├── AGENTS.md                                  # AI / 协作者规范（组织统一版）
├── THIRD_PARTY_NOTICES.md                     # 三方来源与许可证归属链
├── LICENSE                                    # Apache-2.0 全文（本仓自有代码与文档）
├── tree.md                                    # 本文件：目录结构与职责
├── plan.md                                    # 当前计划与进度（可中断恢复）
├── decision.md                                # 关键工程决策与取证留痕
├── CODEOWNERS                                 # Review 自动分派（§7.3 要求每仓自带）
├── .foray-layer                               # 层级声明（META）
├── .clang-format                              # C/C++ 风格，与 platform/interfaces 同一份
├── .gitignore                                 # 构建产物、运行期产物、.dsh 状态
├── pyproject.toml                             # ruff 目标版本 3.10（与 humble 对齐）
├── .github/
│   └── workflows/
│       └── ci.yml                             # 组织 CI 模板裁剪版：8 个作业
├── docker/
│   └── Dockerfile                             # 全链镜像：humble + Fortress + 上游 pin
├── scripts/
│   └── ci/
│       ├── check_style.py                     # 文件卫生门禁：LF / UTF-8 / 末尾单空行
│       ├── check_tree_doc.py                  # tree.md 与 git ls-files 双向一致
│       ├── check_layer.py                     # .foray-layer 层级声明自洽（§6.5）
│       └── selftest.py                        # 门禁自检：注入缺陷断言三门禁会红
└── src/
    ├── foray_rmul_2027_field/                 # 场地资源包（纯资产，无运行时节点）
    │   ├── package.xml                        #   ament_cmake，exec_depend ros_ign_gazebo
    │   ├── CMakeLists.txt                     #   安装 models/worlds/maps/docs/previews
    │   ├── docs/
    │   │   ├── RMUL_2027.md                   #   场地规格：区域 / 尺寸 / 材质口径
    │   │   └── validation/
    │   │       └── source_machine.json        #   STEP 复核：源机位姿与相机内参记录
    │   ├── maps/
    │   │   └── rmul_2027/
    │   │       ├── rmul_2027.pgm              #   官方场地栅格（占据图母本）
    │   │       ├── rmul_2027.yaml             #   栅格元数据：分辨率 / 原点
    │   │       ├── parameters.json            #   STEP 复核参数
    │   │       ├── scenario.json              #   STEP 复核场景
    │   │       ├── step_import.json           #   STEP 导入记录
    │   │       ├── step_measurements.json     #   STEP 实测尺寸
    │   │       ├── step_geometry.json.gz      #   复核中间几何（压缩，二进制）
    │   │       └── geometry_overlay.png       #   栅格与几何叠合校验图
    │   ├── models/
    │   │   └── rmul_2027/
    │   │       ├── model.config               #   gz 模型元数据
    │   │       ├── model.sdf                  #   场地模型：网格引用 + 材质
    │   │       ├── materials/
    │   │       │   └── textures/
    │   │       │       └── official_top.png   #   官方顶视图贴图
    │   │       └── meshes/
    │   │           ├── blue_highland_control_extra_pvc.dae
    │   │           ├── blue_highland_pvc_2mm.dae
    │   │           ├── blue_highland_wood.dae
    │   │           ├── blue_metal_transition.dae
    │   │           ├── blue_parapet.dae
    │   │           ├── blue_supply_extra_pvc.dae
    │   │           ├── blue_tunnel_far_support.dae
    │   │           ├── blue_tunnel_gain_extra_pvc.dae
    │   │           ├── blue_tunnel_metal_roof.dae
    │   │           ├── blue_workbench.dae
    │   │           ├── center_control_extra_pvc.dae
    │   │           ├── floor_pvc_2mm.dae
    │   │           ├── floor_wood.dae
    │   │           ├── perimeter_east.dae
    │   │           ├── perimeter_north.dae
    │   │           ├── perimeter_south.dae
    │   │           ├── perimeter_west.dae
    │   │           ├── red_highland_control_extra_pvc.dae
    │   │           ├── red_highland_pvc_2mm.dae
    │   │           ├── red_highland_wood.dae
    │   │           ├── red_metal_transition.dae
    │   │           ├── red_parapet.dae
    │   │           ├── red_supply_extra_pvc.dae
    │   │           ├── red_tunnel_far_support.dae
    │   │           ├── red_tunnel_gain_extra_pvc.dae
    │   │           ├── red_tunnel_metal_roof.dae
    │   │           ├── red_workbench.dae
    │   │           ├── step_native.dae
    │   │           └── step_textured.dae
    │   ├── previews/
    │   │   ├── overview_source_machine.png    #   源机渲染预览
    │   │   ├── overview_step_native.png       #   STEP 原生几何预览
    │   │   ├── top_source_machine.png         #   源机顶视
    │   │   └── top_step_native.png            #   STEP 顶视
    │   ├── tools/                             #   上游生成/审计脚本（只读参考，不安装）
    │   │   ├── audit_rmul_step.py             #   STEP 几何审计（OCP 可选）
    │   │   ├── check_rmul_runtime.py          #   运行时抽查：点云 / 相机 / 插件
    │   │   ├── rmul_field.py                  #   场地几何 → SDF/DAE 生成器
    │   │   ├── step_field.py                  #   STEP → 简化几何
    │   │   ├── test_rmul_field.py             #   生成器自测
    │   │   └── test_step_field.py             #   STEP 简化自测
    │   ├── worlds/
    │   │   └── rmul_2027.sdf                  #   唯一交付的世界（Fortress 口径）
    │   └── test/
    │       └── test_field_assets.py           #   资源断言：插件前缀、地图几何、网格可解析
    └── foray_simulation_bringup/              # 启动层：把仿真器接到本仓算法上
        ├── package.xml                        #   ament_cmake，exec_depend 上游与自研包
        ├── CMakeLists.txt                     #   安装 launch/config/rviz/models/map + scripts
        ├── launch/
        │   ├── simulation.launch.py           #   世界 + 机器人 spawn + gz 桥
        │   ├── localization_sim.launch.py     #   点云转换 + point_lio + 定位服务
        │   └── navigation_sim.launch.py       #   定位 + odom→base 补全 + Nav2（A3/A6）
        ├── config/
        │   ├── bridge.yaml                    #   ros_gz_bridge 话题映射
        │   ├── gui.config                     #   gz GUI 布局（3D 视图 + 调车四面板）
        │   ├── point_lio_sim.yaml             #   point_lio 仿真参数（上游默认值）
        │   └── nav2_params_sim.yaml           #   Nav2 仿真参数（按 0.8 m 通道重标）
        ├── rviz/
        │   ├── sim.rviz                       #   验收最小集视图（TF / 原始点云 / 先验图）
        │   └── tune.rviz                      #   调车全量视图（代价图 / 规划 / 工具位）
        ├── map/
        │   ├── rmul_2027/
        │   │   ├── rmul_2027.pcd              #   定位先验点云（由网格采样）
        │   │   ├── rmul_2027.pgm              #   定位用栅格（与官方一致）
        │   │   ├── rmul_2027.yaml             #   栅格元数据
        │   │   └── manifest.yaml              #   清单契约：文件 + sha256
        │   └── nav2/                          #   派生导航图（隧道顶按底盘高度打开）
        │       ├── rmul_2027_nav.pgm          #   512x352，仅隧道开放段与官方图不同
        │       └── rmul_2027_nav.yaml         #   与官方 yaml 同分辨率/原点/阈值
        ├── models/
        │   └── foray_sim_robot/
        │       └── model.sdf                  #   仿真机器人（底盘 + mid360 + IMU）
        ├── scripts/
        │   ├── make_pcd.py                    #   场地网格 → 先验点云（三角面格点采样）
        │   ├── make_manifest.py               #   地图目录 → manifest.yaml（含 sha256）
        │   ├── set_initial_pose.py            #   摆位 + 调定位 Relocalize
        │   ├── make_nav_map.py                #   官方栅格 → 导航栅格（清隧道顶投影）
        │   ├── accept.py                      #   A2/A3/A6 运行时验收（ready/a2/a3）
        │   ├── sim_up.sh                      #   一键起整链（默认含 Nav2；--tune 调车视图）
        │   ├── tune_rqt.sh                    #   调车 rqt 面板组（缺哪个少哪个）
        │   └── tune_report.py                 #   话题/TF/定位健康快照（--json 可脚本化）
        └── test/
            ├── test_pcd_align.py              #   A5：点云投影 vs 官方地图 IoU
            ├── test_manifest.py               #   A4：清单契约 + 篡改检测
            ├── test_nav_map.py                #   A3：派生图差异有界 + 通道宽度断言
            ├── test_localization_sim_launch.py  #   L1：launch 必须发 base_footprint -> body
            ├── test_tune_configs.py           #   调车视图话题白名单 + gz GUI 插件名
            ├── test_tune_rqt.py               #   调车 rqt 启动器行为（语法 / 缺包 / 透传）
            └── test_tune_report.py            #   调车快照：频率/失联判定 + --json 契约
```

## 职责边界

| 目录 | 职责 | 不做什么 |
|---|---|---|
| `foray_rmul_2027_field` | 场地几何、材质、地图基准、场地规格文档 | 不放 launch、不写节点 |
| `foray_simulation_bringup` | 世界组装、机器人 spawn、桥接、参数、一键脚本 | 不实现定位/导航算法（那是 `foray_localization` / `foray_navigation`） |
| `map/rmul_2027` | 定位侧先验（点云 + 栅格 + 清单） | 不做运行时修改；重新生成靠脚本 |
| `map/nav2` | 导航侧栅格（官方图的派生副本 + 差异断言） | 不改官方图；差异必须落在隧道开放段内 |
| `tools/`（field 包内） | 上游资产生成与审计脚本，供追溯 | 不被安装、不被运行时依赖 |
| `scripts/ci/` | 仓级门禁脚本，只用标准库 | 不做业务逻辑、不依赖 ROS 环境 |

## 仓级规范文件

本仓纳入组织规范（`foray_docs/repository_structure.md`）的文件与作业：

| 项 | 位置 | 强制来源 |
|---|---|---|
| CODEOWNERS | 仓根 | §7.3（必须在本仓，组织 `.github` 里那份不生效） |
| CI | `.github/workflows/ci.yml` | §6.4（复制组织模板并按技术栈裁剪） |
| 层级声明 | `.foray-layer` | §6.5（CI `layer-check` 校验） |
| C/C++ 风格 | `.clang-format` | §6.6 |
| Python 风格 | `pyproject.toml`（ruff + black） | §6.6 |
