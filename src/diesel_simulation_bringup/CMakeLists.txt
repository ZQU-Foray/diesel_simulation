cmake_minimum_required(VERSION 3.8)
project(diesel_simulation_bringup NONE)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch config rviz models
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY map
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS scripts/make_pcd.py scripts/make_manifest.py scripts/make_nav_map.py
             scripts/set_initial_pose.py scripts/accept.py
             scripts/sim_up.sh scripts/tune_rqt.sh scripts/tune_report.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    # 纯 python 断言用原生 ctest（ament_add_test 需要 gtest 风格结果文件）
    enable_testing()

    # A4：清单自洽 + 栅格改 1 字节必须被检出
    add_test(
        NAME manifest_consistency
        COMMAND python3 test/test_manifest.py map/rmul_2027
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # A5：由网格采样的点云投影 vs 官方地图，占据 IoU 必须 ≥ 0.90
    add_test(
        NAME pcd_align
        COMMAND python3 test/test_pcd_align.py
                --pcd map/rmul_2027/rmul_2027.pcd
                --grid map/rmul_2027/rmul_2027.pgm
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    set_tests_properties(pcd_align PROPERTIES TIMEOUT 120)

    # A3 前置：派生导航栅格只许打开隧道开放段，且走廊宽度够规划
    add_test(
        NAME nav_map
        COMMAND python3 test/test_nav_map.py
                --official-pgm ../diesel_rmul_2027_field/maps/rmul_2027/rmul_2027.pgm
                --official-yaml ../diesel_rmul_2027_field/maps/rmul_2027/rmul_2027.yaml
                --parameters ../diesel_rmul_2027_field/maps/rmul_2027/parameters.json
                --nav-pgm map/nav2/rmul_2027_nav.pgm
                --nav-yaml map/nav2/rmul_2027_nav.yaml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # 调车 rqt 面板组：脚本语法 + 缺包时的行为（宿主没有 rqt 也能跑）
    add_test(
        NAME tune_rqt
        COMMAND python3 test/test_tune_rqt.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # 调车健康快照：频率/失联判定与 --json 契约（纯函数，宿主即可跑）
    add_test(
        NAME tune_report
        COMMAND python3 test/test_tune_report.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # 调车视图：rviz/tune.rviz 的话题白名单、sim.rviz 最小集不许被改、gz GUI 插件名
    add_test(
        NAME tune_configs
        COMMAND python3 test/test_tune_configs.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # L1：仿真链必须发 `base_footprint -> body`（point_lio 硬编码 body，缺这条定位吃不到帧）
    add_test(
        NAME localization_sim_tf
        COMMAND python3 test/test_localization_sim_launch.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

ament_package()
