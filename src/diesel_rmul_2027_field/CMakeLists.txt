cmake_minimum_required(VERSION 3.16)
project(diesel_rmul_2027_field NONE)

find_package(ament_cmake REQUIRED)

# 纯资源包：世界 / 网格 / 地图 / 文档全部按 ament share 布局安装，供 bringup 用
# `get_package_share_directory` 定位。
install(DIRECTORY models worlds maps docs previews
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    enable_testing()
    # 资源一致性断言：世界插件名前缀、地图几何、网格引用可解析。
    # 不需要 Gazebo 运行时，CI 与本地都能跑。
    add_test(NAME field_assets
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_field_assets.py
                ${CMAKE_CURRENT_SOURCE_DIR}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    set_tests_properties(field_assets PROPERTIES TIMEOUT 60)

    install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
