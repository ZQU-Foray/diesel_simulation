cmake_minimum_required(VERSION 3.8)
project(diesel_sim_mcu CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 车体速度直接写进 gz（不走 ros_gz_bridge）：`mcu_link:=true` 时桥接契约里的
# `cmd_vel` 条目不再承担下行（README §3.2 的独占约束），这条通道由本包负责。
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

# 下位机链路的 PC 侧实现（USB CDC 字节通道 + RoboLink v0.2 主机状态机）在
# diesel_platform 的 link 库里，仿真仓不重复实现协议（decision.md D30）。
# 没装就跳过本包的可执行文件：默认档（mcu_link:=false）不需要它。
find_package(diesel_platform_core QUIET)

if(diesel_platform_core_FOUND)
    add_executable(sim_mcu_bridge src/sim_mcu_bridge.cpp)
    target_link_libraries(sim_mcu_bridge PRIVATE
        diesel_platform::link
        ignition-transport11::ignition-transport11
        ignition-msgs8::ignition-msgs8
        ${rclcpp_TARGETS}
        ${geometry_msgs_TARGETS}
        ${std_msgs_TARGETS}
    )
    install(TARGETS sim_mcu_bridge DESTINATION lib/${PROJECT_NAME})
else()
    message(STATUS "未找到 diesel_platform_core，跳过 diesel_sim_mcu（README §3.2 的 mcu_link 需要它）")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    # 纯 python 断言用原生 ctest（与本仓其它包一致）
    enable_testing()

    # A10：过渡桥必须保留失联兜底（链路断了往 gz 写 0），且 gz 话题与桥接契约同源
    add_test(
        NAME sim_mcu_bridge_contract
        COMMAND python3 test/test_sim_mcu_bridge.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

ament_package()
