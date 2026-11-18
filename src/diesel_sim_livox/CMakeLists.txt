cmake_minimum_required(VERSION 3.16)
project(diesel_sim_livox CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

# 纯函数采样核心：只依赖标准库，无 ROS/gz 依赖，裸容器也能编译与单测。
# 角度表解析 + 网格采样 + 帧点生成的全部数学都在这里，插件只负责喂数据与发布。
add_library(livox_pattern STATIC src/pattern.cpp)
# 静态核心会被链进共享插件，必须 -fPIC，否则 ld 报 relocation R_X86_64_PC32。
set_target_properties(livox_pattern PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(livox_pattern PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(livox_pattern PUBLIC cxx_std_17)

# Livox CustomMsg 与 gz 渲染/仿真库都只存在于整链仿真环境（容器镜像 / 本地
# diesel_ws + Ignition Fortress），CI 的裸 ros:humble 容器拿不到。与 diesel_sim_mcu
# 对 diesel_platform_core 的处理一致：找不到就跳过插件目标，保证空仓/裸容器绿灯。
# 注意顺序：ignition-gazebo6 必须先于 rclcpp/livox 查找——它经 fuel_tools7 引
# jsoncpp，若 ROS 侧已导入 JsonCpp::JsonCpp 目标会撞「target already exists」。
find_package(ignition-gazebo6 QUIET)
find_package(ignition-rendering6 QUIET)
find_package(ignition-plugin1 QUIET)
find_package(ignition-common4 QUIET)
find_package(ZLIB QUIET)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 QUIET)

set(_livox_deps_found TRUE)
foreach(_pkg livox_ros_driver2 ignition-gazebo6 ignition-rendering6 ignition-plugin1 ignition-common4 ZLIB)
    if(NOT ${_pkg}_FOUND)
        set(_livox_deps_found FALSE)
        message(STATUS "未找到 ${_pkg}，跳过 diesel_sim_livox 的 gz 插件（整链仿真环境才需要）")
    endif()
endforeach()

if(_livox_deps_found)
    add_library(LivoxMid360System SHARED src/livox_mid360_system.cpp)
    target_link_libraries(LivoxMid360System PRIVATE
        livox_pattern
        ignition-gazebo6::ignition-gazebo6
        ignition-rendering6::ignition-rendering6
        ignition-plugin1::ignition-plugin1
        ignition-common4::ignition-common4
        ZLIB::ZLIB
        ${rclcpp_TARGETS}
        ${ament_index_cpp_TARGETS}
        ${sensor_msgs_TARGETS}
        ${livox_ros_driver2_TARGETS}
    )
    # gz 通过 IGN_GAZEBO_SYSTEM_PLUGIN_PATH 找插件，装到 lib/ 即可被搜索到。
    install(TARGETS LivoxMid360System DESTINATION lib)
endif()

# 官方角度表（gzip 压缩的 scan_mode/mid360.csv）随包安装，launch 用 share 路径喂插件。
install(DIRECTORY scan_mode DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    enable_testing()
    # 纯核心单测由 pytest 驱动（编译 pattern.cpp + pattern_test.cpp 再跑），
    # 与 diesel_sim_mcu 的 test 注册方式一致，dev_task verify 只认 pytest。
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME livox_pattern_test
        COMMAND ${Python3_EXECUTABLE} -m pytest test/test_pattern.py -q
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
