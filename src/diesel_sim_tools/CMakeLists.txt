cmake_minimum_required(VERSION 3.16)
project(diesel_sim_tools NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 库：三条 launch 与 accept.py 都 import 它。没有可执行体，也没有
# C/C++ 源码，所以用 ament_cmake + ament_python_install_package 而不是 ament_python
# ——本仓 pyproject.toml 明确不配置 build-system（见其注释），再加一套 setuptools
# 打包会与仓内既有约定冲突。
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
    enable_testing()
    # 三个单测都是「脚本 + main()」风格，与 field/test 一致：不需要 colcon 安装
    # 产物即可直接跑，CI 与本机行为相同。
    foreach(unit engine_gz bridge profile)
        add_test(NAME tools_${unit}
            COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_${unit}.py
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endforeach()
    set_tests_properties(tools_engine_gz tools_bridge tools_profile
        PROPERTIES TIMEOUT 60)

    install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
