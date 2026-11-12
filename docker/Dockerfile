# 仿真验证镜像：Humble + Fortress + 上游（pin 到固定 SHA）
# 构建：docker build - < docker/Dockerfile
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        libyaml-cpp-dev libeigen3-dev libpcl-dev libgtest-dev \
        python3-pip git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir vcstool2 xmacro

# 上游 pin：与 SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator 的 dependencies.repos 一致，
# pb2025_sentry_nav 的 point_lio / small_gicp_relocalization / pb_nav2_plugins / pointcloud_to_laserscan
# / livox_ros_driver2 / pb_omni_pid_pursuit_controller / pb_teleop_twist_joy 都是子模块，必须递归拉取。
# 只有 pb2025_sentry_nav 取 dev（point_lio / ign_sim_pointcloud_tool / 控制器在其中）。
RUN set -eux; \
    mkdir -p /opt/diesel_ws/src; \
    cd /opt/diesel_ws/src; \
    git clone --quiet https://github.com/gezp/sdformat_tools.git; \
    git -C sdformat_tools checkout --quiet 47c2d1a37372281e5627d444f7537efc0bba11df; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_interfaces.git; \
    git -C rmoss_interfaces checkout --quiet 424f50c831b3e74fa72446969362cdabb2e22cd1; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_core.git; \
    git -C rmoss_core checkout --quiet 8209ec517dfbb7ecac613cf0c9534d67ff918b7d; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo.git; \
    git -C rmoss_gazebo checkout --quiet 7443ff06c345752d0e23172c479d99b27207e2c7; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gz_resources.git; \
    git -C rmoss_gz_resources checkout --quiet b5c759f08844dfda19c79aa870866ace8d4c7b3a; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git; \
    git -C pb2025_robot_description checkout --quiet a0541dddcbfe376f369a6345b532a10923ad9149; \
    git clone --quiet https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git; \
    git -C pb2025_sentry_nav checkout --quiet 5f645d60e8cb88440f0bb44fc63209f136a95a49; \
    git -C pb2025_sentry_nav submodule update --init --recursive --quiet

# Livox-SDK2：livox_ros_driver2 → point_lio 的依赖；版本与 diesel_ws/.repos 的 pin 一致
RUN set -eux; \
    cd /tmp; \
    git clone --quiet https://github.com/Livox-SDK/Livox-SDK2.git; \
    git -C Livox-SDK2 checkout --quiet f5d9375f84efe2b15bc0a052d3e18482ed13adf4; \
    cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release > /tmp/livox.log 2>&1; \
    cmake --build Livox-SDK2/build -j "$(nproc)" >> /tmp/livox.log 2>&1; \
    cmake --install Livox-SDK2/build >> /tmp/livox.log 2>&1

RUN set -o pipefail; set -ex; \
    source /opt/ros/humble/setup.bash; \
    cd /opt/diesel_ws; \
    colcon build --continue-on-error --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        > /tmp/colcon.log 2>&1 || true; \
    grep -E "Summary:|Failed   <<<|Aborted <<<" /tmp/colcon.log | tail -20 || true; \
    grep -B2 -A8 "CMake Error\|error:" /tmp/colcon.log | head -60 || true

RUN echo 'source /opt/diesel_ws/install/setup.bash' >> /ros_entrypoint.sh

WORKDIR /ws
