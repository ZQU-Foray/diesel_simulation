#!/usr/bin/env bash
# 一键起整链（A6）：仿真 + 机器人 + gz 桥 + 点云时序修复 + point_lio + diesel_localization
# + odom→base_footprint 补全 + Nav2。默认 `navigation_sim.launch.py`；
# `--no-nav` 退回只起仿真+定位（`localization_sim.launch.py`）。
#
# 起不来时先看这两处：
#   1) 定位是否 OK：`ros2 topic echo /localization/state --once`；
#   2) TF 是否齐：`ros2 run tf2_ros tf2_echo map base_footprint`。
# 就绪判定用 `ros2 run diesel_simulation_bringup accept.py ready --deadline 300`。
set -o pipefail

usage() {
    cat <<'USAGE'
用法: sim_up.sh [选项]

  --gui           打开 Gazebo 图形界面（默认 headless 软件渲染）
  --rviz          同时打开 RViz
  --rviz-config F RViz 视图（rviz/ 下的文件名，默认 sim.rviz；调车用 tune.rviz）
  --tune          调车一键：等价于 --rviz --rviz-config tune.rviz
  --rqt           同时拉起调车 rqt 面板组（缺哪个面板少哪个，不影响整链）
  --mcu-link      走虚拟 USB CDC + 下位机固件（A10、D30），默认直连 gz
  --mcu-firmware F 同时代起固件 host_sim（foray_mcu 的 firmware_sim 路径）
  --no-nav        只起仿真 + 定位，不起 Nav2
  --nav-delay S   Nav2 延后多少秒启动（默认取 launch 的 25.0）
  --x X           初始位姿 x（默认取 launch 的 6.0）
  --y Y           初始位姿 y（默认取 launch 的 4.0）
  --yaw RAD       初始朝向（默认 0.0）
  -h, --help      显示本帮助
USAGE
    exit "${1:-1}"
}

gui=false
rviz=false
rviz_config=""
rqt=false
with_nav=true
nav_delay=""
pose_args=()
mcu_args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui) gui=true; shift ;;
        --rviz) rviz=true; shift ;;
        --rviz-config) rviz_config="${2:?--rviz-config 需要一个视图文件名}"; shift 2 ;;
        --tune) rviz=true; rviz_config=tune.rviz; shift ;;
        --rqt) rqt=true; shift ;;
        --mcu-link) mcu_args=("mcu_link:=true"); shift ;;
        --mcu-firmware) mcu_args=("mcu_link:=true" "mcu_firmware:=${2:?--mcu-firmware 需要一个可执行文件路径}"); shift 2 ;;
        --no-nav) with_nav=false; shift ;;
        --nav-delay) nav_delay="${2:?--nav-delay 需要一个秒数}"; shift 2 ;;
        --x) pose_args+=("x:=${2:?--x 需要一个数值}"); shift 2 ;;
        --y) pose_args+=("y:=${2:?--y 需要一个数值}"); shift 2 ;;
        --yaw) pose_args+=("yaw:=${2:?--yaw 需要一个数值}"); shift 2 ;;
        -h | --help) usage 0 ;;
        *) echo "未知参数: $1" >&2; usage 1 ;;
    esac
done

# GUI 打开时不再要求 --headless-rendering（两者互斥，见 simulation.launch.py）
headless=true
if [[ $gui == true ]]; then
    headless=false
fi

if [[ $with_nav == true ]]; then
    launch_file=navigation_sim.launch.py
else
    launch_file=localization_sim.launch.py
    if [[ -n $nav_delay ]]; then
        echo "--no-nav 时忽略 --nav-delay" >&2
        nav_delay=""
    fi
fi

# 给了视图就一定要开 RViz，否则参数被静默忽略、主人会以为窗口没配对
if [[ -n $rviz_config ]]; then
    rviz=true
fi

# rqt 参数只存在于 navigation_sim.launch.py，退化到 --no-nav 时不要传未知参数
if [[ $rqt == true && $with_nav == false ]]; then
    echo "--no-nav 时不带 rqt 面板组（rqt 参数在 navigation_sim 里）" >&2
    rqt=false
fi

args=("gui:=$gui" "headless:=$headless" "rviz:=$rviz")
# 视图名与 rqt 只在显式给出时透传：默认值由 launch 一处维护
if [[ $with_nav == true ]]; then
    args+=("rqt:=$rqt")
fi
if [[ -n $rviz_config ]]; then
    args+=("rviz_config:=$rviz_config")
fi
# 位姿只在显式给出时透传：让 launch 里的默认值成为唯一来源，避免脚本又藏一套默认值
args+=("${pose_args[@]}")
if [[ -n $nav_delay ]]; then
    args+=("nav2_delay:=$nav_delay")
fi
args+=("${mcu_args[@]}")

exec ros2 launch diesel_simulation_bringup "$launch_file" "${args[@]}"
