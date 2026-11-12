#!/usr/bin/env bash
# 调车用 rqt 面板组（I13）：一次开出图、话题、曲线、日志、TF 树、图像六块面板。
#
# 用法：tune_rqt.sh [--dry-run]
#   --dry-run  只打印每个面板在不在、缺哪个包，不真的开窗口（门禁与 CI 用这个）
#
# 设计取舍：
#   - 不用 `rqt --perspective-file`：perspective 是 Qt 的二进制布局，手写必然与实际
#     Qt 版本不符，加载失败只会得到一片空白。这里改为「按需拉独立面板」，缺哪个少哪个。
#   - 缺面板不整组失败：调控制器时最需要的往往是 rqt_plot，不能因为没装 rqt_graph 就全丢。
#   - 一个都没装才算失败（退出码 1），并打印对应的 apt 包名。
set -o pipefail

# 面板表：<标识> <可执行文件> <apt 包名>。apt 包名随 ROS_DISTRO 变化，见下方提示。
PANELS=(
    "graph rqt_graph rqt-graph"
    "topic rqt_topic rqt-topic"
    "plot rqt_plot rqt-plot"
    "console rqt_console rqt-console"
    "tf_tree rqt_tf_tree rqt-tf-tree"
    "image_view rqt_image_view rqt-image-view"
)

# rqt_plot 打开时先画这三条：调控制器第一眼看的就是「下发的速度 vs 实际速度」
PLOT_TOPICS=(
    "/cmd_vel/linear/x"
    "/cmd_vel/angular/z"
    "/odometry/twist/twist/linear/x"
)

dry_run=false
if [[ ${1:-} == "--dry-run" ]]; then
    dry_run=true
    shift
fi

missing=()
found=()
for row in "${PANELS[@]}"; do
    read -r name cmd pkg <<<"$row"
    if command -v "$cmd" >/dev/null 2>&1; then
        found+=("$name $cmd")
        if [[ $dry_run == true ]]; then
            echo "panel=$name available=true"
        fi
    else
        missing+=("$name $cmd $pkg")
        if [[ $dry_run == true ]]; then
            echo "panel=$name available=false"
        fi
    fi
done

if [[ $dry_run == true ]]; then
    exit 0
fi

if [[ ${#missing[@]} -gt 0 ]]; then
    distro=${ROS_DISTRO:-humble}
    echo "以下面板没装（不影响其余面板启动）：" >&2
    for row in "${missing[@]}"; do
        read -r name cmd pkg <<<"$row"
        echo "  - $cmd（$name）：sudo apt install ros-${distro}-${pkg}" >&2
    done
fi

if [[ ${#found[@]} -eq 0 ]]; then
    echo "一个 rqt 面板都没找到：先装 ros-${ROS_DISTRO:-humble}-rqt-plot 等包，或确认已 source 工作空间" >&2
    exit 1
fi

pids=()
cleanup() {
    # 统一收摊：Ctrl-C 时把拉起的 rqt 全部带走，免得残留面板连着重启后的图
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

for row in "${found[@]}"; do
    read -r name cmd <<<"$row"
    case "$name" in
        plot)
            "$cmd" "${PLOT_TOPICS[@]}" >/dev/null 2>&1 &
            ;;
        *)
            "$cmd" >/dev/null 2>&1 &
            ;;
    esac
    pids+=("$!")
    echo "started $cmd（pid $!）" >&2
done

# 等第一个面板退出即可（关掉任一窗口就整组收摊，避免留下看不见的孤儿进程）
wait -n
