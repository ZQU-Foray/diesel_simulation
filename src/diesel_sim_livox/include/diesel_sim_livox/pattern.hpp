// Livox Mid-360 非重复扫描的纯函数采样核心（无 ROS/gz 依赖，可独立单测）。
//
// 职责：把官方 scan_mode/mid360.csv 的非重复角度表，采样到一张均匀距离网格上，
// 产出带逐点时间戳的点。角度表决定「往哪个方向看」，网格决定「那个方向多远」。
//
// 为什么是「细网格 + 按角度表采样」而不是逐点发射线：gz 的 RayQuery 两条路径都撑不住
// 200k 射线/秒（CPU 逐三角形求交 / GPU 每射线一次整场渲染），GpuRays 是唯一批量机制，
// 而它只产均匀网格。详见 decision.md D32 的实测取证。
#ifndef DIESEL_SIM_LIVOX__PATTERN_HPP_
#define DIESEL_SIM_LIVOX__PATTERN_HPP_

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace diesel_sim_livox {

// 角度表里的一个扫描方向。line 是发射器编号（官方表每 4 点一组 = 4 个发射器）。
struct PatternDirection {
    float azimuth_rad;
    float elevation_rad;
    std::uint8_t line;
};

// 官方 mid360.csv 角度表。表头 Time/s,Azimuth/deg,Zenith/deg；
// elevation = 90 - zenith（zenith 90° = 水平）。整表 800000 点 @200k/s = 4 s 一个周期。
class PatternTable {
  public:
    // 解析 CSV。成功返回 true；失败（文件不存在/格式错/空表）返回 false 并清空。
    bool LoadCsv(const std::string& path);

    // 从流解析（供插件把 gzip 解压到内存后直接喂入，避免落临时文件）。
    bool LoadCsv(std::istream& in);

    std::size_t size() const {
        return directions_.size();
    }
    bool empty() const {
        return directions_.empty();
    }
    const PatternDirection& at(std::size_t i) const {
        return directions_[i];
    }

    // 表的俯仰角实际范围（弧度）。供插件把 GpuRays 的垂直 FOV 设得恰好罩住角度表，
    // 避免表边缘方向落在网格外被丢弃。
    float elevation_min() const {
        return elevation_min_;
    }
    float elevation_max() const {
        return elevation_max_;
    }

    // 帧 frame 的起始索引（每帧 points_per_frame 点，整表循环）。
    std::size_t FrameStart(std::size_t frame, std::size_t points_per_frame) const;

  private:
    std::vector<PatternDirection> directions_;
    float elevation_min_{0.0F};
    float elevation_max_{0.0F};
};

// 一张均匀距离网格（来自 GpuRays / LaserScan 的角度定义）。
// ranges 行优先：ranges[v * count + h]；方位角 = angle_min + h * angle_step，
// 俯仰角 = vertical_angle_min + v * vertical_step。
struct RangeGrid {
    double angle_min{0.0};
    double angle_step{0.0};
    int count{0};
    double vertical_angle_min{0.0};
    double vertical_step{0.0};
    int vertical_count{0};
    const float* ranges{nullptr};
    double range_min{0.0};
    double range_max{0.0};

    // 采样一个方向的距离（米）。反深度双线性插值：平面墙的倒数深度在屏幕坐标线性，
    // 可恢复精确值；四角深度跳变（max/min 超阈值）或任一角无效时退化最近邻。
    // 未命中 / 方向在网格外返回负值。
    double Sample(double azimuth_rad, double elevation_rad) const;
};

// 一帧采样结果里的一个点。
struct SampledPoint {
    float x;
    float y;
    float z;
    std::uint32_t offset_time_ns;
    std::uint8_t line;
    std::uint8_t tag;
    float reflectivity;
};

// 按角度表对网格采样一帧。frame 决定角度表起点；points_per_frame 通常 20000。
// offset_time = i * point_interval_ns（200k/s ⇒ 5000 ns）；tag 固定 0x10（point_lio
// avia_handler 认 (tag & 0x30) == 0x10 或 0x00）；line 取角度表的发射器编号。
// 只保留命中点；返回实际点数。out 会被 resize 到点数。
std::size_t SampleFrame(const PatternTable& table, const RangeGrid& grid, std::size_t frame,
                        std::size_t points_per_frame, std::uint32_t point_interval_ns,
                        std::vector<SampledPoint>& out);

// 球面方向 → 单位向量（x 前、y 左、z 上；方位角从 +x 朝 +y，俯仰角从 xy 平面起算）。
void DirectionToUnitVector(double azimuth_rad, double elevation_rad, double& x, double& y,
                           double& z);

} // namespace diesel_sim_livox

#endif // DIESEL_SIM_LIVOX__PATTERN_HPP_
