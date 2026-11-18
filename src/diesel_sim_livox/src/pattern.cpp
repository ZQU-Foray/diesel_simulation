#include "diesel_sim_livox/pattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace diesel_sim_livox {
namespace {
// 四角深度跳变阈值：max/min 超过它就认为踩到深度不连续（墙角/障碍边缘），
// 反深度双线性会把边缘两侧混在一起，此时退化最近邻。1.25 = 允许 25% 的深度起伏，
// 平面墙在一个 0.176° 网格单元内的深度变化远小于此，故平面仍走插值恢复精确值。
constexpr double kDepthDiscontinuityRatio = 1.25;

// 反射率不建模材质（需求非目标），统一给一个中等值；point_lio 只把它当 intensity。
constexpr float kDefaultReflectivity = 100.0F;

bool ValidRange(double r, double range_min, double range_max) {
    return std::isfinite(r) && r >= range_min && r <= range_max;
}
} // namespace

bool PatternTable::LoadCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        directions_.clear();
        return false;
    }
    return LoadCsv(in);
}

bool PatternTable::LoadCsv(std::istream& in) {
    directions_.clear();
    elevation_min_ = 0.0F;
    elevation_max_ = 0.0F;

    std::string line;
    // 表头：Time/s,Azimuth/deg,Zenith/deg
    if (!std::getline(in, line)) {
        return false;
    }

    bool first = true;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string tok;
        // 列 0 = Time/s（序号，丢弃）；列 1 = Azimuth/deg；列 2 = Zenith/deg
        if (!std::getline(ss, tok, ',')) {
            continue;
        }
        if (!std::getline(ss, tok, ',')) {
            continue;
        }
        const double az_deg = std::atof(tok.c_str());
        if (!std::getline(ss, tok, ',')) {
            continue;
        }
        const double zen_deg = std::atof(tok.c_str());

        PatternDirection d;
        d.azimuth_rad = static_cast<float>(az_deg * M_PI / 180.0);
        d.elevation_rad = static_cast<float>((90.0 - zen_deg) * M_PI / 180.0);
        d.line = static_cast<std::uint8_t>(directions_.size() % 4);
        directions_.push_back(d);

        if (first) {
            elevation_min_ = elevation_max_ = d.elevation_rad;
            first = false;
        } else {
            elevation_min_ = std::min(elevation_min_, d.elevation_rad);
            elevation_max_ = std::max(elevation_max_, d.elevation_rad);
        }
    }

    return !directions_.empty();
}

std::size_t PatternTable::FrameStart(std::size_t frame, std::size_t points_per_frame) const {
    if (directions_.empty() || points_per_frame == 0) {
        return 0;
    }
    return (frame * points_per_frame) % directions_.size();
}

void DirectionToUnitVector(double azimuth_rad, double elevation_rad, double& x, double& y,
                           double& z) {
    const double ce = std::cos(elevation_rad);
    x = ce * std::cos(azimuth_rad);
    y = ce * std::sin(azimuth_rad);
    z = std::sin(elevation_rad);
}

double RangeGrid::Sample(double azimuth_rad, double elevation_rad) const {
    if (ranges == nullptr || count < 2 || vertical_count < 2 || angle_step <= 0.0 ||
        vertical_step <= 0.0) {
        return -1.0;
    }

    const double fh = (azimuth_rad - angle_min) / angle_step;
    const double fv = (elevation_rad - vertical_angle_min) / vertical_step;
    if (fh < 0.0 || fh > count - 1 || fv < 0.0 || fv > vertical_count - 1) {
        return -1.0; // 方向在网格 FOV 外
    }

    int h0 = static_cast<int>(std::floor(fh));
    int v0 = static_cast<int>(std::floor(fv));
    h0 = std::min(h0, count - 2);
    v0 = std::min(v0, vertical_count - 2);
    const double tx = std::min(1.0, std::max(0.0, fh - h0));
    const double ty = std::min(1.0, std::max(0.0, fv - v0));

    const double r00 = ranges[v0 * count + h0];
    const double r10 = ranges[v0 * count + h0 + 1];
    const double r01 = ranges[(v0 + 1) * count + h0];
    const double r11 = ranges[(v0 + 1) * count + h0 + 1];

    const bool v00 = ValidRange(r00, range_min, range_max);
    const bool v10 = ValidRange(r10, range_min, range_max);
    const bool v01 = ValidRange(r01, range_min, range_max);
    const bool v11 = ValidRange(r11, range_min, range_max);

    if (v00 && v10 && v01 && v11) {
        const double rmin = std::min({r00, r10, r01, r11});
        const double rmax = std::max({r00, r10, r01, r11});
        if (rmax <= rmin * kDepthDiscontinuityRatio) {
            // 平滑区域：反深度双线性（平面的倒数深度在屏幕坐标线性，可恢复精确值）
            const double inv = (1.0 - tx) * (1.0 - ty) / r00 + tx * (1.0 - ty) / r10 +
                               (1.0 - tx) * ty / r01 + tx * ty / r11;
            return (inv > 0.0) ? 1.0 / inv : -1.0;
        }
    }

    // 深度不连续或任一角无效：退化最近邻（取整到最近网格中心）
    int hn = static_cast<int>(std::lround(fh));
    int vn = static_cast<int>(std::lround(fv));
    hn = std::min(count - 1, std::max(0, hn));
    vn = std::min(vertical_count - 1, std::max(0, vn));
    const double rn = ranges[vn * count + hn];
    return ValidRange(rn, range_min, range_max) ? rn : -1.0;
}

std::size_t SampleFrame(const PatternTable& table, const RangeGrid& grid, std::size_t frame,
                        std::size_t points_per_frame, std::uint32_t point_interval_ns,
                        std::vector<SampledPoint>& out) {
    out.clear();
    if (table.empty() || points_per_frame == 0) {
        return 0;
    }
    out.reserve(points_per_frame);

    const std::size_t start = table.FrameStart(frame, points_per_frame);
    const std::size_t n = table.size();
    for (std::size_t i = 0; i < points_per_frame; ++i) {
        const PatternDirection& d = table.at((start + i) % n);
        const double range = grid.Sample(d.azimuth_rad, d.elevation_rad);
        if (range <= 0.0) {
            continue; // 未命中 / FOV 外：丢弃，与真雷达无回波一致
        }
        double ux, uy, uz;
        DirectionToUnitVector(d.azimuth_rad, d.elevation_rad, ux, uy, uz);
        SampledPoint p;
        p.x = static_cast<float>(range * ux);
        p.y = static_cast<float>(range * uy);
        p.z = static_cast<float>(range * uz);
        p.offset_time_ns = static_cast<std::uint32_t>(i) * point_interval_ns;
        p.line = d.line;
        p.tag = 0x10;
        p.reflectivity = kDefaultReflectivity;
        out.push_back(p);
    }
    return out.size();
}

} // namespace diesel_sim_livox
