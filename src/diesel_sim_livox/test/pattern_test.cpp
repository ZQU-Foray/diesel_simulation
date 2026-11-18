// livox_pattern 纯核心单测（无 ROS/gz 依赖，g++ 直接编译运行）。
// 由 test/test_pattern.py 驱动：编译 pattern.cpp + 本文件 → 跑 → 断言退出码 0。
// 用法：pattern_test <mini_pattern.csv 路径>
#include "diesel_sim_livox/pattern.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {
int g_failures = 0;
int g_checks = 0;

void Check(bool ok, const char* name) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n", name);
    }
}

void CheckNear(double got, double want, double tol, const char* name) {
    ++g_checks;
    if (!(std::fabs(got - want) <= tol)) {
        ++g_failures;
        std::printf("FAIL %s got=%.9f want=%.9f tol=%.9f\n", name, got, want, tol);
    }
}

constexpr double kDeg = M_PI / 180.0;

// 构造一张「平面 x = D」的距离网格：range(az,el) = D / (cos el * cos az)。
// 平面倒数深度在屏幕坐标近似线性，反深度双线性应能恢复精确值。
struct PlaneGrid {
    std::vector<float> ranges;
    diesel_sim_livox::RangeGrid grid;

    PlaneGrid(double D, double az_min, double az_span, int count, double el_min, double el_span,
              int vcount) {
        ranges.resize(static_cast<std::size_t>(count) * vcount);
        grid.angle_min = az_min;
        grid.angle_step = az_span / (count - 1);
        grid.count = count;
        grid.vertical_angle_min = el_min;
        grid.vertical_step = el_span / (vcount - 1);
        grid.vertical_count = vcount;
        grid.range_min = 0.1;
        grid.range_max = 100.0;
        for (int v = 0; v < vcount; ++v) {
            const double el = el_min + v * grid.vertical_step;
            for (int h = 0; h < count; ++h) {
                const double az = az_min + h * grid.angle_step;
                ranges[v * count + h] = static_cast<float>(D / (std::cos(el) * std::cos(az)));
            }
        }
        grid.ranges = ranges.data();
    }
};
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: pattern_test <mini_pattern.csv>\n");
        return 2;
    }
    using namespace diesel_sim_livox;

    // ---- LoadCsv：解析、line=i%4、elevation=90-zenith、min/max ----
    PatternTable table;
    Check(table.LoadCsv(argv[1]), "loadcsv_ok");
    Check(table.size() == 16, "loadcsv_size");
    CheckNear(table.at(0).azimuth_rad, 0.0, 1e-6, "az_row0");
    CheckNear(table.at(1).azimuth_rad, 90.0 * kDeg, 1e-6, "az_row1");
    CheckNear(table.at(0).elevation_rad, 0.0, 1e-6, "el_row0_zen90");
    CheckNear(table.at(4).elevation_rad, 10.0 * kDeg, 1e-6, "el_row4_zen80");
    CheckNear(table.at(8).elevation_rad, -10.0 * kDeg, 1e-6, "el_row8_zen100");
    Check(table.at(0).line == 0 && table.at(1).line == 1 && table.at(2).line == 2 &&
              table.at(3).line == 3 && table.at(4).line == 0,
          "line_i_mod_4");
    CheckNear(table.elevation_min(), -10.0 * kDeg, 1e-6, "el_min");
    CheckNear(table.elevation_max(), 10.0 * kDeg, 1e-6, "el_max");

    PatternTable missing;
    Check(!missing.LoadCsv("/nonexistent/no_such_pattern.csv"), "loadcsv_missing_false");
    Check(missing.empty(), "loadcsv_missing_empty");

    // ---- FrameStart：整表循环 ----
    Check(table.FrameStart(0, 5) == 0, "framestart_0");
    Check(table.FrameStart(1, 5) == 5, "framestart_1");
    Check(table.FrameStart(3, 5) == 15, "framestart_3");
    Check(table.FrameStart(4, 5) == 4, "framestart_wrap");

    // ---- DirectionToUnitVector ----
    double x, y, z;
    DirectionToUnitVector(0.0, 0.0, x, y, z);
    CheckNear(x, 1.0, 1e-9, "unit_x_forward");
    CheckNear(y, 0.0, 1e-9, "unit_x_forward_y");
    CheckNear(z, 0.0, 1e-9, "unit_x_forward_z");
    DirectionToUnitVector(90.0 * kDeg, 0.0, x, y, z);
    CheckNear(y, 1.0, 1e-9, "unit_y_left");
    DirectionToUnitVector(0.0, 90.0 * kDeg, x, y, z);
    CheckNear(z, 1.0, 1e-9, "unit_z_up");

    // ---- RangeGrid.Sample：平面反深度双线性恢复精确值 ----
    const double D = 5.0;
    PlaneGrid plane(D, -0.1, 0.2, 16, -0.1, 0.2, 16);
    const double az_mid = plane.grid.angle_min + 0.5 * plane.grid.angle_step;
    const double el_mid = plane.grid.vertical_angle_min + 0.5 * plane.grid.vertical_step;
    const double want = D / (std::cos(el_mid) * std::cos(az_mid));
    CheckNear(plane.grid.Sample(az_mid, el_mid), want, 1e-3, "plane_bilinear_exact");

    // ---- 深度不连续 → 最近邻 ----
    PlaneGrid disc(D, -0.1, 0.2, 16, -0.1, 0.2, 16);
    const float bumped = disc.ranges[0] * 2.0F;
    disc.ranges[0] = bumped; // 角 (0,0) 深度翻倍，max/min=2>1.25
    const double az_near = disc.grid.angle_min + 0.1 * disc.grid.angle_step;
    const double el_near = disc.grid.vertical_angle_min + 0.1 * disc.grid.vertical_step;
    CheckNear(disc.grid.Sample(az_near, el_near), bumped, 1e-4, "discontinuity_nearest");

    // ---- FOV 外 / 未命中 → 负值 ----
    Check(plane.grid.Sample(plane.grid.angle_min - 1.0, 0.0) < 0.0, "out_of_fov_az");
    Check(plane.grid.Sample(0.0, plane.grid.vertical_angle_min - 1.0) < 0.0, "out_of_fov_el");
    PlaneGrid allmiss(D, -0.1, 0.2, 16, -0.1, 0.2, 16);
    for (auto& r : allmiss.ranges) {
        r = 200.0F; // 全部超出 range_max=100 → 无效
    }
    Check(allmiss.grid.Sample(az_mid, el_mid) < 0.0, "all_miss_negative");

    // ---- SampleFrame：命中点、offset_time、line、tag、模长 ----
    // 全 FOV 恒定距离网格，罩住 fixture 的 az∈[0,360)、el∈[-10,10]。
    std::vector<float> flat(64 * 16, 5.0F);
    RangeGrid g;
    g.angle_min = 0.0;
    g.angle_step = 2.0 * M_PI / 63;
    g.count = 64;
    g.vertical_angle_min = -0.2;
    g.vertical_step = 0.4 / 15;
    g.vertical_count = 16;
    g.range_min = 0.1;
    g.range_max = 100.0;
    g.ranges = flat.data();

    std::vector<SampledPoint> pts;
    const std::size_t n = SampleFrame(table, g, 0, 20, 5000, pts);
    Check(n == 20, "sampleframe_count_wraps_table");
    bool offset_ok = true;
    bool line_ok = true;
    bool tag_ok = true;
    bool norm_ok = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].offset_time_ns != i * 5000) {
            offset_ok = false;
        }
        if (pts[i].line != i % 4) {
            line_ok = false;
        }
        if (pts[i].tag != 0x10) {
            tag_ok = false;
        }
        const double norm = std::sqrt(static_cast<double>(pts[i].x) * pts[i].x +
                                      static_cast<double>(pts[i].y) * pts[i].y +
                                      static_cast<double>(pts[i].z) * pts[i].z);
        if (std::fabs(norm - 5.0) > 1e-3) {
            norm_ok = false;
        }
    }
    Check(offset_ok, "offset_time_i_times_interval");
    Check(line_ok, "line_i_mod_4_frame");
    Check(tag_ok, "tag_0x10");
    Check(norm_ok, "point_norm_equals_range");
    // 20000 点 @5000ns 的 offset_time 上界必须 < 0.1s（A2 的 [0,0.1) 覆盖）。
    Check(19999ULL * 5000ULL < 100000000ULL, "offset_coverage_under_100ms");

    // ---- SampleFrame：未命中点被丢弃 ----
    std::vector<float> halfmiss(64 * 16, 5.0F);
    for (int h = 0; h < 32; ++h) {
        for (int v = 0; v < 16; ++v) {
            halfmiss[v * 64 + h] = 200.0F; // 左半 FOV 无效
        }
    }
    RangeGrid g2 = g;
    g2.ranges = halfmiss.data();
    std::vector<SampledPoint> pts2;
    const std::size_t n2 = SampleFrame(table, g2, 0, 16, 5000, pts2);
    Check(n2 < 16, "misses_dropped");
    Check(n2 > 0, "some_hits_remain");

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
