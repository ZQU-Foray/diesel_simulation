// Livox Mid-360 非重复扫描仿真插件（gz system 插件，自持细网格 GpuRays）。
//
// 数据流：GpuRays(2048×336，渲染线程) → 进程内 Copy → livox_pattern 按官方角度表
// 采样 20000 点/帧 → rclcpp 直发 livox_ros_driver2/CustomMsg 到 /livox/lidar。
//
// 为什么这样设计（详见 decision.md D32 的实测取证）：
//  - gz RayQuery 两条路径都撑不住 200k 射线/秒（CPU 逐三角形 / GPU 每射线一次整场渲染）；
//  - GpuRays 是唯一批量投射机制，但只产均匀网格 → 用细网格 + 反深度双线性插值逼近；
//  - 进程内读 GpuRays 缓冲，绕开 LaserScan 2.75MB/帧的序列化+IPC（那会把帧率压到 9Hz）；
//  - 直发 CustomMsg，保住逐点 offset_time/line/tag，激活 point_lio 的 Livox 预处理路径。
//
// 线程纪律：渲染调用（CreateGpuRays/Update/Copy/SetWorldPose）只能在渲染线程
// （events::PostRender）；仿真时间节拍与 link 位姿读取在 PostUpdate（仿真线程），
// 两者经一把锁交换「该采帧了 + 此刻 link 位姿 + 仿真时刻」。
//
// 诚实限制：一帧 20000 点来自同一位姿的一次 GpuRays 快照，offset_time 是按 200k/s
// 合成的。角度图案是真非重复、Livox 路径被真实激活，但帧内运动畸变不是物理仿真
// （那需要帧内多位姿渲染，代价超预算，列为非目标）。
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/GpuRays.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <zlib.h>

#include "diesel_sim_livox/pattern.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace gzsim = ignition::gazebo;
namespace rendering = ignition::rendering;

namespace {
// 角度表加载：.gz 用 zlib 解压到内存再喂流式解析器，避免落临时文件；裸 CSV 直接读。
bool LoadPattern(diesel_sim_livox::PatternTable& table, const std::string& path) {
    if (path.size() >= 3 && path.compare(path.size() - 3, 3, ".gz") == 0) {
        gzFile f = gzopen(path.c_str(), "rb");
        if (f == nullptr) {
            return false;
        }
        std::string data;
        char buf[65536];
        int n = 0;
        while ((n = gzread(f, buf, sizeof(buf))) > 0) {
            data.append(buf, static_cast<std::size_t>(n));
        }
        gzclose(f);
        std::istringstream ss(data);
        return table.LoadCsv(ss);
    }
    return table.LoadCsv(path);
}
} // namespace

class LivoxMid360System : public gzsim::System,
                          public gzsim::ISystemConfigure,
                          public gzsim::ISystemPostUpdate {
  public:
    void Configure(const gzsim::Entity&, const std::shared_ptr<const sdf::Element>& sdf,
                   gzsim::EntityComponentManager&, gzsim::EventManager& eventMgr) override {
        this->topic = sdf->Get<std::string>("topic", "/livox/lidar").first;
        // diesel_localization 只订 sensor_msgs/PointCloud2（localization_node.cpp:191），
        // 与 point_lio 的 CustomMsg 不是同一类型，故同一帧点云再发一路 PointCloud2。
        this->pcTopic = sdf->Get<std::string>("pc_topic", "/livox/lidar_points").first;
        this->frameId = sdf->Get<std::string>("frame_id", "front_mid360").first;
        this->linkName = sdf->Get<std::string>("link", "front_mid360").first;
        this->patternCsv = sdf->Get<std::string>("pattern_csv", "").first;
        // 未显式给 pattern_csv 时，自找本包安装目录下的官方角度表（model.sdf 不写绝对路径，
        // 换安装位置/换机器人都无需改 SDF）。SDF 里给了就以 SDF 为准。
        if (this->patternCsv.empty()) {
            try {
                this->patternCsv =
                    ament_index_cpp::get_package_share_directory("diesel_sim_livox") +
                    "/scan_mode/mid360.csv.gz";
            } catch (const std::exception& e) {
                std::printf("[livox] 无法定位 diesel_sim_livox 角度表: %s\n", e.what());
            }
        }
        this->pointsPerFrame =
            static_cast<std::size_t>(sdf->Get<int>("points_per_frame", 20000).first);
        this->hCount = sdf->Get<int>("h_count", 2048).first;
        this->vCount = sdf->Get<int>("v_count", 336).first;
        this->updateRate = sdf->Get<double>("update_rate", 10.0).first;
        this->rangeMin = sdf->Get<double>("range_min", 0.1).first;
        this->rangeMax = sdf->Get<double>("range_max", 40.0).first;

        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        this->node = rclcpp::Node::make_shared("livox_mid360_sim");
        this->pub = this->node->create_publisher<livox_ros_driver2::msg::CustomMsg>(
            this->topic, rclcpp::SensorDataQoS());
        this->pubPc = this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->pcTopic, rclcpp::SensorDataQoS());

        if (this->patternCsv.empty() || !LoadPattern(this->table, this->patternCsv)) {
            std::printf("[livox] 角度表加载失败: '%s'（插件不会出点）\n", this->patternCsv.c_str());
            std::fflush(stdout);
        } else {
            std::printf("[livox] 角度表 %zu 点, 俯仰 [%.3f, %.3f] deg\n", this->table.size(),
                        this->table.elevation_min() * 180.0 / M_PI,
                        this->table.elevation_max() * 180.0 / M_PI);
            std::fflush(stdout);
        }

        this->sceneConn = eventMgr.Connect<gzsim::events::SceneUpdate>(
            std::bind(&LivoxMid360System::OnSceneUpdate, this));
        this->renderConn = eventMgr.Connect<gzsim::events::PostRender>(
            std::bind(&LivoxMid360System::OnPostRender, this));
        std::printf("[livox] configured topic=%s link=%s grid=%dx%d ppf=%zu\n", this->topic.c_str(),
                    this->linkName.c_str(), this->hCount, this->vCount, this->pointsPerFrame);
        std::fflush(stdout);
    }

    // 仿真线程：按仿真时间节拍决定「该采一帧」，并捕获此刻 link 世界位姿。不碰渲染。
    void PostUpdate(const gzsim::UpdateInfo& info,
                    const gzsim::EntityComponentManager& ecm) override {
        if (info.paused || this->table.empty()) {
            return;
        }
        const double simSec = std::chrono::duration<double>(info.simTime).count();
        if (simSec - this->lastFrame < 1.0 / this->updateRate) {
            return;
        }
        this->lastFrame = simSec;

        if (this->linkEntity == gzsim::kNullEntity) {
            ecm.Each<gzsim::components::Link, gzsim::components::Name>(
                [&](const gzsim::Entity& e, const gzsim::components::Link*,
                    const gzsim::components::Name* name) -> bool {
                    if (name->Data() == this->linkName) {
                        this->linkEntity = e;
                        return false;
                    }
                    return true;
                });
        }
        ignition::math::Pose3d pose;
        if (this->linkEntity == gzsim::kNullEntity) {
            // link 还没建好（实体创建顺序）：本帧不出点，避免从原点发误导性点云。
            return;
        }
        pose = gzsim::worldPose(this->linkEntity, ecm);

        std::lock_guard<std::mutex> lock(this->stateMutex);
        this->sharedPose = pose;
        this->sharedSimTime = simSec;
        this->wantFrame = true;
    }

  private:
    // 渲染线程：建 GpuRays（一次）。垂直 FOV 取角度表实际俯仰范围，罩住全部方向。
    void OnSceneUpdate() {
        if (this->rays) {
            return;
        }
        auto scene = rendering::sceneFromFirstRenderEngine();
        if (!scene) {
            return;
        }
        this->rays = scene->CreateGpuRays();
        this->rays->SetAngleMin(-M_PI);
        this->rays->SetAngleMax(M_PI);
        this->rays->SetVerticalAngleMin(this->table.elevation_min());
        this->rays->SetVerticalAngleMax(this->table.elevation_max());
        this->rays->SetRayCount(this->hCount);
        this->rays->SetVerticalRayCount(this->vCount);
        this->rays->SetNearClipPlane(0.05);
        this->rays->SetFarClipPlane(this->rangeMax + 20.0);
        this->rays->SetClamp(true);
        scene->RootVisual()->AddChild(this->rays);
        std::printf("[livox] GpuRays created range=%dx%d\n", this->rays->RangeCount(),
                    this->rays->VerticalRangeCount());
        std::fflush(stdout);
    }

    // 渲染线程：跟随 link 位姿渲染一帧，进程内采样并发 CustomMsg。
    void OnPostRender() {
        ignition::math::Pose3d pose;
        double simTime = 0.0;
        bool want = false;
        {
            std::lock_guard<std::mutex> lock(this->stateMutex);
            want = this->wantFrame;
            this->wantFrame = false;
            pose = this->sharedPose;
            simTime = this->sharedSimTime;
        }
        if (!want || !this->rays || !this->pub || this->table.empty()) {
            return;
        }

        this->rays->SetWorldPose(pose);
        this->rays->Update();

        const int h = this->rays->RangeCount();
        const int v = this->rays->VerticalRangeCount();
        const int n = h * v;
        const int ch = static_cast<int>(this->rays->Channels());
        // GpuRays::Copy 拷 w*h*Channels() 个 float（range/intensity/第三通道），
        // 只按 w*h 分配会溢出 3 倍直接段错误。
        if (static_cast<int>(this->raw.size()) != n * ch) {
            this->raw.resize(n * ch);
        }
        this->rays->Copy(this->raw.data());

        // 抽出距离通道成连续数组，喂纯核心采样。
        if (static_cast<int>(this->ranges.size()) != n) {
            this->ranges.resize(n);
        }
        for (int i = 0; i < n; ++i) {
            this->ranges[i] = this->raw[i * ch];
        }

        diesel_sim_livox::RangeGrid grid;
        grid.angle_min = this->rays->AngleMin().Radian();
        grid.angle_step = (this->rays->AngleMax().Radian() - grid.angle_min) / (h - 1);
        grid.count = h;
        grid.vertical_angle_min = this->rays->VerticalAngleMin().Radian();
        grid.vertical_step =
            (this->rays->VerticalAngleMax().Radian() - grid.vertical_angle_min) / (v - 1);
        grid.vertical_count = v;
        grid.ranges = this->ranges.data();
        grid.range_min = this->rangeMin;
        grid.range_max = this->rangeMax;

        // 逐点时间戳铺满一帧周期：interval = (1/updateRate) / pointsPerFrame。
        // 默认 10Hz × 20000 点 = 200k/s ⇒ 5000 ns，19999×5000 < 1e8（覆盖 [0,0.1)s）。
        const std::uint32_t intervalNs =
            static_cast<std::uint32_t>(1e9 / (this->updateRate * this->pointsPerFrame));
        const std::size_t got = diesel_sim_livox::SampleFrame(
            this->table, grid, this->frameIndex, this->pointsPerFrame, intervalNs, this->pts);

        const int64_t simNs = static_cast<int64_t>(simTime * 1e9);
        livox_ros_driver2::msg::CustomMsg msg;
        msg.header.stamp = rclcpp::Time(simNs);
        msg.header.frame_id = this->frameId;
        msg.timebase = static_cast<uint64_t>(simNs);
        msg.point_num = static_cast<uint32_t>(got);
        msg.lidar_id = 0;
        msg.points.reserve(got);
        for (const auto& p : this->pts) {
            livox_ros_driver2::msg::CustomPoint cp;
            cp.offset_time = p.offset_time_ns;
            cp.x = p.x;
            cp.y = p.y;
            cp.z = p.z;
            cp.reflectivity = static_cast<uint8_t>(p.reflectivity);
            cp.tag = p.tag;
            cp.line = p.line;
            msg.points.push_back(cp);
        }
        this->pub->publish(msg);

        // 同一帧点云再发一路 PointCloud2 给 diesel_localization（它只订 PointCloud2）。
        // 与 CustomMsg 同 stamp/同 frame_id/同点集，保证两路消费者看到同一帧。
        if (this->pubPc) {
            sensor_msgs::msg::PointCloud2 pc;
            pc.header.stamp = msg.header.stamp;
            pc.header.frame_id = this->frameId;
            pc.height = 1;
            pc.width = static_cast<uint32_t>(got);
            pc.is_dense = true;
            sensor_msgs::PointCloud2Modifier modifier(pc);
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                          sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                          sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                          sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(got);
            sensor_msgs::PointCloud2Iterator<float> ix(pc, "x");
            sensor_msgs::PointCloud2Iterator<float> iy(pc, "y");
            sensor_msgs::PointCloud2Iterator<float> iz(pc, "z");
            sensor_msgs::PointCloud2Iterator<float> ii(pc, "intensity");
            for (const auto& p : this->pts) {
                *ix = p.x;
                *iy = p.y;
                *iz = p.z;
                *ii = p.reflectivity;
                ++ix;
                ++iy;
                ++iz;
                ++ii;
            }
            this->pubPc->publish(pc);
        }
        ++this->frameIndex;

        if (this->frameIndex % 50 == 1) {
            std::printf("[livox] frame=%zu points=%zu/%zu sim=%.2f\n", this->frameIndex, got,
                        this->pointsPerFrame, simTime);
            std::fflush(stdout);
        }
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPc;
    ignition::common::ConnectionPtr sceneConn;
    ignition::common::ConnectionPtr renderConn;
    rendering::GpuRaysPtr rays;

    diesel_sim_livox::PatternTable table;
    std::vector<float> raw;
    std::vector<float> ranges;
    std::vector<diesel_sim_livox::SampledPoint> pts;

    std::string topic;
    std::string pcTopic;
    std::string frameId;
    std::string linkName;
    std::string patternCsv;
    std::size_t pointsPerFrame{20000};
    int hCount{2048};
    int vCount{336};
    double updateRate{10.0};
    double rangeMin{0.1};
    double rangeMax{40.0};

    gzsim::Entity linkEntity{gzsim::kNullEntity};
    double lastFrame{-1.0};
    std::size_t frameIndex{0};

    std::mutex stateMutex;
    ignition::math::Pose3d sharedPose;
    double sharedSimTime{0.0};
    bool wantFrame{false};
};

IGNITION_ADD_PLUGIN(LivoxMid360System, gzsim::System, LivoxMid360System::ISystemConfigure,
                    LivoxMid360System::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(LivoxMid360System, "diesel::LivoxMid360System")
