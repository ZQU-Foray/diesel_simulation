// Copyright 2026 ZQU-Foray
//
// 仿真侧的「下位机过渡桥」：把 ROS 侧的底盘速度编成 RoboLink v0.2 控制快照，
// 经虚拟 USB CDC 交给 foray_mcu 的 host_sim（固件源码在 PC 上跑，README §3.2）；
// 只有固件回了状态帧、确认收到这次控制之后，才把速度写进 gz 车体。
//
// 为什么需要这一层（decision.md D30）：固件目前没有 chassis 能力，收到快照只记账，
// 底盘执行得由仿真侧顶替。等电控组补上 chassis 与 CAN 电机，这一层退化成
// 「CAN 电机替身」——协议侧不再有替身。
//
// 链路状态发布到 `<cmd_vel 话题同级>/mcu_link/status`（`key=value` 文本），供
// `accept.py link`（README A11）读取：link_state / hello_rtt_ms / status_frames /
// frame_errors / controls。

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

#include "diesel/platform/link/robolink_codec.hpp"
#include "diesel/platform/link/robolink_host_link.hpp"
#include "diesel/platform/link/usb_cdc_port.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

// 控制快照的有效期提示：上位机每 20 ms 发一次，50 ms 覆盖两次发送的抖动；
// 与固件侧上限（50 ms）一致（robolink v0.2 §4.7）。
constexpr std::uint16_t VALIDITY_HINT_MS = 50U;

// 控制快照的 flags bit0 = 底盘部分有效（固件 app/topics/pc_link.hpp 的口径）。
constexpr std::uint16_t FLAG_CHASSIS = 0x0001U;

// 控制来源与模式：PC 下发、自动导航（固件 ControlSource::PC / Mode::AUTO_NAV）。
constexpr std::uint8_t SOURCE_PC = 2U;
constexpr std::uint8_t MODE_AUTO_NAV = 2U;

// 指令超时：Nav2 停发后不能拿旧速度一直开下去；与链路失联门限（500 ms）同量级。
constexpr int64_t COMMAND_TIMEOUT_MS = 500;

// 固件 SYSTEM_STATUS 的 link_state 取值（robolink v0.2：0 断开 / 1 USB / 2 READY / 3 ACTIVE）。
constexpr std::uint8_t LINK_STATE_ACTIVE = 3U;

// 主循环周期：20 ms（50 Hz），与上位机的 HELLO/心跳节奏同一量级。
constexpr int64_t LOOP_PERIOD_MS = 20;

// 状态话题的发布周期。
//
// 10 Hz 是给验收留的测量分辨率：A11 要断言「停掉一端后 500 ms 内判活失败」，
// 状态行之间隔得越久，能测到的失联检出时间越粗（1 Hz 时量化误差就有 1 s）。
constexpr int64_t STATUS_PERIOD_MS = 100;

std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string state_name(diesel::platform::link::HostLinkState state) {
    using diesel::platform::link::HostLinkState;
    switch (state) {
    case HostLinkState::Disconnected:
        return "Disconnected";
    case HostLinkState::HelloPending:
        return "HelloPending";
    case HostLinkState::Ready:
        return "Ready";
    case HostLinkState::Active:
        return "Active";
    }
    return "Unknown";
}

} // namespace

namespace diesel_sim_mcu {

class SimMcuBridge : public rclcpp::Node {
  public:
    SimMcuBridge()
        : rclcpp::Node("sim_mcu_bridge"),
          command_topic_(declare_parameter<std::string>("cmd_vel_topic", "/diesel/sentry/cmd_vel")),
          gz_topic_(declare_parameter<std::string>("gz_topic", "/model/diesel_sim_robot/cmd_vel")),
          port_(declare_parameter<std::string>("port", "/tmp/diesel_host")), host_(port_),
          command_timeout_ms_(declare_parameter<int>("command_timeout_ms", COMMAND_TIMEOUT_MS)) {
        gz_publisher_ = gz_node_.Advertise<ignition::msgs::Twist>(gz_topic_);

        command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            command_topic_, rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Twist::SharedPtr message) { on_command(*message); });

        // 状态话题跟着 cmd_vel 的命名空间走，验收脚本按同一个 namespace 找得到它。
        const auto slash = command_topic_.find_last_of('/');
        const std::string prefix =
            slash == std::string::npos ? std::string() : command_topic_.substr(0, slash);
        status_publisher_ =
            create_publisher<std_msgs::msg::String>(prefix + "/mcu_link/status", 10);

        const auto start = now_ms();
        last_command_ms_ = start;
        last_status_publish_ms_ = start;

        timer_ = create_wall_timer(std::chrono::milliseconds(LOOP_PERIOD_MS), [this]() { step(); });

        RCLCPP_INFO(get_logger(), "过渡桥已起：port=%s ros=%s gz=%s", port_path().c_str(),
                    command_topic_.c_str(), gz_topic_.c_str());
    }

  private:
    std::string port_path() const {
        return get_parameter("port").as_string();
    }

    void on_command(const geometry_msgs::msg::Twist& message) {
        command_ = message;
        last_command_ms_ = now_ms();
        ++command_count_;
    }

    /// 把速度写进 gz 车体。链路断开时也必须走这里写 0，否则车会按最后一条指令一直开。
    void publish_body(double vx, double vy, double wz) {
        ignition::msgs::Twist message;
        message.mutable_linear()->set_x(vx);
        message.mutable_linear()->set_y(vy);
        message.mutable_angular()->set_z(wz);
        gz_publisher_.Publish(message);
    }

    diesel::platform::link::ControlSnapshot
    snapshot(const geometry_msgs::msg::Twist& command) const {
        diesel::platform::link::ControlSnapshot snapshot;
        snapshot.source = SOURCE_PC;
        snapshot.mode = MODE_AUTO_NAV;
        snapshot.flags = FLAG_CHASSIS;
        snapshot.chassis_vx = static_cast<float>(command.linear.x);
        snapshot.chassis_vy = static_cast<float>(command.linear.y);
        snapshot.chassis_wz = static_cast<float>(command.angular.z);
        snapshot.validity_hint_ms = VALIDITY_HINT_MS;
        return snapshot;
    }

    /// 固件是否确认收到了我们的控制。
    ///
    /// 判据只用**固件自己报的**两个字段：`link_state == Active` 与
    /// `last_rx_control_seq > 0`（它确实收到过控制快照）。不用
    /// `control_msg_age_ms`——固件第一版是应用层填这个字段，`app/pc_link` 目前只记账，
    /// 它恒为初值 `0xFFFF`（"没有快照"），拿它当判据会让闸门永远关着。
    /// 也不用「回读序号 ≥ 已发条数」：序号回读天然滞后一拍，链路丢一帧就永远追不上。
    bool firmware_confirms() const {
        if (queued_controls_ == 0U || !host_.latest_status().has_value()) {
            return false;
        }
        const auto& status = *host_.latest_status();
        return status.link_state == LINK_STATE_ACTIVE && status.last_rx_control_seq > 0U;
    }

    void step() {
        const auto now = now_ms();
        const auto previous_state = last_state_;
        const auto state = host_.step(static_cast<std::uint64_t>(now));
        last_state_ = state;

        // 失联检出延迟：从「最后一次收到固件状态帧」到「上位机状态机判定断开」。
        // A10 要求停任一端后 500 ms 内判活失败，这个值就是那条判据的实测量——判活门限
        // 是 500 ms，加上 20 ms 的循环粒度，实测应在 500–520 ms 之间。
        if (previous_state == diesel::platform::link::HostLinkState::Active &&
            state == diesel::platform::link::HostLinkState::Disconnected &&
            last_status_rx_ms_ >= 0) {
            drop_detect_ms_ = now - last_status_rx_ms_;
        }

        // 握手耗时从「第一次发出 HELLO」量到「对端应答」，不是从进程启动量：
        // 固件可能比桥晚起几秒，那段时间不该算进握手延迟。
        using diesel::platform::link::HostLinkState;
        if (state != HostLinkState::Disconnected && hello_started_ms_ < 0) {
            hello_started_ms_ = now;
        }
        // 固件在 READY 时会立刻发一帧完整状态，所以同一次 step 里就可能从
        // HelloPending 直接走到 Active，`Ready` 不一定被单独观察到；两个状态都算握手完成。
        const bool handshake_done = state == HostLinkState::Ready || state == HostLinkState::Active;
        if (handshake_done && hello_rtt_ms_ < 0 && hello_started_ms_ >= 0) {
            hello_rtt_ms_ = now - hello_started_ms_;
        }

        const bool command_fresh = now - last_command_ms_ <= command_timeout_ms_;
        const auto command = command_fresh ? command_ : geometry_msgs::msg::Twist();

        // 只有 Active 才允许发控制：Active 的含义是固件已经回过状态帧。
        if (state == diesel::platform::link::HostLinkState::Active) {
            if (host_.queue_control(snapshot(command), static_cast<std::uint64_t>(now))) {
                ++queued_controls_;
            }
        }

        if (firmware_confirms() && command_fresh) {
            publish_body(command.linear.x, command.linear.y, command.angular.z);
        } else {
            publish_body(0.0, 0.0, 0.0);
        }

        // 状态帧的到达要按 50 Hz 的循环采样统计，不能只挂在 10 Hz 的发布上：
        // 挂在发布上时，两次发布之间的新帧会被算成「这一拍只来了一帧」，速率恒等于发布频率。
        observe_status_frames(now);
        publish_status(now, state);
    }

    /// 数固件状态帧：`latest_status()` 在两次收帧之间不变，所以「uptime 变了」才算新帧。
    void observe_status_frames(std::int64_t now) {
        if (!host_.latest_status().has_value()) {
            return;
        }
        const auto& status = *host_.latest_status();
        if (status.uptime_ms == last_status_uptime_ms_) {
            return;
        }
        last_status_uptime_ms_ = status.uptime_ms;
        last_status_rx_ms_ = now;
        ++status_frames_;
    }

    void publish_status(std::int64_t now, diesel::platform::link::HostLinkState state) {
        if (now - last_status_publish_ms_ < STATUS_PERIOD_MS) {
            return;
        }
        last_status_publish_ms_ = now;

        std::uint32_t crc_errors = 0U;
        if (host_.latest_status().has_value() && host_.latest_status()->diagnostics.has_value()) {
            crc_errors = host_.latest_status()->diagnostics->crc_errors;
        }

        // 状态帧速率：按发布周期统计新到达的帧数。桥的主循环是 50 Hz，所以量到的是
        // 「桥实际观察到的状态帧速率」（固件本身发 200 Hz），断流时掉到 0——
        // 这正是 A11 要抓的现象。
        const double status_hz =
            static_cast<double>(status_frames_ - last_reported_status_frames_) * 1000.0 /
            static_cast<double>(STATUS_PERIOD_MS);
        last_reported_status_frames_ = status_frames_;

        std_msgs::msg::String message;
        message.data =
            "link_state=" + state_name(state) + " hello_rtt_ms=" +
            (hello_rtt_ms_ < 0 ? std::string("none") : std::to_string(hello_rtt_ms_)) +
            " status_frames=" + std::to_string(status_frames_) +
            " status_hz=" + std::to_string(static_cast<int>(status_hz)) +
            " frame_errors=" + std::to_string(crc_errors) +
            " controls=" + std::to_string(command_count_) + " drop_detect_ms=" +
            (drop_detect_ms_ < 0 ? std::string("none") : std::to_string(drop_detect_ms_));
        status_publisher_->publish(message);
    }

    const std::string command_topic_;
    const std::string gz_topic_;
    diesel::platform::link::UsbCdcPort port_;
    diesel::platform::link::RoboLinkHostLink host_;
    int command_timeout_ms_;

    ignition::transport::Node gz_node_;
    ignition::transport::Node::Publisher gz_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist command_;
    std::int64_t last_command_ms_ = 0;
    std::int64_t last_status_publish_ms_ = 0;
    std::int64_t hello_started_ms_ = -1;
    std::int64_t hello_rtt_ms_ = -1;
    std::uint32_t last_status_uptime_ms_ = 0U;
    diesel::platform::link::HostLinkState last_state_ =
        diesel::platform::link::HostLinkState::Disconnected;
    std::int64_t last_status_rx_ms_ = -1;
    std::int64_t drop_detect_ms_ = -1;
    std::uint32_t status_frames_ = 0U;
    std::uint32_t last_reported_status_frames_ = 0U;
    std::uint32_t command_count_ = 0U;
    std::uint32_t queued_controls_ = 0U;
};

} // namespace diesel_sim_mcu

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<diesel_sim_mcu::SimMcuBridge>());
    rclcpp::shutdown();
    return 0;
}
