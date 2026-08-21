// topsbot_img_sink_node.cpp - lightweight NV12 consumer (throughput test).
//
// Subscribes TbMsg480P/540P/1080P (loaned, per profile) and only counts
// frames/rate, exposing throughput differences for large raw frames.
#include <atomic>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_msg1080_p.hpp"
#include "tb_img_msgs/msg/tb_msg480_p.hpp"
#include "tb_img_msgs/msg/tb_msg540_p.hpp"

using namespace std::chrono_literals;

class ImgSinkNode : public rclcpp::Node
{
public:
  explicit ImgSinkNode(const rclcpp::NodeOptions & options)
  : Node("topsbot_img_sink", options)
  {
    const std::string profile = declare_parameter<std::string>("profile", "540p");
    const std::string topic = declare_parameter<std::string>("topic", "tbmem_img");
    const int depth = declare_parameter<int>("depth", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
    if (profile == "480p") {
      sub_480_ = create_subscription<tb_img_msgs::msg::TbMsg480P>(
        topic, qos,
        [this](const tb_img_msgs::msg::TbMsg480P::ConstSharedPtr & msg) {
          check_size(*msg, msg->data_size, 640 * 480 * 3 / 2);
        });
    } else if (profile == "1080p") {
      sub_1080_ = create_subscription<tb_img_msgs::msg::TbMsg1080P>(
        topic, qos,
        [this](const tb_img_msgs::msg::TbMsg1080P::ConstSharedPtr & msg) {
          check_size(*msg, msg->data_size, 1920 * 1080 * 3 / 2);
        });
    } else {
      sub_540_ = create_subscription<tb_img_msgs::msg::TbMsg540P>(
        topic, qos,
        [this](const tb_img_msgs::msg::TbMsg540P::ConstSharedPtr & msg) {
          check_size(*msg, msg->data_size, 960 * 540 * 3 / 2);
        });
    }
    RCLCPP_INFO(get_logger(), "profile=%s topic=%s", profile.c_str(), topic.c_str());

    timer_ = create_wall_timer(2s, [this]() {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - last_).count();
      const uint64_t delta = frames_ - last_frames_;
      last_ = now;
      last_frames_ = frames_;
      RCLCPP_INFO(get_logger(), "receive: %llu fps (total %llu)",
                  static_cast<unsigned long long>(delta / dt),
                  static_cast<unsigned long long>(frames_));
    });
  }

private:
  // Checksum over the full NV12 payload (real-consumer read behaviour).
  // With file replay every frame is identical, so the checksum must stay
  // constant — this verifies DMA-written data is read back correctly.
  template<typename MsgT>
  uint32_t checksum(const MsgT & msg, size_t n)
  {
    uint32_t sum = 0;
    const auto * d = msg.data.data();
    for (size_t i = 0; i < n; ++i) {
      sum += d[i];
    }
    return sum;
  }

  template<typename MsgT>
  void check_size(const MsgT & msg, uint32_t data_size, size_t expected)
  {
    const uint64_t f = frames_.fetch_add(1);
    const uint32_t cs = checksum(msg, std::min<size_t>(data_size, expected));
    if ((f % 500) == 0) {
      RCLCPP_INFO(get_logger(), "frame %llu data_size=%u (expect %zu) checksum=%u",
                  static_cast<unsigned long long>(f), data_size, expected, cs);
    }
  }

  rclcpp::Subscription<tb_img_msgs::msg::TbMsg480P>::SharedPtr sub_480_;
  rclcpp::Subscription<tb_img_msgs::msg::TbMsg540P>::SharedPtr sub_540_;
  rclcpp::Subscription<tb_img_msgs::msg::TbMsg1080P>::SharedPtr sub_1080_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> frames_{0};
  std::chrono::steady_clock::time_point last_{std::chrono::steady_clock::now()};
  uint64_t last_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgSinkNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
