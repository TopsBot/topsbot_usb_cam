// topsbot_jpeg_replay_node.cpp - JPEG file replay publisher (high-fps stress
// test): loops a JPEG file and publishes TbJpegFrame via loaned messages.
// fps=0 means "pump as fast as possible" (retry on borrow failure).
// With FASTDDS_DMABUF_BACKEND=1 the loaned segment is backed by carveout
// dmabuf; used to compare throughput with/without the dmabuf backend.
//
// 参数:
//   file       JPEG 文件路径（默认 /tmp/frame.jpg）
//   topic      发布话题（默认 tbmem_jpeg）
//   fps        目标帧率（默认 0 = 尽力推）
//   depth      QoS depth（默认 50，loaned 池大小）
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_jpeg_frame.hpp"

#include <BufferAllocator/BufferAllocatorWrapper.h>

using namespace std::chrono_literals;

namespace
{

}  // namespace

class JpegReplayNode : public rclcpp::Node
{
public:
  explicit JpegReplayNode(const rclcpp::NodeOptions & options)
  : Node("topsbot_jpeg_replay", options)
  {
    const std::string file = declare_parameter<std::string>("file", "/tmp/frame.jpg");
    const std::string topic = declare_parameter<std::string>("topic", "tbmem_jpeg");
    const int fps = declare_parameter<int>("fps", 0);
    const int depth = declare_parameter<int>("depth", 50);
    alloc_ = CreateDmabufHeapBufferAllocator();

    // 预加载 JPEG
    std::ifstream f(file, std::ios::binary);
    if (!f) {
      RCLCPP_FATAL(get_logger(), "cannot open %s", file.c_str());
      return;
    }
    jpeg_.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    RCLCPP_INFO(get_logger(), "loaded %s (%zu bytes), topic=%s fps=%d depth=%d",
                file.c_str(), jpeg_.size(), topic.c_str(), fps, depth);
    if (jpeg_.size() == 0 || jpeg_.size() > tb_img_msgs::msg::TbJpegFrame::MAX_SIZE) {
      RCLCPP_FATAL(get_logger(), "bad jpeg size %zu", jpeg_.size());
      return;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
    pub_ = create_publisher<tb_img_msgs::msg::TbJpegFrame>(topic, qos);

    if (fps > 0) {
      period_ = std::chrono::nanoseconds(1000000000LL / fps);
    }
    timer_ = create_wall_timer(20ms, [this]() { stats(); });
    worker_ = std::thread(&JpegReplayNode::run, this);
  }

  ~JpegReplayNode()
  {
    stop_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void stats()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_stats_).count();
    const uint64_t delta = frames_ - last_frames_;
    last_stats_ = now;
    last_frames_ = frames_;
    RCLCPP_INFO(get_logger(), "publish: %llu fps (total %llu, borrowed_fail %llu)",
                static_cast<unsigned long long>(delta / dt),
                static_cast<unsigned long long>(frames_),
                static_cast<unsigned long long>(borrow_fail_));
  }

  void run()
  {
    while (rclcpp::ok() && !stop_) {
      const auto t0 = std::chrono::steady_clock::now();
      auto loaned_msg = pub_->borrow_loaned_message();
      if (!loaned_msg.is_valid()) {
        ++borrow_fail_;
        std::this_thread::sleep_for(100us);
        continue;
      }
      auto & msg = loaned_msg.get();
      msg.width = 0;
      msg.height = 0;
      msg.encoding.fill(0);
      std::memcpy(msg.encoding.data(), "jpeg", 5);
      msg.data_size = static_cast<uint32_t>(jpeg_.size());
      std::memcpy(msg.data.data(), jpeg_.data(), jpeg_.size());
      const auto stamp = now_timestamp();
      msg.time_stamp.sec = stamp.first;
      msg.time_stamp.nanosec = stamp.second;
      msg.index = static_cast<int32_t>(frames_);

      uint64_t payload_phys = 0;
      if (alloc_ != nullptr) {
        DmabufHeapLookupPhysByPtr(alloc_, msg.data.data(), &payload_phys);
      }
      msg.phys_addr = payload_phys;
      // No explicit cache flush needed: the hardware consumer (JPEG decoder)
      // flushes the input fd globally (physical-address CMO) before DMA.
      // CPU-to-CPU metadata is covered by hardware cache coherence.

      pub_->publish(std::move(loaned_msg));
      ++frames_;

      if (period_.count() > 0) {
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < period_) {
          std::this_thread::sleep_for(period_ - elapsed);
        }
      }
    }
  }

  static std::pair<int32_t, uint32_t> now_timestamp()
  {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(now);
    const auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(now - secs);
    return {static_cast<int32_t>(secs.count()), static_cast<uint32_t>(nsecs.count())};
  }

  rclcpp::Publisher<tb_img_msgs::msg::TbJpegFrame>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> jpeg_;
  BufferAllocator * alloc_ = nullptr;
  std::chrono::nanoseconds period_{0};
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> frames_{0};
  std::atomic<uint64_t> borrow_fail_{0};
  std::chrono::steady_clock::time_point last_stats_{std::chrono::steady_clock::now()};
  uint64_t last_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JpegReplayNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
