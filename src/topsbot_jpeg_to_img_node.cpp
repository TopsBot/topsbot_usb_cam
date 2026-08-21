// topsbot_jpeg_to_img_node.cpp - JPEG decode-and-forward node (zero-copy).
//
// Subscribes TbJpegFrame (loaned, phys_addr filled by the publisher) →
// hardware decode (ta_cv_image_jpeg_dec) → decode output written directly
// into a loaned TbMsg payload (wrapped as an fd via LOOKUP for the decoder)
// → publish TbMsg (loaned + phys_addr) → downstream receives zero-copy.
//
// Zero-copy points:
//   input:  loaned subscription + LOOKUP(phys_addr) fd, decoder DMA-reads JPEG
//   decode: decoder DMA-writes the loaned payload directly, no memcpy
//   output: loaned publish of TbMsg with phys_addr
//
// Parameters:
//   profile   480p | 540p | 1080p (output TbMsg type, default 1080p)
//   in_topic  JPEG input topic (default tbmem_jpeg)
//   out_topic NV12 output topic (default tbmem_img)
//   depth     QoS depth (default 5; loaned pool = segment memory x depth)
//   fallback_heap  dma-heap used for the copy fallback path
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_jpeg_frame.hpp"
#include "tb_img_msgs/msg/tb_msg1080_p.hpp"
#include "tb_img_msgs/msg/tb_msg480_p.hpp"
#include "tb_img_msgs/msg/tb_msg540_p.hpp"

#include "ta_cv_api_ext_c.h"
#include "BufferAllocator/BufferAllocatorWrapper.h"

using namespace std::chrono_literals;


namespace
{

struct ImgSpec
{
  int width;
  int height;
  size_t nv12_size;
  const char * profile;
};

ImgSpec spec_for(const std::string & profile)
{
  if (profile == "480p") {
    return {640, 480, 640 * 480 * 3 / 2, "480p"};
  }
  if (profile == "540p") {
    return {960, 540, 960 * 540 * 3 / 2, "540p"};
  }
  return {1920, 1080, 1920 * 1080 * 3 / 2, "1080p"};
}

// JPEG 头解析：SOF marker 里取真实宽高（与 topsbot_cv 同款逻辑）
int parse_jpeg_dimensions(
  const unsigned char * jpeg_data, const size_t jpeg_size, int * width, int * height)
{
  if (!jpeg_data || jpeg_size < 4 || !width || !height) {
    return -1;
  }
  size_t pos = 0;
  if (jpeg_data[0] == 0xFF && jpeg_data[1] == 0xD8) {
    pos = 2;
  }
  while (pos + 1 < jpeg_size) {
    if (jpeg_data[pos] != 0xFF) {
      ++pos;
      continue;
    }
    ++pos;
    while (pos < jpeg_size && jpeg_data[pos] == 0xFF) {
      ++pos;
    }
    if (pos >= jpeg_size) {
      break;
    }
    const unsigned char marker = jpeg_data[pos++];
    if (marker == 0xD9) {
      break;
    }
    if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
      continue;
    }
    if (pos + 2 > jpeg_size) {
      break;
    }
    const uint16_t seg_len =
      (static_cast<uint16_t>(jpeg_data[pos]) << 8) | jpeg_data[pos + 1];
    if (seg_len < 2 || pos + seg_len > jpeg_size) {
      break;
    }
    const bool sof = (marker >= 0xC0 && marker <= 0xC3) ||
                     (marker >= 0xC5 && marker <= 0xCB) || marker == 0xC9;
    if (sof && seg_len >= 7) {
      *height = (static_cast<int>(jpeg_data[pos + 3]) << 8) | jpeg_data[pos + 4];
      *width = (static_cast<int>(jpeg_data[pos + 5]) << 8) | jpeg_data[pos + 6];
      return 0;
    }
    pos += seg_len;
  }
  return -1;
}

}  // namespace

class JpegToImgNode : public rclcpp::Node
{
public:
  explicit JpegToImgNode(const rclcpp::NodeOptions & options)
  : Node("topsbot_jpeg_to_img", options)
  {
    const std::string profile = declare_parameter<std::string>("profile", "1080p");
    const std::string in_topic = declare_parameter<std::string>("in_topic", "tbmem_jpeg");
    const std::string out_topic = declare_parameter<std::string>("out_topic", "tbmem_img");
    const int depth = declare_parameter<int>("depth", 5);
    fallback_heap_ = declare_parameter<std::string>(
      "fallback_heap", "carveout-heap");  // dma-heap for the copy fallback

    spec_ = spec_for(profile);
    const char * be = std::getenv("FASTDDS_DMABUF_BACKEND");
    dmabuf_mode_ = (be != nullptr && strcmp(be, "1") == 0);
    RCLCPP_INFO(get_logger(), "backend=%s (FASTDDS_DMABUF_BACKEND=%s)",
                dmabuf_mode_ ? "dmabuf/carveout" : "native/shared-mem",
                be != nullptr ? be : "unset");
    RCLCPP_INFO(get_logger(),
                "profile=%s (%dx%d, %zu B) in=%s out=%s depth=%d fallback_heap=%s",
                spec_.profile, spec_.width, spec_.height, spec_.nv12_size,
                in_topic.c_str(), out_topic.c_str(), depth,
                fallback_heap_.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
    // Input: loaned TbJpegFrame subscription
    jpeg_sub_ = create_subscription<tb_img_msgs::msg::TbJpegFrame>(
      in_topic, qos,
      [this](const tb_img_msgs::msg::TbJpegFrame::ConstSharedPtr & msg) {
        on_jpeg(msg);
      });

    // Output: loaned TbMsg* publisher (per profile)
    if (profile == "480p") {
      pub_480_ = create_publisher<tb_img_msgs::msg::TbMsg480P>(out_topic, qos);
    } else if (profile == "540p") {
      pub_540_ = create_publisher<tb_img_msgs::msg::TbMsg540P>(out_topic, qos);
    } else {
      pub_1080_ = create_publisher<tb_img_msgs::msg::TbMsg1080P>(out_topic, qos);
    }
  }

  ~JpegToImgNode() = default;

private:
  template<typename MsgT>
  void decode_and_publish(
    const typename rclcpp::Publisher<MsgT>::SharedPtr & pub,
    const tb_img_msgs::msg::TbJpegFrame::ConstSharedPtr & jpeg)
  {
    // 0. Parse the real JPEG dimensions from the header; the decoder sizes
    //    the output image from them, so a fixed profile size would mismatch.
    int jpeg_w = 0;
    int jpeg_h = 0;
    if (parse_jpeg_dimensions(jpeg->data.data(), jpeg->data_size, &jpeg_w, &jpeg_h) != 0 ||
        jpeg_w <= 0 || jpeg_h <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cannot parse JPEG dims (size=%u)", jpeg->data_size);
      return;
    }
    const size_t nv12 = static_cast<size_t>(jpeg_w) * jpeg_h * 3 / 2;
    if (nv12 > spec_.nv12_size) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "JPEG %dx%d exceeds profile %s capacity",
                           jpeg_w, jpeg_h, spec_.profile);
      return;
    }

    // 1. Input fd: only try LOOKUP in dmabuf mode (without the carveout
    //    backend there is no registrable phys; go straight to the copy path).
    int in_fd = -1;
    if (dmabuf_mode_ && jpeg->phys_addr != 0) {
      uint64_t len = jpeg->data_size;
      in_fd = DmabufHeapLookupByPhys(alloc_, jpeg->phys_addr, &len);
      if (in_fd < 0) {
        // LOOKUP failed (e.g. segment not carveout-backed): downgrade to
        // the copy path for subsequent frames
        dmabuf_mode_ = false;
        RCLCPP_INFO(get_logger(),
                    "input LOOKUP failed (phys=0x%llx): downgraded to native copy path",
                    static_cast<unsigned long long>(jpeg->phys_addr));
      }
    }
    // Fallback: copy the JPEG into a fresh carveout buffer when phys_addr
    // is unavailable
    int in_fd_fb = -1;
    if (in_fd < 0) {
      in_fd_fb = DmabufHeapAlloc(alloc_, "carveout-heap", jpeg->data_size);
      if (in_fd_fb >= 0) {
        void * v = mmap(nullptr, jpeg->data_size, PROT_READ | PROT_WRITE, MAP_SHARED, in_fd_fb, 0);
        if (v != MAP_FAILED) {
          memcpy(v, jpeg->data.data(), jpeg->data_size);
          munmap(v, jpeg->data_size);
        }
      }
      in_fd = in_fd_fb;
    }
    if (in_fd < 0) {
      return;
    }

    // 2. Borrow the loaned output payload
    auto loaned = pub->borrow_loaned_message();
    if (!loaned.is_valid()) {
      close(in_fd);
      return;
    }
    auto & msg = loaned.get();

    // 2.5 Touch every payload page (read-only): the decoder DMA-writes the
    //     payload without creating CPU page-table entries, and pagemap-based
    //     phys resolution requires the pages to have been CPU-accessed.
    {
      volatile unsigned char * base = msg.data.data();
      for (size_t off = 0; off < nv12; off += 4096) {
        (void)base[off];  // volatile read forces page-table entry creation
      }
    }

    // 3. Output fd: in dmabuf mode LOOKUP the loaned payload so the decoder
    //    writes it directly (zero-copy); otherwise decode into a temporary
    //    buffer and copy into the payload afterwards.
    int out_fd = -1;
    int tmp_fd = -1;
    uint64_t m1_phys = 0;
    if (dmabuf_mode_) {
      DmabufHeapLookupPhysByPtr(alloc_, msg.data.data(), &m1_phys);
      if (m1_phys != 0) {
        uint64_t len = nv12;
        out_fd = DmabufHeapLookupByPhys(alloc_, m1_phys, &len);
      }
      if (out_fd < 0) {
        dmabuf_mode_ = false;
        RCLCPP_INFO(get_logger(),
                    "M1 LOOKUP failed (phys=0x%llx): downgraded to native copy path",
                    static_cast<unsigned long long>(m1_phys));
      }
    }
    if (out_fd < 0) {
      tmp_fd = DmabufHeapAlloc(alloc_, fallback_heap_.c_str(), nv12);
      out_fd = tmp_fd;
    }
    if (out_fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "output buffer unavailable (m1 phys=0x%llx)",
                           static_cast<unsigned long long>(m1_phys));
      close(in_fd);
      return;
    }

    // 4. Decode: bind the output image to the loaned payload fd
    ta_image_t img{};
    int ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_NV12, &img, out_fd);
    if (ret == 0) {
      ret = ta_cv_image_jpeg_dec(jpeg->data_size, in_fd, &img);
      ta_cv_image_destroy_ext(&img);
    }
    if (tmp_fd >= 0) {
      // Native SHM path: decode into a temporary carveout buffer, then
      // copy into the payload (CPU write, no cache sync needed)
      ++stats_fallback_;
      void * tmp_v = mmap(nullptr, nv12, PROT_READ, MAP_SHARED, tmp_fd, 0);
      if (tmp_v != MAP_FAILED) {
        std::memcpy(msg.data.data(), tmp_v, nv12);
        munmap(tmp_v, nv12);
      }
    } else {
      ++stats_zero_copy_;
    }
    if ((stats_zero_copy_ + stats_fallback_) % 500 == 0) {
      RCLCPP_INFO(get_logger(),
                  "path stats: zero_copy=%lu fallback_copy=%lu (%.1f%% zero-copy)",
                  static_cast<unsigned long>(stats_zero_copy_),
                  static_cast<unsigned long>(stats_fallback_),
                  stats_zero_copy_ * 100.0 /
                    static_cast<double>(stats_zero_copy_ + stats_fallback_));
    }
    close(out_fd);
    close(in_fd);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "decode failed (%d) size=%u phys=0x%llx", ret,
                           jpeg->data_size, static_cast<unsigned long long>(jpeg->phys_addr));
      return;
    }

    // 5. Fill payload fields + phys_addr
    msg.width = static_cast<uint32_t>(jpeg_w);
    msg.height = static_cast<uint32_t>(jpeg_h);
    msg.step = static_cast<uint32_t>(jpeg_w);
    msg.data_size = static_cast<uint32_t>(nv12);
    std::memset(msg.encoding.data(), 0, msg.encoding.size());
    std::memcpy(msg.encoding.data(), "nv12", 5);
    msg.time_stamp = jpeg->time_stamp;
    msg.index = frame_index_++;
    msg.phys_addr = m1_phys;

    // 6. No explicit cache invalidate needed: the hardware decoder
    //    invalidates the output fd globally (physical-address CMO) after
    //    DMA, covering stale lines in consumer processes.
    if (!dmabuf_mode_) {
      msg.phys_addr = 0;
    } else {
      msg.phys_addr = m1_phys;
    }

    pub->publish(std::move(loaned));
  }

  void on_jpeg(const tb_img_msgs::msg::TbJpegFrame::ConstSharedPtr & jpeg)
  {
    if (jpeg->data_size == 0) {
      return;
    }
    if (!alloc_) {
      alloc_ = CreateDmabufHeapBufferAllocator();
    }
    if (!alloc_) {
      return;
    }
    if (pub_480_) {
      decode_and_publish<tb_img_msgs::msg::TbMsg480P>(pub_480_, jpeg);
    } else if (pub_540_) {
      decode_and_publish<tb_img_msgs::msg::TbMsg540P>(pub_540_, jpeg);
    } else if (pub_1080_) {
      decode_and_publish<tb_img_msgs::msg::TbMsg1080P>(pub_1080_, jpeg);
    }
  }

  ImgSpec spec_{};
  std::string fallback_heap_;
  bool dmabuf_mode_ = false;
  BufferAllocator * alloc_ = nullptr;
  rclcpp::Subscription<tb_img_msgs::msg::TbJpegFrame>::SharedPtr jpeg_sub_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg480P>::SharedPtr pub_480_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg540P>::SharedPtr pub_540_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg1080P>::SharedPtr pub_1080_;
  int32_t frame_index_ = 0;
  uint64_t stats_zero_copy_ = 0;
  uint64_t stats_fallback_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JpegToImgNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
