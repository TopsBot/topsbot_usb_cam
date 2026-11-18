// Copyright (c) 2026 TOPSBOT contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TOPSBOT_USB_CAM__LOANED_TB_IMG_PUBLISHER_HPP_
#define TOPSBOT_USB_CAM__LOANED_TB_IMG_PUBLISHER_HPP_

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_msg1080_p.hpp"
#include "tb_img_msgs/msg/tb_msg480_p.hpp"
#include "tb_img_msgs/msg/tb_msg540_p.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam.hpp"

#include <BufferAllocator/BufferAllocatorWrapper.h>

namespace topsbot_usb_cam
{

enum class TbImgProfile
{
  MSG_480P,
  MSG_540P,
  MSG_1080P,
};

inline TbImgProfile select_tb_img_profile(
  uint32_t /*width*/, uint32_t /*height*/, uint32_t payload_size)
{
  constexpr uint32_t kMax480 = tb_img_msgs::msg::TbMsg480P::MAX_SIZE;
  constexpr uint32_t kMax540 = tb_img_msgs::msg::TbMsg540P::MAX_SIZE;
  constexpr uint32_t kMax1080 = tb_img_msgs::msg::TbMsg1080P::MAX_SIZE;

  // Pick the smallest fixed buffer that fits (e.g. 640x640 NV12 fits TbMsg480P).
  if (payload_size <= kMax480) {
    return TbImgProfile::MSG_480P;
  }
  if (payload_size <= kMax540) {
    return TbImgProfile::MSG_540P;
  }
  if (payload_size <= kMax1080) {
    return TbImgProfile::MSG_1080P;
  }
  return TbImgProfile::MSG_1080P;
}

inline TbImgProfile tb_img_profile_from_string(const std::string & profile)
{
  if (profile == "480p" || profile == "TbMsg480P") {
    return TbImgProfile::MSG_480P;
  }
  if (profile == "540p" || profile == "TbMsg540P") {
    return TbImgProfile::MSG_540P;
  }
  if (profile == "1080p" || profile == "TbMsg1080P") {
    return TbImgProfile::MSG_1080P;
  }
  return TbImgProfile::MSG_1080P;
}

inline void fill_tb_img_encoding(
  std::array<uint8_t, 12> & encoding_field, const std::string & encoding)
{
  encoding_field.fill(0);
  const auto copy_len = std::min(encoding.size(), encoding_field.size() - 1);
  std::memcpy(encoding_field.data(), encoding.c_str(), copy_len);
}

template<typename MsgT>
bool publish_loaned_tb_img(
  const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher,
  UsbCam * camera,
  BufferAllocator * allocator,
  int32_t & frame_index,
  const rclcpp::Logger & logger)
{
  auto loaned_msg = publisher->borrow_loaned_message();
  if (!loaned_msg.is_valid()) {
    RCLCPP_WARN(logger, "borrow_loaned_message failed");
    return false;
  }

  auto & msg = loaned_msg.get();
  const auto data_size = static_cast<uint32_t>(camera->get_image_size_in_bytes());
  if (data_size > MsgT::MAX_SIZE) {
    RCLCPP_ERROR(
      logger,
      "Image payload %u bytes exceeds tb_img_msgs MAX_SIZE %u",
      data_size, MsgT::MAX_SIZE);
    return false;
  }

  msg.width = camera->get_image_width();
  msg.height = camera->get_image_height();
  fill_tb_img_encoding(msg.encoding, camera->get_pixel_format()->ros());
  msg.step = camera->get_image_step();
  camera->get_image(reinterpret_cast<char *>(msg.data.data()));
  msg.data_size = data_size;

  // dmabuf backend: expose the loaned payload phys so hardware consumers
  // (NPU / PP / encoder) can LOOKUP it and skip the copy. 0 = backend not
  // active, consumers fall back to copying.
  uint64_t payload_phys = 0;
  if (allocator != nullptr) {
    DmabufHeapLookupPhysByPtr(allocator, msg.data.data(), &payload_phys);
  }
  msg.phys_addr = payload_phys;

  const auto stamp = camera->get_image_timestamp();
  msg.time_stamp.sec = stamp.tv_sec;
  msg.time_stamp.nanosec = stamp.tv_nsec;
  msg.index = frame_index++;

  publisher->publish(std::move(loaned_msg));
  return true;
}

template<typename MsgT>
struct RawPixelSlot
{
  int fd{-1};
  bool queued{false};
  uint64_t phys{0};
  std::optional<rclcpp::LoanedMessage<MsgT>> loan;
};

template<typename MsgT>
struct RawPixelQueue
{
  std::array<RawPixelSlot<MsgT>, 2> slots{};
  unsigned int queued{0};
  bool streaming{false};
};

inline uint32_t raw_pixel_frame_bytes(UsbCam * camera)
{
  const auto format = camera->get_pixel_format();
  return static_cast<uint32_t>(format->compute_frame_size_in_bytes(
    camera->get_image_width(), camera->get_image_height()));
}

template<typename MsgT>
void fill_raw_pixel_header(MsgT & msg, UsbCam * camera, uint32_t frame_bytes, int32_t & frame_index)
{
  msg.width = static_cast<uint32_t>(camera->get_image_width());
  msg.height = static_cast<uint32_t>(camera->get_image_height());
  msg.step = camera->get_image_step();
  msg.data_size = frame_bytes;
  fill_tb_img_encoding(msg.encoding, camera->get_pixel_format()->ros());
  const auto stamp = camera->get_image_timestamp();
  msg.time_stamp.sec = stamp.tv_sec;
  msg.time_stamp.nanosec = stamp.tv_nsec;
  msg.index = frame_index++;
}

/// Machine 2.2 for raw pixels (YUYV, UYVY, NV12, NV21, RGB, mono). The loan
/// slot is QBUF'd. DQBUF publishes that slot and does not requeue it.
template<typename MsgT>
bool publish_raw_pixel_loan_queue(
  const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher,
  RawPixelQueue<MsgT> & queue,
  UsbCam * camera,
  BufferAllocator * allocator,
  int32_t & frame_index,
  const rclcpp::Logger & logger)
{
  if (!publisher || !camera || allocator == nullptr) {
    return false;
  }

  const uint32_t frame_bytes = raw_pixel_frame_bytes(camera);
  const uint32_t sizeimage = camera->capture_sizeimage();
  if (frame_bytes == 0 || sizeimage < frame_bytes || sizeimage > MsgT::MAX_SIZE) {
    RCLCPP_ERROR(
      logger,
      "raw pixel sizeimage %u frame %u does not fit TbMsg MAX_SIZE %u",
      sizeimage, frame_bytes, MsgT::MAX_SIZE);
    return false;
  }

  auto enqueue = [&]() -> bool {
    int free_index = -1;
    for (size_t i = 0; i < queue.slots.size(); ++i) {
      if (!queue.slots[i].queued) {
        free_index = static_cast<int>(i);
        break;
      }
    }
    if (free_index < 0) {
      return false;
    }
    auto loaned = publisher->borrow_loaned_message();
    if (!loaned.is_valid()) {
      return false;
    }
    auto & msg = loaned.get();
    volatile unsigned char * base = msg.data.data();
    for (size_t off = 0; off < sizeimage; off += 4096) {
      (void)base[off];
    }
    uint64_t phys = 0;
    DmabufHeapLookupPhysByPtr(allocator, msg.data.data(), &phys);
    uint64_t len = sizeimage;
    const int fd = phys == 0 ? -1 : DmabufHeapLookupByPhys(allocator, phys, &len);
    uint64_t got = 0;
    uint64_t got_len = 0;
    const int phys_rc = fd < 0 ? -1 : DmabufHeapGetPhysAddr(allocator, fd, &got, &got_len);
    if (fd < 0 || phys_rc != 0 || got != phys || got_len < sizeimage) {
      if (fd >= 0) {
        close(fd);
      }
      RCLCPP_ERROR(
        logger,
        "loan fd is not the data[] slice (lookup=0x%llx get=0x%llx len=%llu sizeimage=%u)",
        static_cast<unsigned long long>(phys),
        static_cast<unsigned long long>(got),
        static_cast<unsigned long long>(got_len),
        sizeimage);
      return false;
    }
    if (!camera->queue_import_dmabuf(static_cast<unsigned int>(free_index), fd, sizeimage)) {
      close(fd);
      RCLCPP_ERROR(logger, "QBUF of raw pixel loan fd failed");
      return false;
    }
    queue.slots[free_index].fd = fd;
    queue.slots[free_index].queued = true;
    queue.slots[free_index].phys = phys;
    queue.slots[free_index].loan.emplace(std::move(loaned));
    ++queue.queued;
    return true;
  };

  while (queue.queued < queue.slots.size()) {
    if (!enqueue()) {
      break;
    }
  }
  if (!queue.streaming) {
    if (queue.queued < queue.slots.size()) {
      return true;
    }
    camera->start_import_stream();
    queue.streaming = true;
  }
  if (queue.queued < 2) {
    return true;
  }

  int fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * data = nullptr;
  if (!camera->dequeue_dmabuf(fd, index, bytes_used, data)) {
    return false;
  }
  (void)fd;
  (void)data;
  if (index >= queue.slots.size() || !queue.slots[index].queued || !queue.slots[index].loan) {
    RCLCPP_ERROR(logger, "DQBUF index %u is not a queued loan", index);
    return false;
  }
  queue.slots[index].queued = false;
  if (queue.queued > 0) {
    --queue.queued;
  }
  const int lookup_fd = queue.slots[index].fd;
  queue.slots[index].fd = -1;
  auto loaned = std::move(*queue.slots[index].loan);
  queue.slots[index].loan.reset();
  if (lookup_fd >= 0) {
    close(lookup_fd);
  }

  if (bytes_used != frame_bytes) {
    RCLCPP_WARN(
      logger, "raw pixel bytesused %u does not match frame %u", bytes_used, frame_bytes);
    return false;
  }

  auto & msg = loaned.get();
  fill_raw_pixel_header(msg, camera, frame_bytes, frame_index);
  msg.phys_addr = queue.slots[index].phys;
  publisher->publish(std::move(loaned));
  return true;
}

/// Machine 2.1 for raw pixels. Copy the whole frame into the POSIX SHM loan.
template<typename MsgT>
bool publish_raw_pixel_capture_copy(
  const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher,
  UsbCam * camera,
  int32_t & frame_index,
  const rclcpp::Logger & logger)
{
  if (!publisher || !camera) {
    return false;
  }
  int fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * data = nullptr;
  if (!camera->dequeue_dmabuf(fd, index, bytes_used, data)) {
    return false;
  }
  struct Requeue
  {
    UsbCam * camera;
    unsigned int index;
    ~Requeue() {camera->requeue_dmabuf(index);}
  } requeue{camera, index};
  (void)fd;

  const uint32_t frame_bytes = raw_pixel_frame_bytes(camera);
  if (data == nullptr || bytes_used != frame_bytes || frame_bytes > MsgT::MAX_SIZE) {
    RCLCPP_WARN(
      logger, "raw pixel capture %u bytes does not match frame %u", bytes_used, frame_bytes);
    return false;
  }

  auto loaned = publisher->borrow_loaned_message();
  if (!loaned.is_valid()) {
    RCLCPP_WARN(logger, "borrow_loaned_message failed");
    return false;
  }
  auto & msg = loaned.get();
  std::memcpy(msg.data.data(), data, frame_bytes);
  fill_raw_pixel_header(msg, camera, frame_bytes, frame_index);
  msg.phys_addr = 0;
  publisher->publish(std::move(loaned));
  return true;
}

class LoanedTbImgPublisher
{
public:
  LoanedTbImgPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    TbImgProfile profile)
  : profile_(profile),
    allocator_(CreateDmabufHeapBufferAllocator())
  {
    const auto qos = rclcpp::SensorDataQoS();
    switch (profile_) {
      case TbImgProfile::MSG_480P:
        pub_480_ = node->create_publisher<tb_img_msgs::msg::TbMsg480P>(topic, qos);
        break;
      case TbImgProfile::MSG_540P:
        pub_540_ = node->create_publisher<tb_img_msgs::msg::TbMsg540P>(topic, qos);
        break;
      case TbImgProfile::MSG_1080P:
        pub_1080_ = node->create_publisher<tb_img_msgs::msg::TbMsg1080P>(topic, qos);
        break;
    }
  }

  ~LoanedTbImgPublisher()
  {
    if (allocator_ != nullptr) {
      FreeDmabufHeapBufferAllocator(allocator_);
    }
  }

  TbImgProfile profile() const
  {
    return profile_;
  }

  /// MJPEG DMABUF capture. With FASTDDS_DMABUF_BACKEND=1 the JPU writes the
  /// loaned carveout. Otherwise it decodes into a temporary carveout and one
  /// copy fills the shared-memory loan.
  bool publish_dmabuf_nv12(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger);

  /// MJPEG DMABUF capture to RGB. JPU writes a temporary NV12 carveout, then
  /// CSC writes RGB. With FASTDDS_DMABUF_BACKEND=1 the RGB lands in the loan.
  bool publish_dmabuf_rgb(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger);

  /// Raw pixel DMABUF. FASTDDS_DMABUF_BACKEND=1 QBUFs the loan itself.
  bool publish_raw_loan_queue(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    switch (profile_) {
      case TbImgProfile::MSG_480P:
        return publish_raw_pixel_loan_queue<tb_img_msgs::msg::TbMsg480P>(
          pub_480_, raw_480_, camera, allocator_, frame_index, logger);
      case TbImgProfile::MSG_540P:
        return publish_raw_pixel_loan_queue<tb_img_msgs::msg::TbMsg540P>(
          pub_540_, raw_540_, camera, allocator_, frame_index, logger);
      case TbImgProfile::MSG_1080P:
        return publish_raw_pixel_loan_queue<tb_img_msgs::msg::TbMsg1080P>(
          pub_1080_, raw_1080_, camera, allocator_, frame_index, logger);
    }
    return false;
  }

  /// Raw pixel DMABUF into a POSIX SHM loan. One copy of the frame, phys_addr 0.
  bool publish_raw_capture_copy(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    switch (profile_) {
      case TbImgProfile::MSG_480P:
        return publish_raw_pixel_capture_copy<tb_img_msgs::msg::TbMsg480P>(
          pub_480_, camera, frame_index, logger);
      case TbImgProfile::MSG_540P:
        return publish_raw_pixel_capture_copy<tb_img_msgs::msg::TbMsg540P>(
          pub_540_, camera, frame_index, logger);
      case TbImgProfile::MSG_1080P:
        return publish_raw_pixel_capture_copy<tb_img_msgs::msg::TbMsg1080P>(
          pub_1080_, camera, frame_index, logger);
    }
    return false;
  }

  bool publish(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    switch (profile_) {
      case TbImgProfile::MSG_480P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg480P>(
          pub_480_, camera, allocator_, frame_index, logger);
      case TbImgProfile::MSG_540P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg540P>(
          pub_540_, camera, allocator_, frame_index, logger);
      case TbImgProfile::MSG_1080P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg1080P>(
          pub_1080_, camera, allocator_, frame_index, logger);
    }
    return false;
  }

private:
  TbImgProfile profile_;
  BufferAllocator * allocator_{nullptr};
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg480P>::SharedPtr pub_480_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg540P>::SharedPtr pub_540_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg1080P>::SharedPtr pub_1080_;
  RawPixelQueue<tb_img_msgs::msg::TbMsg480P> raw_480_;
  RawPixelQueue<tb_img_msgs::msg::TbMsg540P> raw_540_;
  RawPixelQueue<tb_img_msgs::msg::TbMsg1080P> raw_1080_;
};

}  // namespace topsbot_usb_cam

#endif  // TOPSBOT_USB_CAM__LOANED_TB_IMG_PUBLISHER_HPP_
