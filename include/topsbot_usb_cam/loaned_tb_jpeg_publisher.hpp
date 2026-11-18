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

#ifndef TOPSBOT_USB_CAM__LOANED_TB_JPEG_PUBLISHER_HPP_
#define TOPSBOT_USB_CAM__LOANED_TB_JPEG_PUBLISHER_HPP_

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_jpeg_frame.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam.hpp"

#include <unistd.h>

#include <BufferAllocator/BufferAllocatorWrapper.h>

namespace topsbot_usb_cam
{

// Resolve the physical address of a loaned payload inside the carveout
// FastDDS DataSharing segment (if the dmabuf backend is active). Returns 0
// when the backend is not enabled or the pointer is not in a segment.
inline uint64_t loaned_payload_phys(const void * data_ptr)
{
  BufferAllocator * allocator = CreateDmabufHeapBufferAllocator();
  if (!allocator) {
    return 0;
  }
  uint64_t phys = 0;
  const int ret = DmabufHeapLookupPhysByPtr(allocator, data_ptr, &phys);
  FreeDmabufHeapBufferAllocator(allocator);
  return (ret == 0) ? phys : 0;
}

inline void fill_tb_jpeg_encoding(
  std::array<uint8_t, 12> & encoding_field, const std::string & encoding)
{
  encoding_field.fill(0);
  const auto copy_len = std::min(encoding.size(), encoding_field.size() - 1);
  std::memcpy(encoding_field.data(), encoding.c_str(), copy_len);
}

class LoanedTbJpegPublisher
{
public:
  explicit LoanedTbJpegPublisher(
    rclcpp::Node * node,
    const std::string & topic)
  {
    publisher_ = node->create_publisher<tb_img_msgs::msg::TbJpegFrame>(
      topic, rclcpp::SensorDataQoS());
  }

  bool publish(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    if (!publisher_ || !camera) {
      return false;
    }

    auto loaned_msg = publisher_->borrow_loaned_message();
    if (!loaned_msg.is_valid()) {
      RCLCPP_WARN(logger, "borrow_loaned_message failed for TbJpegFrame");
      return false;
    }

    auto & msg = loaned_msg.get();
    msg.width = camera->get_image_width();
    msg.height = camera->get_image_height();
    fill_tb_jpeg_encoding(msg.encoding, "jpeg");
    camera->get_image(reinterpret_cast<char *>(msg.data.data()));

    const auto data_size = static_cast<uint32_t>(camera->get_image_bytes_used());
    if (data_size == 0) {
      RCLCPP_WARN(logger, "TbJpegFrame capture returned 0 bytes, skip publish");
      return false;
    }
    if (data_size > tb_img_msgs::msg::TbJpegFrame::MAX_SIZE) {
      RCLCPP_ERROR(
        logger,
        "JPEG payload %u bytes exceeds TbJpegFrame MAX_SIZE %u",
        data_size, tb_img_msgs::msg::TbJpegFrame::MAX_SIZE);
      return false;
    }
    msg.data_size = data_size;

    // dmabuf backend: publish the payload phys so hardware consumers can wrap
    // it into an fd (LOOKUP_BY_PHYS) and skip the copy into a fresh buffer.
    // 0 = backend not active, consumers fall back to copying.
    const uint64_t payload_phys = loaned_payload_phys(msg.data.data());
    msg.phys_addr = payload_phys;

    const auto stamp = camera->get_image_timestamp();
    msg.time_stamp.sec = stamp.tv_sec;
    msg.time_stamp.nanosec = stamp.tv_nsec;
    msg.index = frame_index++;

    publisher_->publish(std::move(loaned_msg));
    return true;
  }

  /// DMABUF capture pool, POSIX SHM loan. Copy only the captured JPEG
  /// (bytesused). The loan is not a carveout, so phys_addr stays 0 and the
  /// capture fd is requeued after the copy.
  bool publish_capture_copy(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    if (!publisher_ || !camera) {
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

    if (bytes_used < 2 || data == nullptr || data[0] != 0xFF || data[1] != 0xD8) {
      RCLCPP_WARN(logger, "capture DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
      return false;
    }
    if (bytes_used > tb_img_msgs::msg::TbJpegFrame::MAX_SIZE) {
      RCLCPP_ERROR(
        logger, "JPEG %u bytes exceeds TbJpegFrame MAX_SIZE %u",
        bytes_used, tb_img_msgs::msg::TbJpegFrame::MAX_SIZE);
      return false;
    }

    auto loaned = publisher_->borrow_loaned_message();
    if (!loaned.is_valid()) {
      RCLCPP_WARN(logger, "borrow_loaned_message failed for TbJpegFrame");
      return false;
    }
    auto & msg = loaned.get();
    std::memcpy(msg.data.data(), data, bytes_used);
    msg.width = camera->get_image_width();
    msg.height = camera->get_image_height();
    fill_tb_jpeg_encoding(msg.encoding, "jpeg");
    msg.data_size = bytes_used;
    msg.phys_addr = 0;
    const auto stamp = camera->get_image_timestamp();
    msg.time_stamp.sec = stamp.tv_sec;
    msg.time_stamp.nanosec = stamp.tv_nsec;
    msg.index = frame_index++;
    publisher_->publish(std::move(loaned));
    return true;
  }

  /// Machine 2.2: each loan slot is QBUF'd. DQBUF publishes that same slot and
  /// does not requeue it. The next frame borrows a new loan.
  bool publish_loan_queue(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    if (!publisher_ || !camera) {
      return false;
    }
    if (alloc_ == nullptr) {
      alloc_ = CreateDmabufHeapBufferAllocator();
    }
    if (alloc_ == nullptr) {
      RCLCPP_WARN(logger, "JPEG loan allocator failed");
      return false;
    }

    const uint32_t sizeimage = camera->capture_sizeimage();
    if (sizeimage == 0 || sizeimage > tb_img_msgs::msg::TbJpegFrame::MAX_SIZE) {
      RCLCPP_ERROR(
        logger, "sizeimage %u does not fit TbJpegFrame MAX_SIZE %u",
        sizeimage, tb_img_msgs::msg::TbJpegFrame::MAX_SIZE);
      return false;
    }

    while (queued_ < slots_.size()) {
      if (!enqueue_loan(camera, sizeimage, logger)) {
        break;
      }
    }
    if (!streaming_) {
      if (queued_ < slots_.size()) {
        return true;
      }
      camera->start_import_stream();
      streaming_ = true;
    }
    if (queued_ < 2) {
      return true;
    }

    int fd = -1;
    unsigned int index = 0;
    uint32_t bytes_used = 0;
    const uint8_t * data = nullptr;
    if (!camera->dequeue_dmabuf(fd, index, bytes_used, data)) {
      return false;
    }
    if (index >= slots_.size() || !slots_[index].queued || !slots_[index].loan) {
      RCLCPP_ERROR(logger, "DQBUF index %u is not a queued loan", index);
      return false;
    }
    slots_[index].queued = false;
    if (queued_ > 0) {
      --queued_;
    }
    const int lookup_fd = slots_[index].fd;
    slots_[index].fd = -1;

    auto loaned = std::move(*slots_[index].loan);
    slots_[index].loan.reset();
    if (lookup_fd >= 0) {
      close(lookup_fd);
    }

    auto & msg = loaned.get();
    if (bytes_used < 2 || msg.data[0] != 0xFF || msg.data[1] != 0xD8) {
      RCLCPP_WARN(logger, "loan DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
      return false;
    }
    if (bytes_used > tb_img_msgs::msg::TbJpegFrame::MAX_SIZE) {
      RCLCPP_ERROR(logger, "JPEG %u bytes exceeds TbJpegFrame", bytes_used);
      return false;
    }

    msg.width = camera->get_image_width();
    msg.height = camera->get_image_height();
    fill_tb_jpeg_encoding(msg.encoding, "jpeg");
    msg.data_size = bytes_used;
    msg.phys_addr = slots_phys_[index];
    const auto stamp = camera->get_image_timestamp();
    msg.time_stamp.sec = stamp.tv_sec;
    msg.time_stamp.nanosec = stamp.tv_nsec;
    msg.index = frame_index++;
    publisher_->publish(std::move(loaned));
    return true;
  }

private:
  bool enqueue_loan(UsbCam * camera, uint32_t sizeimage, const rclcpp::Logger & logger)
  {
    int free_index = -1;
    for (size_t i = 0; i < slots_.size(); ++i) {
      if (!slots_[i].queued) {
        free_index = static_cast<int>(i);
        break;
      }
    }
    if (free_index < 0) {
      return false;
    }

    auto loaned = publisher_->borrow_loaned_message();
    if (!loaned.is_valid()) {
      return false;
    }
    auto & msg = loaned.get();
    volatile unsigned char * base = msg.data.data();
    for (size_t off = 0; off < sizeimage; off += 4096) {
      (void)base[off];
    }

    uint64_t phys = 0;
    DmabufHeapLookupPhysByPtr(alloc_, msg.data.data(), &phys);
    uint64_t len = sizeimage;
    const int fd = phys == 0 ? -1 : DmabufHeapLookupByPhys(alloc_, phys, &len);
    uint64_t got = 0;
    uint64_t got_len = 0;
    const int phys_rc = fd < 0 ? -1 : DmabufHeapGetPhysAddr(alloc_, fd, &got, &got_len);
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
      RCLCPP_ERROR(logger, "QBUF of JPEG loan fd failed");
      return false;
    }

    slots_[free_index].fd = fd;
    slots_[free_index].queued = true;
    slots_phys_[free_index] = phys;
    slots_[free_index].loan.emplace(std::move(loaned));
    ++queued_;
    return true;
  }

  struct Slot
  {
    int fd{-1};
    bool queued{false};
    std::optional<rclcpp::LoanedMessage<tb_img_msgs::msg::TbJpegFrame>> loan;
  };

  rclcpp::Publisher<tb_img_msgs::msg::TbJpegFrame>::SharedPtr publisher_;
  BufferAllocator * alloc_{nullptr};
  std::array<Slot, 2> slots_{};
  std::array<uint64_t, 2> slots_phys_{};
  unsigned int queued_{0};
  bool streaming_{false};
};

}  // namespace topsbot_usb_cam

#endif  // TOPSBOT_USB_CAM__LOANED_TB_JPEG_PUBLISHER_HPP_
