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
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_jpeg_frame.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam.hpp"

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

private:
  rclcpp::Publisher<tb_img_msgs::msg::TbJpegFrame>::SharedPtr publisher_;
};

}  // namespace topsbot_usb_cam

#endif  // TOPSBOT_USB_CAM__LOANED_TB_JPEG_PUBLISHER_HPP_
