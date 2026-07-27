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
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tb_img_msgs/msg/tb_msg1080_p.hpp"
#include "tb_img_msgs/msg/tb_msg480_p.hpp"
#include "tb_img_msgs/msg/tb_msg540_p.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam.hpp"

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

  const auto stamp = camera->get_image_timestamp();
  msg.time_stamp.sec = stamp.tv_sec;
  msg.time_stamp.nanosec = stamp.tv_nsec;
  msg.index = frame_index++;

  publisher->publish(std::move(loaned_msg));
  return true;
}

class LoanedTbImgPublisher
{
public:
  LoanedTbImgPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    TbImgProfile profile)
  : profile_(profile)
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

  TbImgProfile profile() const
  {
    return profile_;
  }

  bool publish(UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
  {
    switch (profile_) {
      case TbImgProfile::MSG_480P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg480P>(
          pub_480_, camera, frame_index, logger);
      case TbImgProfile::MSG_540P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg540P>(
          pub_540_, camera, frame_index, logger);
      case TbImgProfile::MSG_1080P:
        return publish_loaned_tb_img<tb_img_msgs::msg::TbMsg1080P>(
          pub_1080_, camera, frame_index, logger);
    }
    return false;
  }

private:
  TbImgProfile profile_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg480P>::SharedPtr pub_480_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg540P>::SharedPtr pub_540_;
  rclcpp::Publisher<tb_img_msgs::msg::TbMsg1080P>::SharedPtr pub_1080_;
};

}  // namespace topsbot_usb_cam

#endif  // TOPSBOT_USB_CAM__LOANED_TB_IMG_PUBLISHER_HPP_
