// Copyright 2014 Robert Bosch, LLC
// Copyright 2021 Evan Flynn
// Copyright (c) 2026 TOPSBOT contributors.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rcpputils/env.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam_node.hpp"
#include "topsbot_usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace topsbot_usb_cam
{

namespace
{

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }
    return target_path.string();
  }
  return path;
}

std::string normalize_pixel_format(const std::string & name)
{
  if (name == "mjpeg") {
    return "raw_mjpeg";
  }
  return name;
}

bool is_raw_mjpeg_format(const std::string & name)
{
  return normalize_pixel_format(name) == "raw_mjpeg";
}

void check_zero_copy_env(const rclcpp::Logger & logger)
{
  const std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
  if (ros_zerocopy_env.empty()) {
    RCLCPP_ERROR(
      logger,
      "zero_copy is enabled but RMW_FASTRTPS_USE_QOS_FROM_XML is not set. "
      "Loaned message transport may fall back to copy mode.");
    return;
  }
  if (ros_zerocopy_env == "1") {
    RCLCPP_INFO(logger, "Fast DDS zero-copy environment is enabled");
  } else {
    RCLCPP_WARN(
      logger,
      "RMW_FASTRTPS_USE_QOS_FROM_XML=%s, expected 1 for zero-copy",
      ros_zerocopy_env.c_str());
  }
}

}  // namespace

TopsbotUsbCamNode::TopsbotUsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("topsbot_usb_cam", node_options),
  m_camera(new topsbot_usb_cam::UsbCam()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_compressed_img_msg(nullptr),
  m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
      rclcpp::QoS {100}.get_rmw_qos_profile()))),
  m_compressed_image_publisher(nullptr),
  m_compressed_cam_info_publisher(nullptr),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
  m_service_capture(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &TopsbotUsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "mjpeg");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);
  this->declare_parameter("contrast", -1);
  this->declare_parameter("saturation", -1);
  this->declare_parameter("sharpness", -1);
  this->declare_parameter("gain", -1);
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);
  this->declare_parameter("zero_copy", true);
  this->declare_parameter("tbmem_topic", "tbmem_img");
  this->declare_parameter("tb_img_profile", "auto");

  get_params();
  init();
  m_parameters_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &TopsbotUsbCamNode::parameters_callback, this,
      std::placeholders::_1));
}

TopsbotUsbCamNode::~TopsbotUsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_publish_timer.reset();
  m_tbmem_publisher.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();
  delete m_camera;
}

void TopsbotUsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    m_camera->start_capturing();
    response->message = "Start Capturing";
  } else {
    m_camera->stop_capturing();
    response->message = "Stop Capturing";
  }
  response->success = true;
}

void TopsbotUsbCamNode::init()
{
  while (m_parameters.frame_id == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required parameters not set, waiting...");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  m_camera_info.reset(
    new camera_info_manager::CameraInfoManager(
      this, m_parameters.camera_name, m_parameters.camera_info_url));
  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  {
    auto available_devices = topsbot_usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Device is not available: `" << m_parameters.device_name << "`");
      rclcpp::shutdown();
      return;
    }
  }

  if (!m_zero_copy) {
    if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
      m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
      m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
      m_compressed_image_publisher =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
        std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
      m_compressed_cam_info_publisher =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::QoS(100));
    }
  }

  m_image_msg->header.frame_id = m_parameters.frame_id;
  RCLCPP_INFO(
    this->get_logger(),
    "Starting '%s' (%s) at %dx%d via %s (%s) at %d FPS, zero_copy=%s",
    m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
    m_parameters.image_width, m_parameters.image_height,
    m_parameters.io_method_name.c_str(), m_parameters.pixel_format_name.c_str(),
    m_parameters.framerate, m_zero_copy ? "true" : "false");

  const io_method_t io_method =
    topsbot_usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == topsbot_usb_cam::utils::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR(this->get_logger(), "Unknown IO method '%s'",
      m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  m_camera->configure(m_parameters, io_method);
  set_v4l2_params();
  m_camera->start();

  if (m_zero_copy) {
    check_zero_copy_env(this->get_logger());
    TbImgProfile profile;
    if (m_tb_img_profile_name == "auto") {
      profile = select_tb_img_profile(
        m_camera->get_image_width(),
        m_camera->get_image_height(),
        m_camera->get_image_size_in_bytes());
    } else {
      profile = tb_img_profile_from_string(m_tb_img_profile_name);
    }
    m_tbmem_publisher = std::make_unique<LoanedTbImgPublisher>(
      this, m_tbmem_topic, profile);
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing loaned tb_img_msgs on '%s'",
      m_tbmem_topic.c_str());
  }

  const int period_ms = 1000 / std::max(m_parameters.framerate, 1);
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&TopsbotUsbCamNode::update, this));

  if (!m_zero_copy) {
    const int publish_period_ms = 1000 / std::max(m_parameters.framerate, 1);
    m_publish_timer = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&TopsbotUsbCamNode::publish, this));
  }
}

void TopsbotUsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus", "zero_copy", "tbmem_topic",
      "tb_img_profile"
    });
  assign_params(parameters);
}

void TopsbotUsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "camera_name") {
      m_parameters.camera_name = parameter.value_to_string();
    } else if (name == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (name == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (name == "framerate") {
      m_parameters.framerate = parameter.as_int();
    } else if (name == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (name == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (name == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (name == "pixel_format") {
      m_parameters.pixel_format_name =
        normalize_pixel_format(parameter.value_to_string());
    } else if (name == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (name == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    } else if (name == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (name == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (name == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (name == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (name == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (name == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (name == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (name == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (name == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (name == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (name == "focus") {
      m_parameters.focus = parameter.as_int();
    } else if (name == "zero_copy") {
      m_zero_copy = parameter.as_bool();
    } else if (name == "tbmem_topic") {
      m_tbmem_topic = parameter.value_to_string();
    } else if (name == "tb_img_profile") {
      m_tb_img_profile_name = parameter.value_to_string();
    }
  }
}

void TopsbotUsbCamNode::set_v4l2_params()
{
  if (m_parameters.brightness >= 0) {
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
  }
  if (m_parameters.contrast >= 0) {
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
  }
  if (m_parameters.saturation >= 0) {
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
  }
  if (m_parameters.sharpness >= 0) {
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
  }
  if (m_parameters.gain >= 0) {
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
  }
  if (m_parameters.auto_white_balance) {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
  } else {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }
  if (!m_parameters.autoexposure) {
    m_camera->set_v4l_parameter("exposure_auto", 1);
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
  } else {
    m_camera->set_v4l_parameter("exposure_auto", 3);
  }
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    m_camera->set_v4l_parameter("focus_auto", 1);
  } else {
    m_camera->set_v4l_parameter("focus_auto", 0);
    if (m_parameters.focus >= 0) {
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

bool TopsbotUsbCamNode::take_and_send_image()
{
  if (m_image_msg->data.size() != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  m_camera->get_image(reinterpret_cast<char *>(m_image_msg->data.data()));
  const auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool TopsbotUsbCamNode::take_and_send_image_mjpeg()
{
  if (m_compressed_img_msg->data.size() != m_camera->get_image_size_in_bytes()) {
    m_compressed_img_msg->format = "jpeg";
    m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  m_camera->get_image(reinterpret_cast<char *>(m_compressed_img_msg->data.data()));
  const auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;
  return true;
}

bool TopsbotUsbCamNode::tbmem_take_and_send_image()
{
  if (!m_tbmem_publisher) {
    return false;
  }
  return m_tbmem_publisher->publish(m_camera, m_send_index, this->get_logger());
}

rcl_interfaces::msg::SetParametersResult TopsbotUsbCamNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  m_timer->reset();
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TopsbotUsbCamNode::update()
{
  if (!m_camera->is_capturing()) {
    return;
  }

  bool ok = true;
  if (m_zero_copy) {
    ok = tbmem_take_and_send_image();
  } else if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
    ok = take_and_send_image_mjpeg();
  } else {
    ok = take_and_send_image();
  }

  if (!ok) {
    RCLCPP_WARN_ONCE(this->get_logger(), "USB camera frame was not published in time.");
  }
}

void TopsbotUsbCamNode::publish()
{
  if (m_zero_copy) {
    return;
  }
  if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
    m_compressed_image_publisher->publish(*m_compressed_img_msg);
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  } else {
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  }
}

}  // namespace topsbot_usb_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topsbot_usb_cam::TopsbotUsbCamNode)
