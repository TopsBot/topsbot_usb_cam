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
#include <chrono>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rcpputils/env.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "topsbot_usb_cam/topsbot_usb_cam_node.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>

#include "ta_cv_api_ext_c.h"
#include "topsbot_usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace topsbot_usb_cam
{

namespace
{

bool dmabuf_loan_backend_enabled()
{
  const char * backend = std::getenv("FASTDDS_DMABUF_BACKEND");
  return backend != nullptr && std::string(backend) == "1";
}

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

bool is_raw_pixel_format(const std::string & name)
{
  if (is_raw_mjpeg_format(name)) {
    return false;
  }
  for (const auto & format : driver_supported_formats()) {
    if (format->name() == name && !format->requires_conversion()) {
      return true;
    }
  }
  return false;
}

rcl_interfaces::msg::ParameterDescriptor param_desc(const char * description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

void declare_usb_cam_parameters(rclcpp::Node * node)
{
  node->declare_parameter(
    "camera_name", "test_camera",
    param_desc("Logical camera name for camera_info_manager"));
  node->declare_parameter(
    "camera_info_url", "package://topsbot_usb_cam/config/camera_info.yaml",
    param_desc("Calibration URL (package:// or file://); empty uses defaults"));
  node->declare_parameter("framerate", 30, param_desc("Capture frame rate (Hz)"));
  node->declare_parameter("frame_id", "usb_cam", param_desc("TF / message header frame_id"));
  node->declare_parameter("image_height", 480, param_desc("Capture height in pixels"));
  node->declare_parameter("image_width", 640, param_desc("Capture width in pixels"));
  node->declare_parameter(
    "io_method", "mmap",
    param_desc("V4L2 I/O method: mmap, read, userptr, or dmabuf"));
  node->declare_parameter(
    "pixel_format", "mjpeg2rgb",
    param_desc(
      "Output format: mjpeg2rgb, mjpeg2nv12, raw_mjpeg/mjpeg, uyvy, yuyv, ..."));
  node->declare_parameter(
    "av_device_format", "YUV422P",
    param_desc("FFmpeg device format hint for decode path"));
  node->declare_parameter(
    "video_device", "/dev/video0",
    param_desc("V4L2 device path, e.g. /dev/video0"));
  node->declare_parameter(
    "brightness", -1,
    param_desc("V4L2 brightness (-1 = leave unchanged)"));
  node->declare_parameter(
    "contrast", -1,
    param_desc("V4L2 contrast (-1 = leave unchanged)"));
  node->declare_parameter(
    "saturation", -1,
    param_desc("V4L2 saturation (-1 = leave unchanged)"));
  node->declare_parameter(
    "sharpness", -1,
    param_desc("V4L2 sharpness (-1 = leave unchanged)"));
  node->declare_parameter("gain", -1, param_desc("V4L2 gain (-1 = leave unchanged)"));
  node->declare_parameter(
    "auto_white_balance", true,
    param_desc("Enable automatic white balance"));
  node->declare_parameter(
    "white_balance", 4000,
    param_desc("Manual white balance temperature when auto is false"));
  node->declare_parameter("autoexposure", true, param_desc("Enable automatic exposure"));
  node->declare_parameter(
    "exposure", 100,
    param_desc("Manual exposure value when autoexposure is false"));
  node->declare_parameter("autofocus", false, param_desc("Enable continuous autofocus"));
  node->declare_parameter(
    "focus", -1,
    param_desc("Manual focus (-1 = leave unchanged)"));
  node->declare_parameter(
    "zero_copy", true,
    param_desc("Publish tb_img_msgs via loaned messages (requires Fast DDS or Cyclone DDS SHM env)"));
}

void check_zero_copy_env(const rclcpp::Logger & logger)
{
  const std::string rmw = rcpputils::get_env_var("RMW_IMPLEMENTATION");
  if (rmw.find("cyclonedds") != std::string::npos) {
    const std::string uri = rcpputils::get_env_var("CYCLONEDDS_URI");
    if (uri.empty()) {
      RCLCPP_ERROR(
        logger,
        "zero_copy with Cyclone DDS but CYCLONEDDS_URI is not set. "
        "Use setup_shm:=true (zc_env.launch.py) or export CYCLONEDDS_URI.");
      return;
    }
    RCLCPP_INFO(logger, "Cyclone DDS zero-copy environment is enabled");
    return;
  }

  const std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
  if (ros_zerocopy_env.empty()) {
    RCLCPP_ERROR(
      logger,
      "zero_copy is enabled but RMW_FASTRTPS_USE_QOS_FROM_XML is not set. "
      "Use setup_shm:=true (zc_env.launch.py) for Fast DDS SHM.");
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
  m_image_publisher(nullptr),
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
  declare_usb_cam_parameters(this);

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
  m_tbmem_publisher.reset();
  m_tbjpeg_publisher.reset();
  if (m_nv12_alloc != nullptr) {
    FreeDmabufHeapBufferAllocator(m_nv12_alloc);
    m_nv12_alloc = nullptr;
  }
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
      const std::string compressed_topic =
        std::string(BASE_TOPIC_NAME) + "/compressed";
      m_compressed_image_publisher =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
        compressed_topic, rclcpp::QoS(100));
      m_compressed_cam_info_publisher =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::QoS(100));
      RCLCPP_INFO(this->get_logger(), "Publishing %s", compressed_topic.c_str());
    } else {
      m_image_publisher = std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(
          this, BASE_TOPIC_NAME, rclcpp::QoS {100}.get_rmw_qos_profile()));
      RCLCPP_INFO(this->get_logger(), "Publishing %s", BASE_TOPIC_NAME);
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

  const bool dmabuf_nv12 =
    m_parameters.io_method_name == "dmabuf" &&
    m_parameters.pixel_format_name == "mjpeg2nv12";
  const bool dmabuf_rgb =
    m_parameters.io_method_name == "dmabuf" &&
    m_parameters.pixel_format_name == "mjpeg2rgb";
  const bool dmabuf_jpeg =
    m_parameters.io_method_name == "dmabuf" &&
    is_raw_mjpeg_format(m_parameters.pixel_format_name);
  const bool dmabuf_raw =
    m_parameters.io_method_name == "dmabuf" &&
    is_raw_pixel_format(m_parameters.pixel_format_name);
  if (m_parameters.io_method_name == "dmabuf" &&
    !dmabuf_nv12 && !dmabuf_rgb && !dmabuf_jpeg && !dmabuf_raw)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "io_method=dmabuf is only implemented for mjpeg2nv12, mjpeg2rgb, raw mjpeg, and raw pixels");
    rclcpp::shutdown();
    return;
  }
  if (dmabuf_nv12) {
    if (!m_zero_copy) {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2nv12 DMABUF capture: hardware decode, then copy NV12 into sensor_msgs/Image");
    } else if (dmabuf_loan_backend_enabled()) {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2nv12 DMABUF capture: hardware decode writes the carveout loan");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2nv12 DMABUF capture: hardware decode, then copy NV12 into the SHM loan");
    }
  }
  if (dmabuf_rgb) {
    if (!m_zero_copy) {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2rgb DMABUF capture: hardware decode and CSC, then copy RGB into sensor_msgs/Image");
    } else if (dmabuf_loan_backend_enabled()) {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2rgb DMABUF capture: hardware decode, CSC writes the carveout loan");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "mjpeg2rgb DMABUF capture: hardware decode and CSC, then copy RGB into the SHM loan");
    }
  }
  if (dmabuf_jpeg) {
    if (!m_zero_copy) {
      RCLCPP_INFO(
        this->get_logger(),
        "raw MJPEG DMABUF capture: copy JPEG bytes into sensor_msgs/CompressedImage");
    } else if (dmabuf_loan_backend_enabled()) {
      m_camera->use_loan_import(2);
      RCLCPP_INFO(
        this->get_logger(),
        "raw MJPEG DMABUF capture: loan slots are queued directly");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "raw MJPEG DMABUF capture: copy JPEG bytes into the SHM loan");
    }
  }
  if (dmabuf_raw) {
    if (!m_zero_copy) {
      RCLCPP_INFO(
        this->get_logger(),
        "raw pixel DMABUF capture: copy the frame into sensor_msgs/Image");
    } else if (dmabuf_loan_backend_enabled()) {
      m_camera->use_loan_import(2);
      RCLCPP_INFO(
        this->get_logger(),
        "raw pixel DMABUF capture: loan slots are queued directly");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "raw pixel DMABUF capture: copy the frame into the SHM loan");
    }
  }

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
    if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
      m_tbjpeg_publisher = std::make_unique<LoanedTbJpegPublisher>(this, "tbmem_jpeg");
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing loaned TbJpegFrame on 'tbmem_jpeg' (raw MJPEG)");
    } else {
      const TbImgProfile profile = select_tb_img_profile(
        m_camera->get_image_width(),
        m_camera->get_image_height(),
        m_camera->get_image_size_in_bytes());
      m_tbmem_publisher = std::make_unique<LoanedTbImgPublisher>(this, "tbmem_img", profile);
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing loaned tb_img_msgs on 'tbmem_img'");
    }
  }

  const int period_ms = 1000 / std::max(m_parameters.framerate, 1);
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&TopsbotUsbCamNode::update, this));
}

void TopsbotUsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus", "zero_copy"
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
  // sizeimage is only the capture-buffer capacity. Keep that much room for the
  // copy, then publish the length V4L2 reported in bytesused.
  const auto buffer_bytes = m_camera->get_image_size_in_bytes();
  if (m_compressed_img_msg->data.size() < buffer_bytes) {
    m_compressed_img_msg->data.resize(buffer_bytes);
  }
  m_compressed_img_msg->format = "jpeg";

  m_camera->get_image(reinterpret_cast<char *>(m_compressed_img_msg->data.data()));
  const auto used = m_camera->get_image_bytes_used();
  if (used == 0 || used > m_compressed_img_msg->data.size()) {
    return false;
  }
  m_compressed_img_msg->data.resize(used);

  const auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;
  return true;
}

bool TopsbotUsbCamNode::tbjpeg_take_and_send_image()
{
  if (!m_tbjpeg_publisher) {
    return false;
  }
  return m_tbjpeg_publisher->publish(m_camera, m_send_index, this->get_logger());
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
    if (
      is_raw_pixel_format(m_parameters.pixel_format_name) &&
      m_parameters.io_method_name == "dmabuf" &&
      dmabuf_loan_backend_enabled())
    {
      ok = m_tbmem_publisher &&
        m_tbmem_publisher->publish_raw_loan_queue(m_camera, m_send_index, this->get_logger());
    } else if (
      is_raw_pixel_format(m_parameters.pixel_format_name) &&
      m_parameters.io_method_name == "dmabuf")
    {
      ok = m_tbmem_publisher &&
        m_tbmem_publisher->publish_raw_capture_copy(m_camera, m_send_index, this->get_logger());
    } else if (
      is_raw_mjpeg_format(m_parameters.pixel_format_name) &&
      m_parameters.io_method_name == "dmabuf" &&
      dmabuf_loan_backend_enabled())
    {
      ok = m_tbjpeg_publisher &&
        m_tbjpeg_publisher->publish_loan_queue(m_camera, m_send_index, this->get_logger());
    } else if (
      is_raw_mjpeg_format(m_parameters.pixel_format_name) &&
      m_parameters.io_method_name == "dmabuf")
    {
      ok = m_tbjpeg_publisher &&
        m_tbjpeg_publisher->publish_capture_copy(m_camera, m_send_index, this->get_logger());
    } else if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
      ok = tbjpeg_take_and_send_image();
    } else if (
      m_parameters.io_method_name == "dmabuf" &&
      m_parameters.pixel_format_name == "mjpeg2nv12")
    {
      ok = m_tbmem_publisher &&
        m_tbmem_publisher->publish_dmabuf_nv12(m_camera, m_send_index, this->get_logger());
    } else if (
      m_parameters.io_method_name == "dmabuf" &&
      m_parameters.pixel_format_name == "mjpeg2rgb")
    {
      ok = m_tbmem_publisher &&
        m_tbmem_publisher->publish_dmabuf_rgb(m_camera, m_send_index, this->get_logger());
    } else {
      ok = tbmem_take_and_send_image();
    }
  } else if (
    m_parameters.io_method_name == "dmabuf" &&
    m_parameters.pixel_format_name == "mjpeg2nv12")
  {
    ok = take_and_send_dmabuf_nv12();
  } else if (
    m_parameters.io_method_name == "dmabuf" &&
    m_parameters.pixel_format_name == "mjpeg2rgb")
  {
    ok = take_and_send_dmabuf_rgb();
  } else if (
    m_parameters.io_method_name == "dmabuf" &&
    is_raw_mjpeg_format(m_parameters.pixel_format_name))
  {
    ok = take_and_send_dmabuf_jpeg();
  } else if (
    m_parameters.io_method_name == "dmabuf" &&
    is_raw_pixel_format(m_parameters.pixel_format_name))
  {
    ok = take_and_send_dmabuf_raw();
  } else if (is_raw_mjpeg_format(m_parameters.pixel_format_name)) {
    ok = take_and_send_image_mjpeg();
  } else {
    ok = take_and_send_image();
  }

  if (!ok) {
    RCLCPP_WARN_ONCE(this->get_logger(), "USB camera frame was not published in time.");
    return;
  }

  if (!m_zero_copy) {
    publish();
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

namespace
{

int parse_jpeg_dimensions(
  const unsigned char * jpeg_data, size_t jpeg_size, int * width, int * height)
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

class RequeueDmabuf
{
public:
  RequeueDmabuf(UsbCam * camera, unsigned int index)
  : camera_(camera), index_(index) {}

  ~RequeueDmabuf()
  {
    camera_->requeue_dmabuf(index_);
  }

private:
  UsbCam * camera_;
  unsigned int index_;
};

template<typename MsgT>
bool publish_dmabuf_nv12_msg(
  const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher,
  UsbCam * camera,
  BufferAllocator * allocator,
  int32_t & frame_index,
  const rclcpp::Logger & logger)
{
  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * jpeg = nullptr;
  if (!camera->dequeue_dmabuf(cap_fd, index, bytes_used, jpeg)) {
    return false;
  }
  RequeueDmabuf requeue(camera, index);

  if (bytes_used < 2 || jpeg == nullptr || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    RCLCPP_WARN(logger, "DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
    return false;
  }

  int jpeg_w = 0;
  int jpeg_h = 0;
  if (parse_jpeg_dimensions(jpeg, bytes_used, &jpeg_w, &jpeg_h) != 0 ||
    jpeg_w != static_cast<int>(camera->get_image_width()) ||
    jpeg_h != static_cast<int>(camera->get_image_height()))
  {
    RCLCPP_WARN(
      logger, "JPEG SOF %dx%d does not match capture %zux%zu",
      jpeg_w, jpeg_h, camera->get_image_width(), camera->get_image_height());
    return false;
  }

  const size_t nv12 = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3 / 2;
  if (nv12 > MsgT::MAX_SIZE) {
    RCLCPP_ERROR(
      logger, "NV12 %zu bytes exceeds TbMsg MAX_SIZE %u", nv12, MsgT::MAX_SIZE);
    return false;
  }

  auto loaned = publisher->borrow_loaned_message();
  if (!loaned.is_valid()) {
    RCLCPP_WARN(logger, "borrow_loaned_message failed");
    return false;
  }
  auto & msg = loaned.get();

  // FASTDDS_DMABUF_BACKEND=1: the loan is carveout. LOOKUP its fd and let the
  // JPU write NV12 there. Otherwise the loan is POSIX SHM, which the JPU
  // cannot write, so decode into a private carveout and copy once.
  const bool dma_loan = dmabuf_loan_backend_enabled();
  int out_fd = -1;
  uint64_t payload_phys = 0;
  if (dma_loan) {
    volatile unsigned char * base = msg.data.data();
    for (size_t off = 0; off < nv12; off += 4096) {
      (void)base[off];
    }
    DmabufHeapLookupPhysByPtr(allocator, msg.data.data(), &payload_phys);
    if (payload_phys != 0) {
      uint64_t len = nv12;
      out_fd = DmabufHeapLookupByPhys(allocator, payload_phys, &len);
    }
    if (out_fd < 0) {
      RCLCPP_WARN(
        logger, "loan LOOKUP failed (phys=0x%llx)",
        static_cast<unsigned long long>(payload_phys));
      return false;
    }
  } else {
    out_fd = DmabufHeapAlloc(allocator, "carveout-heap", nv12);
    if (out_fd < 0) {
      RCLCPP_WARN(logger, "NV12 carveout alloc failed");
      return false;
    }
  }

  ta_image_t image{};
  int ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_NV12, &image, out_fd);
  if (ret == 0) {
    ret = ta_cv_image_jpeg_dec(bytes_used, static_cast<unsigned int>(cap_fd), &image);
    ta_cv_image_destroy_ext(&image);
  }
  if (!dma_loan && ret == 0) {
    void * decoded = mmap(nullptr, nv12, PROT_READ, MAP_SHARED, out_fd, 0);
    if (decoded == MAP_FAILED) {
      ret = -1;
    } else {
      std::memcpy(msg.data.data(), decoded, nv12);
      munmap(decoded, nv12);
    }
  }
  close(out_fd);
  if (ret != 0) {
    RCLCPP_WARN(logger, "ta_cv_image_jpeg_dec failed (%d)", ret);
    return false;
  }

  msg.width = static_cast<uint32_t>(jpeg_w);
  msg.height = static_cast<uint32_t>(jpeg_h);
  msg.step = static_cast<uint32_t>(jpeg_w);
  msg.data_size = static_cast<uint32_t>(nv12);
  fill_tb_img_encoding(msg.encoding, "nv12");
  const auto stamp = camera->get_image_timestamp();
  msg.time_stamp.sec = stamp.tv_sec;
  msg.time_stamp.nanosec = stamp.tv_nsec;
  msg.index = frame_index++;
  msg.phys_addr = dma_loan ? payload_phys : 0;
  publisher->publish(std::move(loaned));
  return true;
}

template<typename MsgT>
bool publish_dmabuf_rgb_msg(
  const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher,
  UsbCam * camera,
  BufferAllocator * allocator,
  int32_t & frame_index,
  const rclcpp::Logger & logger)
{
  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * jpeg = nullptr;
  if (!camera->dequeue_dmabuf(cap_fd, index, bytes_used, jpeg)) {
    return false;
  }
  RequeueDmabuf requeue(camera, index);

  if (bytes_used < 2 || jpeg == nullptr || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    RCLCPP_WARN(logger, "DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
    return false;
  }

  int jpeg_w = 0;
  int jpeg_h = 0;
  if (parse_jpeg_dimensions(jpeg, bytes_used, &jpeg_w, &jpeg_h) != 0 ||
    jpeg_w != static_cast<int>(camera->get_image_width()) ||
    jpeg_h != static_cast<int>(camera->get_image_height()))
  {
    RCLCPP_WARN(
      logger, "JPEG SOF %dx%d does not match capture %zux%zu",
      jpeg_w, jpeg_h, camera->get_image_width(), camera->get_image_height());
    return false;
  }
  if ((jpeg_w & 0xF) != 0) {
    RCLCPP_ERROR(logger, "CSC output width %d is not a multiple of 16", jpeg_w);
    return false;
  }

  const size_t nv12 = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3 / 2;
  const size_t rgb = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3;
  if (rgb > MsgT::MAX_SIZE) {
    RCLCPP_ERROR(logger, "RGB %zu bytes exceeds TbMsg MAX_SIZE %u", rgb, MsgT::MAX_SIZE);
    return false;
  }

  auto loaned = publisher->borrow_loaned_message();
  if (!loaned.is_valid()) {
    RCLCPP_WARN(logger, "borrow_loaned_message failed");
    return false;
  }
  auto & msg = loaned.get();

  const int nv12_fd = DmabufHeapAlloc(allocator, "carveout-heap", nv12);
  if (nv12_fd < 0) {
    RCLCPP_WARN(logger, "NV12 carveout alloc failed");
    return false;
  }

  ta_image_t nv12_image{};
  int ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_NV12, &nv12_image, nv12_fd);
  if (ret == 0) {
    ret = ta_cv_image_jpeg_dec(bytes_used, static_cast<unsigned int>(cap_fd), &nv12_image);
  }
  if (ret != 0) {
    ta_cv_image_destroy_ext(&nv12_image);
    close(nv12_fd);
    RCLCPP_WARN(logger, "ta_cv_image_jpeg_dec failed (%d)", ret);
    return false;
  }

  const bool dma_loan = dmabuf_loan_backend_enabled();
  int rgb_fd = -1;
  uint64_t payload_phys = 0;
  if (dma_loan) {
    volatile unsigned char * base = msg.data.data();
    for (size_t off = 0; off < rgb; off += 4096) {
      (void)base[off];
    }
    DmabufHeapLookupPhysByPtr(allocator, msg.data.data(), &payload_phys);
    if (payload_phys != 0) {
      uint64_t len = rgb;
      rgb_fd = DmabufHeapLookupByPhys(allocator, payload_phys, &len);
    }
    if (rgb_fd < 0) {
      ta_cv_image_destroy_ext(&nv12_image);
      close(nv12_fd);
      RCLCPP_WARN(
        logger, "loan LOOKUP failed (phys=0x%llx)",
        static_cast<unsigned long long>(payload_phys));
      return false;
    }
  } else {
    rgb_fd = DmabufHeapAlloc(allocator, "carveout-heap", rgb);
    if (rgb_fd < 0) {
      ta_cv_image_destroy_ext(&nv12_image);
      close(nv12_fd);
      RCLCPP_WARN(logger, "RGB carveout alloc failed");
      return false;
    }
  }

  ta_image_t rgb_image{};
  ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_RGB_PACKED, &rgb_image, rgb_fd);
  if (ret == 0) {
    ta_cv_rect_t crop{};
    crop.start_x = 0;
    crop.start_y = 0;
    crop.crop_w = jpeg_w;
    crop.crop_h = jpeg_h;
    ta_cv_resize_t resize{};
    resize.in_width = jpeg_w;
    resize.in_height = jpeg_h;
    resize.out_width = jpeg_w;
    resize.out_height = jpeg_h;
    ta_cv_resize_image_t resize_attr{};
    resize_attr.resize_img_attr = &resize;
    resize_attr.interpolation = 5;
    ret = ta_cv_image_csc_convert_to(
      nv12_image, rgb_image, crop, &resize_attr, CSC_YCbCr2RGB_BT601);
  }
  ta_cv_image_destroy_ext(&rgb_image);
  ta_cv_image_destroy_ext(&nv12_image);
  close(nv12_fd);

  if (!dma_loan && ret == 0) {
    void * decoded = mmap(nullptr, rgb, PROT_READ, MAP_SHARED, rgb_fd, 0);
    if (decoded == MAP_FAILED) {
      ret = -1;
    } else {
      std::memcpy(msg.data.data(), decoded, rgb);
      munmap(decoded, rgb);
    }
  }
  close(rgb_fd);
  if (ret != 0) {
    RCLCPP_WARN(logger, "ta_cv_image_csc_convert_to failed (%d)", ret);
    return false;
  }

  msg.width = static_cast<uint32_t>(jpeg_w);
  msg.height = static_cast<uint32_t>(jpeg_h);
  msg.step = static_cast<uint32_t>(jpeg_w * 3);
  msg.data_size = static_cast<uint32_t>(rgb);
  fill_tb_img_encoding(msg.encoding, "rgb8");
  const auto stamp = camera->get_image_timestamp();
  msg.time_stamp.sec = stamp.tv_sec;
  msg.time_stamp.nanosec = stamp.tv_nsec;
  msg.index = frame_index++;
  msg.phys_addr = dma_loan ? payload_phys : 0;
  publisher->publish(std::move(loaned));
  return true;
}

}  // namespace

bool TopsbotUsbCamNode::take_and_send_dmabuf_nv12()
{
  if (m_nv12_alloc == nullptr) {
    m_nv12_alloc = CreateDmabufHeapBufferAllocator();
  }
  if (m_nv12_alloc == nullptr) {
    RCLCPP_WARN(get_logger(), "NV12 carveout allocator failed");
    return false;
  }

  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * jpeg = nullptr;
  if (!m_camera->dequeue_dmabuf(cap_fd, index, bytes_used, jpeg)) {
    return false;
  }
  RequeueDmabuf requeue(m_camera, index);

  if (bytes_used < 2 || jpeg == nullptr || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    RCLCPP_WARN(get_logger(), "DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
    return false;
  }

  int jpeg_w = 0;
  int jpeg_h = 0;
  if (parse_jpeg_dimensions(jpeg, bytes_used, &jpeg_w, &jpeg_h) != 0 ||
    jpeg_w != static_cast<int>(m_camera->get_image_width()) ||
    jpeg_h != static_cast<int>(m_camera->get_image_height()))
  {
    RCLCPP_WARN(
      get_logger(), "JPEG SOF %dx%d does not match capture %zux%zu",
      jpeg_w, jpeg_h, m_camera->get_image_width(), m_camera->get_image_height());
    return false;
  }

  const size_t nv12 = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3 / 2;
  if (nv12 != m_camera->get_image_size_in_bytes()) {
    RCLCPP_ERROR(
      get_logger(), "NV12 %zu bytes does not match payload %zu",
      nv12, m_camera->get_image_size_in_bytes());
    return false;
  }

  if (m_image_msg->data.size() != nv12) {
    m_image_msg->width = static_cast<uint32_t>(jpeg_w);
    m_image_msg->height = static_cast<uint32_t>(jpeg_h);
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      m_image_msg->step = static_cast<uint32_t>(nv12 / jpeg_h);
    }
    m_image_msg->data.resize(nv12);
  }

  const int out_fd = DmabufHeapAlloc(m_nv12_alloc, "carveout-heap", nv12);
  if (out_fd < 0) {
    RCLCPP_WARN(get_logger(), "NV12 carveout alloc failed");
    return false;
  }

  ta_image_t image{};
  int ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_NV12, &image, out_fd);
  if (ret == 0) {
    ret = ta_cv_image_jpeg_dec(bytes_used, static_cast<unsigned int>(cap_fd), &image);
    ta_cv_image_destroy_ext(&image);
  }
  if (ret == 0) {
    void * decoded = mmap(nullptr, nv12, PROT_READ, MAP_SHARED, out_fd, 0);
    if (decoded == MAP_FAILED) {
      ret = -1;
    } else {
      std::memcpy(m_image_msg->data.data(), decoded, nv12);
      munmap(decoded, nv12);
    }
  }
  close(out_fd);
  if (ret != 0) {
    RCLCPP_WARN(get_logger(), "ta_cv_image_jpeg_dec failed (%d)", ret);
    return false;
  }

  const auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool TopsbotUsbCamNode::take_and_send_dmabuf_rgb()
{
  if (m_nv12_alloc == nullptr) {
    m_nv12_alloc = CreateDmabufHeapBufferAllocator();
  }
  if (m_nv12_alloc == nullptr) {
    RCLCPP_WARN(get_logger(), "NV12 carveout allocator failed");
    return false;
  }

  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * jpeg = nullptr;
  if (!m_camera->dequeue_dmabuf(cap_fd, index, bytes_used, jpeg)) {
    return false;
  }
  RequeueDmabuf requeue(m_camera, index);

  if (bytes_used < 2 || jpeg == nullptr || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    RCLCPP_WARN(get_logger(), "DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
    return false;
  }

  int jpeg_w = 0;
  int jpeg_h = 0;
  if (parse_jpeg_dimensions(jpeg, bytes_used, &jpeg_w, &jpeg_h) != 0 ||
    jpeg_w != static_cast<int>(m_camera->get_image_width()) ||
    jpeg_h != static_cast<int>(m_camera->get_image_height()))
  {
    RCLCPP_WARN(
      get_logger(), "JPEG SOF %dx%d does not match capture %zux%zu",
      jpeg_w, jpeg_h, m_camera->get_image_width(), m_camera->get_image_height());
    return false;
  }
  if ((jpeg_w & 0xF) != 0) {
    RCLCPP_ERROR(get_logger(), "CSC output width %d is not a multiple of 16", jpeg_w);
    return false;
  }

  const size_t rgb = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3;
  const size_t nv12 = static_cast<size_t>(jpeg_w) * static_cast<size_t>(jpeg_h) * 3 / 2;
  if (rgb != m_camera->get_image_size_in_bytes()) {
    RCLCPP_ERROR(
      get_logger(), "RGB %zu bytes does not match payload %zu",
      rgb, m_camera->get_image_size_in_bytes());
    return false;
  }

  if (m_image_msg->data.size() != rgb) {
    m_image_msg->width = static_cast<uint32_t>(jpeg_w);
    m_image_msg->height = static_cast<uint32_t>(jpeg_h);
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      m_image_msg->step = static_cast<uint32_t>(jpeg_w * 3);
    }
    m_image_msg->data.resize(rgb);
  }

  const int nv12_fd = DmabufHeapAlloc(m_nv12_alloc, "carveout-heap", nv12);
  if (nv12_fd < 0) {
    RCLCPP_WARN(get_logger(), "NV12 carveout alloc failed");
    return false;
  }
  ta_image_t nv12_image{};
  int ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_NV12, &nv12_image, nv12_fd);
  if (ret == 0) {
    ret = ta_cv_image_jpeg_dec(bytes_used, static_cast<unsigned int>(cap_fd), &nv12_image);
  }
  if (ret != 0) {
    ta_cv_image_destroy_ext(&nv12_image);
    close(nv12_fd);
    RCLCPP_WARN(get_logger(), "ta_cv_image_jpeg_dec failed (%d)", ret);
    return false;
  }

  const int rgb_fd = DmabufHeapAlloc(m_nv12_alloc, "carveout-heap", rgb);
  if (rgb_fd < 0) {
    ta_cv_image_destroy_ext(&nv12_image);
    close(nv12_fd);
    RCLCPP_WARN(get_logger(), "RGB carveout alloc failed");
    return false;
  }

  ta_image_t rgb_image{};
  ret = ta_cv_image_create_ext(jpeg_h, jpeg_w, FORMAT_RGB_PACKED, &rgb_image, rgb_fd);
  if (ret == 0) {
    ta_cv_rect_t crop{};
    crop.crop_w = jpeg_w;
    crop.crop_h = jpeg_h;
    ta_cv_resize_t resize{};
    resize.in_width = jpeg_w;
    resize.in_height = jpeg_h;
    resize.out_width = jpeg_w;
    resize.out_height = jpeg_h;
    ta_cv_resize_image_t resize_attr{};
    resize_attr.resize_img_attr = &resize;
    resize_attr.interpolation = 5;
    ret = ta_cv_image_csc_convert_to(
      nv12_image, rgb_image, crop, &resize_attr, CSC_YCbCr2RGB_BT601);
  }
  ta_cv_image_destroy_ext(&rgb_image);
  ta_cv_image_destroy_ext(&nv12_image);
  close(nv12_fd);
  if (ret == 0) {
    void * decoded = mmap(nullptr, rgb, PROT_READ, MAP_SHARED, rgb_fd, 0);
    if (decoded == MAP_FAILED) {
      ret = -1;
    } else {
      std::memcpy(m_image_msg->data.data(), decoded, rgb);
      munmap(decoded, rgb);
    }
  }
  close(rgb_fd);
  if (ret != 0) {
    RCLCPP_WARN(get_logger(), "ta_cv_image_csc_convert_to failed (%d)", ret);
    return false;
  }

  const auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool TopsbotUsbCamNode::take_and_send_dmabuf_raw()
{
  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * pixels = nullptr;
  if (!m_camera->dequeue_dmabuf(cap_fd, index, bytes_used, pixels)) {
    return false;
  }
  RequeueDmabuf requeue(m_camera, index);
  (void)cap_fd;

  const auto format = m_camera->get_pixel_format();
  const auto frame_bytes = static_cast<uint32_t>(format->compute_frame_size_in_bytes(
    m_camera->get_image_width(), m_camera->get_image_height()));
  if (pixels == nullptr || bytes_used != frame_bytes) {
    RCLCPP_WARN(
      get_logger(), "raw pixel capture %u bytes does not match frame %u",
      bytes_used, frame_bytes);
    return false;
  }

  m_image_msg->width = m_camera->get_image_width();
  m_image_msg->height = m_camera->get_image_height();
  m_image_msg->encoding = format->ros();
  m_image_msg->step = m_camera->get_image_step();
  m_image_msg->data.assign(pixels, pixels + frame_bytes);
  const auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool TopsbotUsbCamNode::take_and_send_dmabuf_jpeg()
{
  int cap_fd = -1;
  unsigned int index = 0;
  uint32_t bytes_used = 0;
  const uint8_t * jpeg = nullptr;
  if (!m_camera->dequeue_dmabuf(cap_fd, index, bytes_used, jpeg)) {
    return false;
  }
  RequeueDmabuf requeue(m_camera, index);
  (void)cap_fd;

  if (bytes_used < 2 || jpeg == nullptr || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
    RCLCPP_WARN(get_logger(), "DQBUF is not a JPEG frame (bytes=%u)", bytes_used);
    return false;
  }

  m_compressed_img_msg->format = "jpeg";
  m_compressed_img_msg->data.assign(jpeg, jpeg + bytes_used);
  const auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;
  return true;
}

bool LoanedTbImgPublisher::publish_dmabuf_nv12(
  UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
{
  switch (profile_) {
    case TbImgProfile::MSG_480P:
      return publish_dmabuf_nv12_msg<tb_img_msgs::msg::TbMsg480P>(
        pub_480_, camera, allocator_, frame_index, logger);
    case TbImgProfile::MSG_540P:
      return publish_dmabuf_nv12_msg<tb_img_msgs::msg::TbMsg540P>(
        pub_540_, camera, allocator_, frame_index, logger);
    case TbImgProfile::MSG_1080P:
      return publish_dmabuf_nv12_msg<tb_img_msgs::msg::TbMsg1080P>(
        pub_1080_, camera, allocator_, frame_index, logger);
  }
  return false;
}

bool LoanedTbImgPublisher::publish_dmabuf_rgb(
  UsbCam * camera, int32_t & frame_index, const rclcpp::Logger & logger)
{
  switch (profile_) {
    case TbImgProfile::MSG_480P:
      return publish_dmabuf_rgb_msg<tb_img_msgs::msg::TbMsg480P>(
        pub_480_, camera, allocator_, frame_index, logger);
    case TbImgProfile::MSG_540P:
      return publish_dmabuf_rgb_msg<tb_img_msgs::msg::TbMsg540P>(
        pub_540_, camera, allocator_, frame_index, logger);
    case TbImgProfile::MSG_1080P:
      return publish_dmabuf_rgb_msg<tb_img_msgs::msg::TbMsg1080P>(
        pub_1080_, camera, allocator_, frame_index, logger);
  }
  return false;
}

}  // namespace topsbot_usb_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topsbot_usb_cam::TopsbotUsbCamNode)
