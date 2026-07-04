// Copyright 2026 TOPSBOT contributors.
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

#ifndef TOPSBOT_USB_CAM__FORMATS__NV12_HPP_
#define TOPSBOT_USB_CAM__FORMATS__NV12_HPP_

#include "linux/videodev2.h"

#include "topsbot_usb_cam/formats/pixel_format_base.hpp"

namespace topsbot_usb_cam
{
namespace formats
{

inline size_t yuv420sp_bytes_per_line(size_t width)
{
  return width;
}

inline size_t yuv420sp_frame_size(size_t width, size_t height)
{
  return width * height * 3 / 2;
}

/// V4L2 direct capture when the camera exposes NV12.
class NV12 : public pixel_format_base
{
public:
  explicit NV12(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "nv12",
      V4L2_PIX_FMT_NV12,
      topsbot_usb_cam::constants::NV12,
      1,
      8,
      false)
  {
    (void)args;
  }

  size_t compute_bytes_per_line(size_t width) const override
  {
    return yuv420sp_bytes_per_line(width);
  }

  size_t compute_frame_size_in_bytes(size_t width, size_t height) const override
  {
    return yuv420sp_frame_size(width, height);
  }
};

/// V4L2 direct capture when the camera exposes NV21.
class NV21 : public pixel_format_base
{
public:
  explicit NV21(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "nv21",
      V4L2_PIX_FMT_NV21,
      topsbot_usb_cam::constants::NV21,
      1,
      8,
      false)
  {
    (void)args;
  }

  size_t compute_bytes_per_line(size_t width) const override
  {
    return yuv420sp_bytes_per_line(width);
  }

  size_t compute_frame_size_in_bytes(size_t width, size_t height) const override
  {
    return yuv420sp_frame_size(width, height);
  }
};

}  // namespace formats
}  // namespace topsbot_usb_cam

#endif  // TOPSBOT_USB_CAM__FORMATS__NV12_HPP_
