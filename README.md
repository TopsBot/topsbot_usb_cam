# topsbot_usb_cam

[中文文档](README_cn.md)

# Overview

`topsbot_usb_cam` captures images from USB cameras and publishes them on ROS 2 topics for downstream vision and detection nodes.

Two publishing modes are supported:

- **Standard image topics** (`sensor_msgs`): for PC preview, bag recording, and `rqt_image_view` debugging
- **Zero-copy shared memory** (`tb_img_msgs`): for low-latency on-board pipelines (e.g. YOLO, CV preprocessing)

This package is customized from [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) v0.8.1.

# Bill of Materials

| # | Item | Notes |
| - | ---- | ----- |
| 1 | USB camera | UVC protocol; MJPEG 640×480 recommended |
| 2 | ROS 2 Humble host | e.g. TOPSBOT MES20 or a Linux PC with USB ports |

# Usage

## Hardware Setup

Plug the USB camera into a USB port. Close any other program that may be using the camera before starting this node.

List video devices:

```bash
ls /dev/video*
v4l2-ctl -d /dev/video0 --list-formats-ext
```

The first available device is usually `/dev/video0`. If not, specify it at launch with `video_device:=/dev/videoX`.

## Installation

### apt

On a ROS 2 Humble device with the TOPSBOT apt repository configured (e.g. MES20):

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-topsbot-usb-cam
```

### Cross-compile from source

Cross-compilation for MES20 is done in the `topsbot_cc` Docker environment (x86 host + RISC-V sysroot). Place this repo under `topsbot_cc/topsbot_ws/src/topsbot_usb_cam` and ensure `topsbot_sysroot` is ready (the Docker startup script checks this automatically).

**1. Enter the cross-build container**

From the `topsbot_cc` repo root:

```bash
sudo topsbot_ws/tools/start_topsbot_cross_build_docker.sh
```

The working directory inside the container is `/mnt/topsbot_cc/topsbot_ws`.

**2. Install dependencies**

Inside the container (first build or when packages are missing):

```bash
./tools/sysroot_apt.sh update
./tools/sysroot_apt.sh install \
  libgflags-dev nlohmann-json3-dev \
  ros-humble-image-transport ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport ros-humble-image-publisher \
  ros-humble-camera-info-manager ros-humble-diagnostic-updater \
  ros-humble-diagnostic-msgs ros-humble-statistics-msgs ros-humble-xacro \
  ros-humble-backward-ros libdw-dev libssl-dev mesa-utils libgl1 \
  libgoogle-glog-dev ros-humble-tb-img-msgs
```

**3. Build**

```bash
./tools/build.sh -s topsbot_usb_cam
```

On success, output looks like:

```text
colcon build selction: topsbot_usb_cam
PACKAGE_SELECTION: --packages-select topsbot_usb_cam
build platform MES20
Starting >>> topsbot_usb_cam
Finished <<< topsbot_usb_cam [18.0s]

Summary: 1 package finished [18.1s]
```

Artifacts are under `topsbot_ws/install/`. After syncing to MES20, run `source install/setup.bash`. See the sibling `topsbot_cc` repo for Docker, sysroot, and toolchain details.

## Launch the camera

After sourcing the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash

# Default: 640×480, MJPEG decoded to RGB, zero-copy publishing
ros2 launch topsbot_usb_cam usb_cam.launch.py

# Specify camera device
ros2 launch topsbot_usb_cam usb_cam.launch.py video_device:=/dev/video0

# NV12 zero-copy for YOLO / CV downstream
ros2 launch topsbot_usb_cam usb_cam.launch.py preset:=presets/mjpeg2nv12_zerocopy.yaml

# PC preview (standard ROS image, shared-memory env disabled)
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  preset:=presets/mjpeg2rgb_sensor.yaml setup_shm:=false
```

On success, logs look like:

```text
Starting 'test_camera' (/dev/video0) at 640x480 via mmap (mjpeg2rgb) at 15 FPS, zero_copy=true
Publishing loaned tb_img_msgs on 'tbmem_img'
```

## Resolution and pixel format

Before launch, use `v4l2-ctl` to confirm the camera **supports** the target resolution and format (see Hardware Setup). If the driver does not support them, the node may fail to start or report `Select timeout`.

### Resolution and frame rate

Override defaults (640×480 @ 15 fps) via launch arguments:

```bash
# 1280×720 @ 30 fps, keep default preset for everything else
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  image_width:=1280 image_height:=720 framerate:=30
```

### Pixel format

Pixel format is controlled by `pixel_format`. Two approaches:

**Option 1: switch preset (recommended)**

| Goal | Command |
| ---- | ------- |
| RGB, on-board zero-copy | `preset:=presets/mjpeg2rgb_zerocopy.yaml` (default) |
| NV12 for YOLO / CV | `preset:=presets/mjpeg2nv12_zerocopy.yaml` |
| RGB, PC preview | `preset:=presets/mjpeg2rgb_sensor.yaml setup_shm:=false` |
| Raw MJPEG compressed stream | `preset:=presets/mjpeg_compressed.yaml setup_shm:=false` |

Example:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  preset:=presets/mjpeg2nv12_zerocopy.yaml
```

**Option 2: override `pixel_format` directly**

Keep other preset settings and override format only:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py pixel_format:=mjpeg2nv12
```

### Common `pixel_format` values

| `pixel_format` | Output | Use case |
| -------------- | ------ | -------- |
| `mjpeg2rgb` | Decoded RGB | General vision, preview (**recommended**) |
| `mjpeg2nv12` | Decoded NV12 | YOLO, CV preprocessing (**recommended**) |
| `mjpeg` | Raw JPEG bitstream | Recording, streaming, custom decode |
| `yuyv` | Uncompressed YUYV | Only after v4l2 verification |

The `mjpeg` argument is mapped internally to `raw_mjpeg`. For embedded USB cameras, prefer `mjpeg2rgb` or `mjpeg2nv12`.

### Combined example

```bash
# 1280×720, NV12 zero-copy, 30 fps
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  preset:=presets/mjpeg2nv12_zerocopy.yaml \
  image_width:=1280 image_height:=720 framerate:=30
```

## Viewing output

The zero-copy topic `/tbmem_img` uses custom `tb_img_msgs` types; RViz / rqt cannot subscribe directly. For image preview, switch to the `mjpeg2rgb_sensor` preset (see Option 1 below).

### Option 1: PC preview

Use the `mjpeg2rgb_sensor` preset, then subscribe with `rqt_image_view` on `/image_raw`:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  preset:=presets/mjpeg2rgb_sensor.yaml setup_shm:=false

# In another terminal
ros2 run rqt_image_view rqt_image_view
```

### Option 2: Verify topic data

Zero-copy mode (default):

```bash
ros2 topic hz /tbmem_img
ros2 topic echo /tbmem_img --once
```

Standard image mode:

```bash
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once
```

### Option 3: Dual USB cameras

```bash
ros2 launch topsbot_usb_cam dual_camera.launch.py
# Topics: /cam0/image_raw, /cam1/image_raw
```

# Interface reference

## Published topics

| Topic | Message type | Notes |
| ----- | ------------ | ----- |
| `/tbmem_img` | `tb_img_msgs/msg/TbMsg480P` etc. | Zero-copy mode (`zero_copy:=true`) |
| `/image_raw` | `sensor_msgs/msg/Image` | Standard mode, decoded RGB/YUYV etc. |
| `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Standard mode, raw MJPEG stream |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Standard mode only; not published in zero-copy mode |

## Launch arguments

| Argument | Description | Type | Default |
| -------- | ----------- | ---- | ------- |
| `preset` | Scenario config under `config/` | string | `presets/mjpeg2rgb_zerocopy.yaml` |
| `video_device` | Camera device path | string | `/dev/video0` |
| `image_width` | Image width | int | `640` |
| `image_height` | Image height | int | `480` |
| `framerate` | Frame rate (integer) | int | `15` |
| `pixel_format` | Override preset pixel format | string | (empty, use preset) |
| `setup_shm` | Configure Fast DDS shared-memory env | bool | `true` |
| `zero_copy` | Override preset zero-copy flag | bool | (empty, use preset) |

Common presets:

| preset | Use case |
| ------ | -------- |
| `presets/mjpeg2rgb_zerocopy.yaml` | On-board RGB zero-copy (**default**) |
| `presets/mjpeg2nv12_zerocopy.yaml` | On-board NV12 zero-copy (YOLO / CV) |
| `presets/mjpeg2rgb_sensor.yaml` | PC / rqt preview |
| `presets/mjpeg_compressed.yaml` | Raw MJPEG stream, lower CPU |

## Node parameters

| Parameter | Description | Type | Default |
| --------- | ----------- | ---- | ------- |
| `video_device` | Device path | string | `/dev/video0` |
| `image_width` | Image width | int | `640` |
| `image_height` | Image height | int | `480` |
| `framerate` | Frame rate | int | `15` |
| `pixel_format` | Pixel format | string | See preset |
| `zero_copy` | Enable zero-copy | bool | See preset |
| `tbmem_topic` | Zero-copy topic name | string | `tbmem_img` |
| `frame_id` | TF frame name | string | `usb_cam` |
| `camera_info_url` | Calibration file URL | string | `package://topsbot_usb_cam/config/camera_info.yaml` |

Supported `pixel_format` values include `mjpeg`, `mjpeg2rgb`, `mjpeg2nv12`, `yuyv`, `nv12`, `nv21`, etc. For embedded USB cameras, **prefer `mjpeg2rgb` or `mjpeg2nv12`**.

Full parameter lists are in `config/usb_cam_common.yaml` and `config/presets/*.yaml`.

# FAQ

**1. How do I find the camera device?**

After plugging in the camera, run `ls /dev/video*` — usually `/dev/video0`. If launch reports `Device is not available`, try another device index or check for another process (`fuser /dev/video0`).

**2. `Select timeout` or immediate exit after launch?**

Test streaming with v4l2 first:

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=30
```

If this hangs, the driver does not support the current format — switch to an MJPEG preset or try another USB port / camera.

**3. No image on PC?**

- Ensure `source /opt/ros/humble/setup.bash` was run
- For preview, use `preset:=presets/mjpeg2rgb_sensor.yaml setup_shm:=false`
- Check `ros2 topic list` and `ros2 topic hz /image_raw`
- For cross-machine viewing, `ROS_DOMAIN_ID` must match on both sides

**4. How to view images in rqt / RViz with zero-copy?**

`/tbmem_img` publishes `tb_img_msgs`, which generic tools do not support. Switch to a standard image preset:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  preset:=presets/mjpeg2rgb_sensor.yaml setup_shm:=false
```

**5. Does missing camera calibration block streaming?**

No. Without a calibration file, `/camera_info` uses defaults. Point `camera_info_url` to your calibration yaml when needed.

**6. How to run the YOLO detection demo?**

```bash
ros2 launch topsbot_yolo_det cam_yolo_demo.launch.py
```

That launch starts the camera with the NV12 zero-copy preset automatically.

**7. `framerate` argument error?**

`framerate` must be an **integer** (e.g. `15`), not `15.0`.

**8. How to change resolution and format?**

See Resolution and pixel format above. If the stream fails after changes, verify support with `v4l2-ctl --list-formats-ext`.

---

## License and upstream

| Component | License |
| --------- | ------- |
| Code derived from `usb_cam` | BSD 2-Clause (`LICENSE.BSD`, `NOTICE`) |
| TOPSBOT additions and modifications | Apache-2.0 (`LICENSE`) |

Redistributions must retain `LICENSE`, `LICENSE.BSD`, and `NOTICE`.
