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
source /opt/tbros/humble/setup.bash
sudo apt update
sudo apt install -y tbros-humble-topsbot-usb-cam
```

After install, without cloning the repo:

```bash
cat $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/QUICKSTART.md
ros2 launch topsbot_usb_cam usb_cam.launch.py -s
```

### Cross-compile from source

Cross-compilation for MES20 is done in the `topsbot_cc` Docker environment (x86 host + RISC-V sysroot). Place this repo under `topsbot_cc/topsbot_ws/src/topsbot_usb_cam` and ensure `topsbot_sysroot` is ready (the Docker startup script checks this automatically).

**1. Enter the cross-build container**

From the `topsbot_cc` repo root:

```bash
sudo topsbot_ws/tools/scripts/start_topsbot_cross_build_docker.sh
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
  libgoogle-glog-dev tbros-humble-tb-img-msgs
```

**3. Build**

```bash
./tools/scripts/build.sh -s topsbot_usb_cam
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
source /opt/tbros/humble/setup.bash

# Default: 640×480, MJPEG decoded to RGB, zero-copy publishing
ros2 launch topsbot_usb_cam usb_cam.launch.py

# Specify camera device
ros2 launch topsbot_usb_cam usb_cam.launch.py video_device:=/dev/video0

# NV12 zero-copy for YOLO / CV downstream
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# PC preview (standard ROS image, shared-memory env disabled)
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false
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
# 1280×720 @ 30 fps, keep default yaml for everything else
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  image_width:=1280 image_height:=720 framerate:=30
```

### Pixel format

Pixel format is controlled by `pixel_format`. Two approaches:

**Option 1: switch params_file (recommended)**

| Goal | Command |
| ---- | ------- |
| RGB, on-board zero-copy | default (`mjpeg2rgb_640x480_zc.yaml`) |
| NV12 zero-copy | `params_file:=mjpeg2nv12_640x480_zc.yaml` |
| RGB, PC preview | `params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false` |
| Raw MJPEG compressed stream | `params_file:=mjpeg_640x480_compressed.yaml setup_shm:=false` |

Example:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2nv12_640x480_zc.yaml
```

**Option 2: override `pixel_format` directly**

Keep other yaml settings and override format only:

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
  params_file:=mjpeg2nv12_640x480_zc.yaml \
  image_width:=1280 image_height:=720 framerate:=30
```

## Viewing output

Choose a preview tool based on the **topic type** from your current `params_file` (table below). Zero-copy `tb_img_msgs` / `TbJpegFrame` topics **cannot** be subscribed by RViz or `rqt_image_view` directly.

| Camera config (`params_file`) | Topic | Message type | Recommended preview |
|-------------------------------|-------|--------------|---------------------|
| `mjpeg2nv12_640x480_zc.yaml` etc. (zero-copy) | `/tbmem_img` | `tb_img_msgs` | **[`topsbot_webviz`](../topsbot_webviz)** browser (`tb_img.yaml`) |
| `mjpeg_640x480_zc.yaml` | `/tbmem_jpeg` | `TbJpegFrame` | **topsbot_webviz** (`tb_jpeg.yaml`) |
| `mjpeg2rgb_640x480_sensor.yaml` etc. | `/image_raw` | `sensor_msgs/Image` | **`rqt_image_view`**, RViz2 Image; or topsbot_webviz (`sensor_image.yaml`) |
| `mjpeg_640x480_compressed.yaml` | `/image_raw/compressed` | `sensor_msgs/CompressedImage` | topsbot_webviz (`sensor_compressed.yaml`); `rqt_image_view` in some setups |
| YOLO full pipeline | `/tbmem_img` + `/detections` | — | **topsbot_yolo_det** `cam_yolo_webviz_demo.launch.py` (webviz built in) |

### Board browser preview (zero-copy, recommended)

Keep the zero-copy camera yaml; start **topsbot_webviz** separately:

```bash
# MES20 — camera publishing /tbmem_img (NV12 zero-copy)
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# MES20 — another terminal
ros2 launch topsbot_webviz webviz.launch.py params_file:=tb_img.yaml
```

Open `http://<board-ip>:8000/` in a browser. For `/tbmem_jpeg`, use `params_file:=tb_jpeg.yaml`.

### PC / standard ROS image (rqt, RViz)

For `sensor_msgs/Image`, use the sensor preset and common ROS tools:

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false

# Another terminal — select /image_raw in rqt
ros2 run rqt_image_view rqt_image_view

# Or RViz2: Add → Image → Topic /image_raw
```

The NV12 sensor path (`mjpeg2nv12_640x480_sensor.yaml`) also publishes `/image_raw`; `rqt_image_view` can subscribe (display depends on `encoding`).

### Verify topic data only (no image)

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

### Multi USB cameras

```bash
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py
# Default two cameras: /cam0/image_raw, /cam1/image_raw
# Defaults: cam0=/dev/video0, cam1=/dev/video2 — see: ... usb_cam_multi.launch.py -s
# Adding cam2/cam3 devices automatically starts four cameras:
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py \
  cam0_video_device:=/dev/video6 cam1_video_device:=/dev/video8 \
  cam2_video_device:=/dev/video14 cam3_video_device:=/dev/video16
```

Multi-camera launch uses ROS namespaces (`cam0` … `cam3`); **topic basenames match single-camera** under each namespace. Up to four cameras; **default is 2** (cam0/cam1). Use `channel_count:=4` with `cam2_video_device` / `cam3_video_device` for four cameras.

# Interface reference

## Published topics

Topic names follow `pixel_format` + `zero_copy` (not user-configurable). Multi-camera launch puts each node in namespace `cam0` … `cam3`; topics under that namespace use the same basenames as single-camera.

| Condition | Single camera | Multi camera (`usb_cam_multi`) | Message type |
| --------- | ------------- | --------------------------- | -------------- |
| `zero_copy:=true`, not raw mjpeg | `/tbmem_img` | `/camN/tbmem_img` | `tb_img_msgs` NV12/RGB |
| `zero_copy:=true`, `pixel_format:=mjpeg` | `/tbmem_jpeg` | `/camN/tbmem_jpeg` | `TbJpegFrame` |
| `zero_copy:=false`, not mjpeg | `/image_raw` | `/camN/image_raw` | `sensor_msgs/Image` |
| `zero_copy:=false`, `pixel_format:=mjpeg` | `/image_raw/compressed` | `/camN/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| Standard mode | `/camera_info` | `/camN/camera_info` | `sensor_msgs/CameraInfo` |

Zero-copy mode does not publish `camera_info`.

## Launch arguments

| Argument | Description | Type | Default |
| -------- | ----------- | ---- | ------- |
| `params_file` | Full params yaml under `config/` | string | `mjpeg2rgb_640x480_zc.yaml` |
| `video_device` | Camera device path | string | `/dev/video0` |
| `image_width` | Image width | int | `640` |
| `image_height` | Image height | int | `480` |
| `framerate` | Frame rate (integer) | int | `15` |
| `frame_id` | Frame id | string | `usb_cam` |
| `pixel_format` | Override yaml pixel format | string | (empty, use yaml) |
| `setup_shm` | Configure zero-copy DDS env (auto-detects `RMW_IMPLEMENTATION`; starts/stops `iox-roudi` for Cyclone) | bool | `true` |
| `zero_copy` | Override yaml zero-copy flag | bool | (empty, use yaml) |

Built-in configs: `ls .../config/*_*x*_*.yaml` (see `config/README.md`).

Full launch args: `ros2 launch topsbot_usb_cam usb_cam.launch.py -s`

## Node parameters

| Parameter | Description | Type | Default |
| --------- | ----------- | ---- | ------- |
| `video_device` | Device path | string | `/dev/video0` |
| `image_width` | Image width | int | `640` |
| `image_height` | Image height | int | `480` |
| `framerate` | Frame rate | int | `15` |
| `pixel_format` | Pixel format | string | See `params_file` yaml |
| `zero_copy` | Enable zero-copy | bool | See `params_file` yaml |
| `frame_id` | TF frame name | string | `usb_cam` |
| `camera_info_url` | Calibration file URL | string | `package://topsbot_usb_cam/config/camera_info.yaml` |

Supported `pixel_format` values include `mjpeg`, `mjpeg2rgb`, `mjpeg2nv12`, `yuyv`, `nv12`, `nv21`, etc. For embedded USB cameras, **prefer `mjpeg2rgb` or `mjpeg2nv12`**.

Publish topics are derived from `pixel_format` + `zero_copy` (see `config/README.md`); built-in yaml files do not set `tbmem_topic` etc. Use `ros2 param set` for advanced overrides.

Full parameter reference: `config/*_*x*_*.yaml` and `config/params_reference.yaml`. At runtime: `ros2 param describe /usb_cam <name>`.

# FAQ

**1. How do I find the camera device?**

After plugging in the camera, run `ls /dev/video*` — usually `/dev/video0`. If launch reports `Device is not available`, try another device index or check for another process (`fuser /dev/video0`).

**2. `Select timeout` or immediate exit after launch?**

Test streaming with v4l2 first:

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=30
```

If this hangs, the driver does not support the current format — switch to an MJPEG config such as `mjpeg2rgb_640x480_sensor.yaml` or try another USB port / camera.

**3. No image on PC?**

- Ensure `source /opt/tbros/humble/setup.bash` was run
- Use a `sensor_msgs` preset: `params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false`, then `rqt_image_view` on `/image_raw`
- Check `ros2 topic list` and `ros2 topic hz /image_raw`
- For cross-machine viewing, `ROS_DOMAIN_ID` must match on both sides

**4. How to view images with zero-copy?**

`/tbmem_img` uses `tb_img_msgs`; rqt / RViz cannot display it directly. Options:

- **Board browser**: `topsbot_webviz` + `tb_img.yaml` (see **Viewing output** above)
- **Standard image path**: `mjpeg2rgb_640x480_sensor.yaml` + `rqt_image_view`

**5. Does missing camera calibration block streaming?**

No. Without a calibration file, `/camera_info` uses defaults. Point `camera_info_url` to your calibration yaml when needed.

**6. How to run the YOLO detection demo?**

```bash
ros2 launch topsbot_yolo_det cam_yolo_webviz_demo.launch.py
```

That launch starts the camera with NV12 zero-copy config automatically.

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
