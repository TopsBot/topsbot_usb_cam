# topsbot_usb_cam

[English](README.md)

# 功能介绍

`topsbot_usb_cam` 从 USB 摄像头采集图像，并通过 ROS 2 话题发布，供视觉、检测等下游节点订阅。

支持两种发布方式：

- **标准图像话题**（`sensor_msgs`）：适合 PC 预览、录包、`rqt_image_view` 调试
- **零拷贝共享内存**（`tb_img_msgs`）：适合板端低延迟链路（如 YOLO、CV 预处理）

本包基于 [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) v0.8.1 定制开发。

# 物料清单

| 序号 | 名称 | 说明 |
| ---- | ---- | ---- |
| 1 | USB 摄像头 | UVC 协议，建议支持 MJPEG 640×480 |
| 2 | 运行 ROS 2 Humble 的主机 | 如 TOPSBOT MES20 或带 USB 口的 Linux PC |

# 使用方法

## 硬件连接

将 USB 摄像头插入设备 USB 口。若已有其他程序占用摄像头，请先关闭后再启动本节点。

查看设备号：

```bash
ls /dev/video*
v4l2-ctl -d /dev/video0 --list-formats-ext
```

一般第一个可用设备为 `/dev/video0`；若不对，启动时用 `video_device:=/dev/videoX` 指定。

## 安装

### APT 安装

在已配置 TOPSBOT apt 源、运行 ROS 2 Humble 的设备上（如 MES20）：

```bash
source /opt/tbros/humble/setup.bash
sudo apt update
sudo apt install -y tbros-humble-topsbot-usb-cam
```

安装后无需克隆仓库即可查阅快速入门：

```bash
cat $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/QUICKSTART.md
ros2 launch topsbot_usb_cam usb_cam.launch.py -s
```

### 从源码交叉编译

面向 MES20 的交叉编译在 `topsbot_cc` Docker 环境中完成（x86 主机 + RISC-V sysroot）。将本仓库置于 `topsbot_cc/topsbot_ws/src/topsbot_usb_cam`，并确保 `topsbot_sysroot` 已就绪（启动 Docker 时会自动检查）。

**1. 进入交叉编译容器**

在 `topsbot_cc` 仓库根目录执行：

```bash
sudo topsbot_ws/tools/scripts/start_topsbot_cross_build_docker.sh
```

容器启动后工作目录为 `/mnt/topsbot_cc/topsbot_ws`。

**2. 安装依赖**

在容器内执行（首次编译或缺包时）：

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

**3. 编译**

```bash
./tools/scripts/build.sh -s topsbot_usb_cam
```

编译成功时输出类似：

```text
colcon build selction: topsbot_usb_cam
PACKAGE_SELECTION: --packages-select topsbot_usb_cam
build platform MES20
Starting >>> topsbot_usb_cam
Finished <<< topsbot_usb_cam [18.0s]

Summary: 1 package finished [18.1s]
```

产物位于 `topsbot_ws/install/`，同步至 MES20 后执行 `source install/setup.bash` 加载环境。Docker 镜像、sysroot 与工具链说明见同级 `topsbot_cc` 仓库文档。

## 启动相机

配置 ROS 2 环境后，在终端执行：

```bash
source /opt/tbros/humble/setup.bash

# 默认：640×480，MJPEG 解码为 RGB，零拷贝发布
ros2 launch topsbot_usb_cam usb_cam.launch.py

# 指定摄像头设备
ros2 launch topsbot_usb_cam usb_cam.launch.py video_device:=/dev/video0

# 对接 YOLO / NV12 下游（零拷贝）
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# PC 上预览（标准 ROS 图像，关闭共享内存环境）
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false
```

启动成功时，日志类似：

```text
Starting 'test_camera' (/dev/video0) at 640x480 via mmap (mjpeg2rgb) at 15 FPS, zero_copy=true
Publishing loaned tb_img_msgs on 'tbmem_img'
```

## 修改分辨率与图像格式

启动前先用 `v4l2-ctl` 确认摄像头**支持**目标分辨率和格式（见上文「硬件连接」）。若驱动层不支持，节点会启动失败或报 `Select timeout`。

### 修改分辨率、帧率

通过 launch 参数覆盖默认值（默认 640×480 @ 15fps）：

```bash
# 1280×720 @ 30fps，其余保持默认 yaml
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  image_width:=1280 image_height:=720 framerate:=30
```

### 修改图像格式

图像格式由 `pixel_format` 决定，推荐两种方式：

**方式一：切换 params_file（推荐）**

| 需求 | 命令 |
| ---- | ---- |
| RGB，板端零拷贝 | 默认（`mjpeg2rgb_640x480_zc.yaml`） |
| NV12，板端零拷贝 | `params_file:=mjpeg2nv12_640x480_zc.yaml` |
| MJPEG 码流零拷贝 | `params_file:=mjpeg_640x480_zc.yaml` |
| RGB，PC 预览 | `params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false` |
| 原始 MJPEG 压缩流 | `params_file:=mjpeg_640x480_compressed.yaml setup_shm:=false` |

示例：

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2nv12_640x480_zc.yaml
```

**方式二：launch 参数直接指定 `pixel_format`**

在保持当前 yaml 其他设置不变时，可单独覆盖格式：

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py pixel_format:=mjpeg2nv12
```

### 常用 `pixel_format` 说明

| `pixel_format` | 输出内容 | 适用场景 |
| -------------- | -------- | -------- |
| `mjpeg2rgb` | 解码后的 RGB | 通用视觉、预览（**推荐**） |
| `mjpeg2nv12` | 解码后的 NV12 | YOLO、CV 预处理（**推荐**） |
| `mjpeg` | 原始 JPEG 压缩流 | `TbJpegFrame` 零拷贝（`/tbmem_jpeg`）或 `CompressedImage` |
| `yuyv` | 未压缩 YUYV | 仅当 v4l2 验证可用时再试 |

参数 `mjpeg` 会自动映射为内部格式 `raw_mjpeg`。嵌入式 USB 摄像头优先选用 `mjpeg2rgb` 或 `mjpeg2nv12`。

### 组合示例

```bash
# 1280×720，NV12 零拷贝，30fps
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2nv12_640x480_zc.yaml \
  image_width:=1280 image_height:=720 framerate:=30
```

## 查看效果

预览方式取决于当前 `params_file` 发布的**话题类型**（见下表）。零拷贝的 `tb_img_msgs` / `TbJpegFrame` **不能**直接用 RViz、`rqt_image_view` 订阅。

| 相机配置（`params_file`） | 话题 | 消息类型 | 推荐预览 |
|---------------------------|------|----------|----------|
| `mjpeg2nv12_640x480_zc.yaml` 等零拷贝 | `/tbmem_img` | `tb_img_msgs` | **[`topsbot_webviz`](../topsbot_webviz)** 浏览器（`tb_img.yaml`） |
| `mjpeg_640x480_zc.yaml` | `/tbmem_jpeg` | `TbJpegFrame` | **topsbot_webviz**（`tb_jpeg.yaml`） |
| `mjpeg2rgb_640x480_sensor.yaml` 等 | `/image_raw` | `sensor_msgs/Image` | **`rqt_image_view`**、RViz2 Image；或 topsbot_webviz（`sensor_image.yaml`） |
| `mjpeg_640x480_compressed.yaml` | `/image_raw/compressed` | `sensor_msgs/CompressedImage` | topsbot_webviz（`sensor_compressed.yaml`）；部分环境可用 `rqt_image_view` |
| 联调 YOLO 完整链路 | `/tbmem_img` + `/detections` | — | **topsbot_yolo_det** `cam_yolo_webviz_demo.launch.py`（内置 webviz） |

### 板端浏览器预览（零拷贝，推荐）

相机保持零拷贝 yaml 不变，另开 **topsbot_webviz**：

```bash
# MES20 — 相机已发布 /tbmem_img（NV12 零拷贝）
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# MES20 — 另开终端
ros2 launch topsbot_webviz webviz.launch.py params_file:=tb_img.yaml
```

浏览器打开 `http://<板卡IP>:8000/`。MJPEG 零拷贝话题 `/tbmem_jpeg` 时用 `params_file:=tb_jpeg.yaml`。

### PC / 标准图像（rqt、RViz）

需要 `sensor_msgs/Image` 时，切换 sensor 预设后用常见 ROS 工具：

```bash
# MES20 或 PC
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false

# 另开终端 — rqt 选话题 /image_raw
ros2 run rqt_image_view rqt_image_view

# 或 RViz2：Add → Image → Topic 选 /image_raw
```

NV12 sensor 路径（`mjpeg2nv12_640x480_sensor.yaml`）同样发布 `/image_raw`，`rqt_image_view` 可订阅（显示效果取决于 `encoding`）。

### 仅确认话题有数据（不看画面）

零拷贝模式（默认）：

```bash
ros2 topic hz /tbmem_img
ros2 topic echo /tbmem_img --once
```

标准图像模式：

```bash
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once
```

### 多路 USB 相机

```bash
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py
# 默认两路：/cam0/image_raw、/cam1/image_raw（零拷贝：/cam0/tbmem_img、/cam1/tbmem_img）
# 默认设备：cam0=/dev/video0，cam1=/dev/video2
# 填写 cam2/cam3 设备后自动启用四路：
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py \
  cam0_video_device:=/dev/video6 cam1_video_device:=/dev/video8 \
  cam2_video_device:=/dev/video14 cam3_video_device:=/dev/video16
# 查看/修改：ros2 launch topsbot_usb_cam usb_cam_multi.launch.py -s
```

多相机通过 ROS namespace（`cam0`～`cam3`）区分，**namespace 内话题名与单相机相同**，无需配置话题名。最多四路，**默认启动 2 路**（cam0/cam1）；四路需 `channel_count:=4` 并配置 cam2/cam3 设备。

# 接口说明

## 发布话题

话题名由 `pixel_format` + `zero_copy` 自动决定，**不可通过参数或 yaml 修改**。多相机时 launch 为每路设置 namespace（`cam0`～`cam3`），话题在对应 namespace 下与单相机同名（见下表「多相机」列）。

| 条件 | 单相机话题 | 多相机话题（usb_cam_multi） | 消息类型 |
| ---- | ---------- | ------------------------- | -------- |
| `zero_copy:=true`，非 raw mjpeg | `/tbmem_img` | `/camN/tbmem_img` | `tb_img_msgs` NV12/RGB |
| `zero_copy:=true`，`pixel_format:=mjpeg` | `/tbmem_jpeg` | `/camN/tbmem_jpeg` | `TbJpegFrame` |
| `zero_copy:=false`，非 mjpeg | `/image_raw` | `/camN/image_raw` | `sensor_msgs/Image` |
| `zero_copy:=false`，`pixel_format:=mjpeg` | `/image_raw/compressed` | `/camN/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| 标准模式 | `/camera_info` | `/camN/camera_info` | `sensor_msgs/CameraInfo` |

零拷贝模式不发布 `camera_info`。

## Launch 参数

| 参数名 | 说明 | 类型 | 默认值 |
| ------ | ---- | ---- | ------ |
| `params_file` | 全量参数 yaml，位于 `config/` | string | `mjpeg2rgb_640x480_zc.yaml` |
| `video_device` | 摄像头设备 | string | `/dev/video0` |
| `image_width` | 图像宽度 | int | `640` |
| `image_height` | 图像高度 | int | `480` |
| `framerate` | 帧率（整数） | int | `15` |
| `frame_id` | 坐标系名称 | string | `usb_cam` |
| `pixel_format` | 像素格式，覆盖 yaml | string | （空，沿用 yaml） |
| `setup_shm` | 是否配置零拷贝 DDS 环境（按 `RMW_IMPLEMENTATION` 自动识别；Cyclone 时自动启停 `iox-roudi`） | bool | `true` |
| `zero_copy` | 覆盖 yaml 中的零拷贝开关 | bool | （空，沿用 yaml） |

内置配置列表：`ls .../config/*_*x*_*.yaml`（详见 `config/README.md`）

完整 launch 参数：`ros2 launch topsbot_usb_cam usb_cam.launch.py -s`

## 节点参数

| 参数名 | 说明 | 类型 | 默认值 |
| ------ | ---- | ---- | ------ |
| `video_device` | 设备路径 | string | `/dev/video0` |
| `image_width` | 图像宽度 | int | `640` |
| `image_height` | 图像高度 | int | `480` |
| `framerate` | 帧率 | int | `15` |
| `pixel_format` | 像素格式 | string | 见 `params_file` yaml |
| `zero_copy` | 是否零拷贝 | bool | 见 `params_file` yaml |
| `frame_id` | 坐标系名称 | string | `usb_cam` |
| `camera_info_url` | 标定文件路径 | string | `package://topsbot_usb_cam/config/camera_info.yaml` |

支持的 `pixel_format` 包括：`mjpeg`、`mjpeg2rgb`、`mjpeg2nv12`、`yuyv`、`nv12`、`nv21` 等。嵌入式 USB 摄像头**建议优先使用 `mjpeg2rgb` 或 `mjpeg2nv12`**。

发布话题由 `pixel_format` + `zero_copy` 自动决定（见上表与 `config/README.md`），yaml 中无需也不支持话题相关参数。

完整参数见 `config/*_*x*_*.yaml` 与 `config/params_reference.yaml`。运行中可用 `ros2 param describe /usb_cam <name>` 查看说明。

# 常见问题

**1. 如何确认摄像头设备号？**

插入 USB 摄像头后执行 `ls /dev/video*`，一般为 `/dev/video0`。若启动报 `Device is not available`，换一个设备号或检查是否被其他进程占用（`fuser /dev/video0`）。

**2. 启动后报 `Select timeout` 或很快退出？**

先用 v4l2 测试能否出帧：

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=30
```

若此处卡住，说明驱动层不支持当前格式，请改用 `mjpeg2rgb_640x480_sensor.yaml` 等 MJPEG 相关配置，或更换 USB 口 / 摄像头。

**3. PC 上看不到图像？**

- 确认已 `source /opt/tbros/humble/setup.bash`
- 使用 `sensor_msgs` 预设：`params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false`，再用 `rqt_image_view` 订阅 `/image_raw`
- 用 `ros2 topic list` 和 `ros2 topic hz /image_raw` 确认有数据
- 跨机查看时，两端 `ROS_DOMAIN_ID` 需一致

**4. 零拷贝模式下如何看图？**

`/tbmem_img` 为 `tb_img_msgs`，`rqt` / RViz 无法直接显示。可选：

- **板端浏览器**：`topsbot_webviz` + `tb_img.yaml`（见上文「查看效果」）
- **改用标准图像**：`mjpeg2rgb_640x480_sensor.yaml` + `rqt_image_view`

**5. 没有相机标定文件会影响使用吗？**

不影响取流和发布。无标定文件时 `/camera_info` 使用默认值；有标定需求时，修改 `camera_info_url` 指向你的标定 yaml。

**6. 如何对接 YOLO 检测 demo？**

```bash
ros2 launch topsbot_yolo_det cam_yolo_webviz_demo.launch.py
```

该 launch 会自动以 NV12 零拷贝配置启动相机。

**7. `framerate` 参数报错？**

`framerate` 必须为**整数**（如 `15`），不要写 `15.0`。

**8. 如何修改分辨率和图像格式？**

见上文「修改分辨率与图像格式」。修改后若无法出图，先用 `v4l2-ctl --list-formats-ext` 核对摄像头是否支持该分辨率与 MJPEG/YUYV 格式。

---

## 许可与上游

| 组件 | 许可 |
| ---- | ---- |
| 源自 `usb_cam` 的代码 | BSD 2-Clause（`LICENSE.BSD`、`NOTICE`） |
| TOPSBOT 新增/修改代码 | Apache-2.0（`LICENSE`） |

二次分发时请保留 `LICENSE`、`LICENSE.BSD` 与 `NOTICE`。
