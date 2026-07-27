# topsbot_usb_cam 快速入门（apt / install 用户）

## 1. 启动相机（主入口）

```bash
source /opt/tbros/humble/setup.bash   # 或 source install/setup.bash

ros2 launch topsbot_usb_cam usb_cam.launch.py
```

默认加载 `config/mjpeg2rgb_640x480_zc.yaml`（板端 RGB 零拷贝 640×480）。

## 2. 本包有哪些 launch？

```bash
ls $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/launch/
```

| launch 文件 | 用途 |
|-------------|------|
| `usb_cam.launch.py` | **单路 USB 相机（常用）** |
| `usb_cam_multi.launch.py` | **多路 USB 相机**（最多 `cam0`～`cam3`） |

多路示例（**默认 2 路** cam0/cam1；四路需 `channel_count:=4`）：

```bash
# 默认两路
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py \
  cam0_video_device:=/dev/video0 cam1_video_device:=/dev/video2

# 四路
ros2 launch topsbot_usb_cam usb_cam_multi.launch.py \
  channel_count:=4 \
  cam0_video_device:=/dev/video0 cam1_video_device:=/dev/video2 \
  cam2_video_device:=/dev/video14 cam3_video_device:=/dev/video16

| `zc_env.launch.py` | 零拷贝环境（通常由 `usb_cam` / `usb_cam_multi` 自动 include） |

## 3. 有哪些内置配置？

```bash
ls $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/*_*x*_*.yaml
```

详见同目录 `config/README.md`。

## 4. 查看可改参数

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py -s
```

## 5. 常用启动示例

```bash
# 换设备
ros2 launch topsbot_usb_cam usb_cam.launch.py video_device:=/dev/video2

# 换分辨率（覆盖 yaml 中的宽高）
ros2 launch topsbot_usb_cam usb_cam.launch.py image_width:=1280 image_height:=720

# 换场景配置
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# PC 预览（非零拷贝）
ros2 launch topsbot_usb_cam usb_cam.launch.py \
  params_file:=mjpeg2rgb_640x480_sensor.yaml setup_shm:=false

# Cyclone DDS 零拷贝（与其它终端一致：只 export RMW，RouDi 由 launch 自动启停）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch topsbot_usb_cam usb_cam.launch.py
```

## 6. 节点支持哪些 ROS 参数？

**启动后**（最权威）：

```bash
ros2 param list /usb_cam
ros2 param describe /usb_cam framerate
ros2 param dump /usb_cam
```

**启动前**（静态参考）：

```bash
cat $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/mjpeg2rgb_640x480_zc.yaml
cat $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/params_reference.yaml
```

## 7. 自定义配置

```bash
PKG=$(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam
cp $PKG/config/mjpeg2rgb_640x480_zc.yaml ~/my_cam.yaml
# 编辑 ~/my_cam.yaml 后：
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=/home/.../my_cam.yaml
```

## 8. 参数从哪里来？

后者覆盖前者：

1. **params_file** — 全量 yaml（默认 `mjpeg2rgb_640x480_zc.yaml`）
2. **launch 硬件参数** — 设备、分辨率、帧率、`frame_id`
3. **launch 场景覆盖** — 非空的 `pixel_format`、`zero_copy` 等

## 9. 发布话题

| 条件 | 单相机 | 多相机（`usb_cam_multi`，N=0…3） |
|------|--------|-------------------------|
| `zero_copy:=true`，非 raw mjpeg | `/tbmem_img` | `/camN/tbmem_img` |
| `zero_copy:=true`，`pixel_format:=mjpeg` | `/tbmem_jpeg` | `/camN/tbmem_jpeg` |
| `zero_copy:=false`，非 mjpeg | `/image_raw` | `/camN/image_raw` |
| `zero_copy:=false`，`pixel_format:=mjpeg` | `/image_raw/compressed` | `/cam0/image_raw/compressed` 等 |

多相机用 ROS namespace 区分，namespace 内话题名与单相机相同。最多 `cam0`～`cam3` 四路；默认设备只启用前两路。完整文档见 `README_cn.md`。

## 10. 联调 topsbot_cv（图像处理）

`topsbot_cv` 配置分两类 API 模板，每种 `input_msg` 各 2 份 yaml（不再为 crop/resize/csc 各建一份）：

| 模板后缀 | ta-cv API | 用途 |
|----------|-----------|------|
| `*_image_csc_640x480.yaml` | `ta_cv_image_csc_convert_to` | crop / resize / CSC，用 `pipeline` 组合 |
| `*_convert_padding_640x480.yaml` | `ta_cv_image_convert_padding` | YOLO letterbox 等填充 |

### 成对启动（640×480）

```bash
# 1) 相机 + crop（最常见）
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg_640x480_zc.yaml
ros2 launch topsbot_cv image_processor.launch.py params_file:=tb_jpeg_image_csc_640x480.yaml

# 2) NV12 零拷贝
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml
ros2 launch topsbot_cv image_processor.launch.py params_file:=tb_img_image_csc_640x480.yaml

# 3) PC 预览（sensor_msgs）
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_sensor.yaml setup_shm:=false
ros2 launch topsbot_cv image_processor.launch.py params_file:=sensor_image_image_csc_640x480.yaml

# 4) CompressedImage
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg_640x480_compressed.yaml
ros2 launch topsbot_cv image_processor.launch.py params_file:=sensor_compressed_image_csc_640x480.yaml
```

| usb_cam `params_file` | cv `params_file`（image_csc） | cv `params_file`（letterbox） |
|-----------------------|-------------------------------|-------------------------------|
| `mjpeg_640x480_zc.yaml` | `tb_jpeg_image_csc_640x480.yaml` | `tb_jpeg_convert_padding_640x480.yaml` |
| `mjpeg2nv12_640x480_zc.yaml` | `tb_img_image_csc_640x480.yaml` | `tb_img_convert_padding_640x480.yaml` |
| `mjpeg2nv12_640x480_sensor.yaml` | `sensor_image_image_csc_640x480.yaml` | `sensor_image_convert_padding_640x480.yaml` |
| `mjpeg_640x480_compressed.yaml` | `sensor_compressed_image_csc_640x480.yaml` | `sensor_compressed_convert_padding_640x480.yaml` |

### 不必换 yaml 的常见覆盖

```bash
# crop → crop+resize+csc（同一 image_csc 模板）
ros2 launch topsbot_cv image_processor.launch.py \
  params_file:=tb_jpeg_image_csc_640x480.yaml pipeline:=crop,resize,csc

# 多相机 namespace
ros2 launch topsbot_cv image_processor.launch.py \
  params_file:=tb_img_image_csc_640x480.yaml input_topic:=/cam0/tbmem_img

# letterbox
ros2 launch topsbot_cv image_processor.launch.py params_file:=tb_jpeg_convert_padding_640x480.yaml
```

更多参数见 `topsbot_cv` 的 `config/README.md` 与 `params_reference.yaml`。
