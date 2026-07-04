# config/ 目录说明

## 文件命名规范

```
{pixel_format}_{宽}x{高}_{模式}.yaml
```

**`pixel_format` 与 yaml 内 `ros__parameters.pixel_format` 字段一致**（不是简称）。

| 字段 | 含义 | 取值示例 |
|------|------|----------|
| pixel_format | V4L2 采集后的解码/输出格式 | `mjpeg2rgb`、`mjpeg2nv12`、`mjpeg`（原始 MJPEG 码流） |
| 分辨率 | `image_width` × `image_height` | `640x480` |
| 模式 | 发布路径 | `zc`（tb_img 零拷贝）、`sensor`（`sensor_msgs/Image`）、`compressed`（`sensor_msgs/CompressedImage`） |

示例：`mjpeg2nv12_640x480_sensor.yaml` = `pixel_format: mjpeg2nv12`、640×480、`sensor_msgs` 非零拷贝。

## 内置配置

```bash
ls $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/*_*x*_*.yaml
```

| 文件 | pixel_format | 模式 | 实际发布 |
|------|--------------|------|----------|
| `mjpeg2rgb_640x480_zc.yaml` | mjpeg2rgb | 零拷贝（**launch 默认**） | `/tbmem_img` |
| `mjpeg2nv12_640x480_zc.yaml` | mjpeg2nv12 | 零拷贝 | `/tbmem_img` |
| `mjpeg_640x480_zc.yaml` | mjpeg | MJPEG 码流零拷贝 | `/tbmem_jpeg` |
| `mjpeg2rgb_640x480_sensor.yaml` | mjpeg2rgb | sensor_msgs | `/image_raw` |
| `mjpeg2nv12_640x480_sensor.yaml` | mjpeg2nv12 | sensor_msgs | `/image_raw` |
| `mjpeg_640x480_compressed.yaml` | mjpeg | 压缩流 | `/image_raw/compressed` |

话题名与 TbMsg 类型由节点根据 `pixel_format` + `zero_copy` 自动选择，**内置 yaml 无需填写** `tbmem_topic` / `tb_jpeg_topic` / `tb_img_profile`。

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

cp $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/mjpeg2rgb_640x480_zc.yaml ~/my_cam.yaml
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=/home/.../my_cam.yaml
```

## 参数合并顺序（后者覆盖前者）

1. `params_file` — 上表中的全量 yaml
2. launch 硬件参数 — `video_device`、`image_width`、`image_height`、`framerate`、`frame_id`
3. launch 场景覆盖 — 非空的 `pixel_format`、`zero_copy`

## 其他文件

| 文件 | 说明 |
|------|------|
| `params_reference.yaml` | 参数说明速查（**不自动加载**） |
| `camera_info.yaml` | 默认标定（可选） |
| `shm_fastdds.xml` | Fast DDS 共享内存配置 |

双相机：`dual_camera.launch.py` 启动 `cam0` / `cam1` 两个 namespace，每路节点名为 `usb_cam`；namespace 内话题与单相机同名（如 `/cam0/tbmem_img`）。用 `cam0_video_device` / `cam1_video_device` 等 launch 参数区分硬件。

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py -s
ros2 param describe /usb_cam pixel_format
```
