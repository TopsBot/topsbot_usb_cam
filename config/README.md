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
| `shm_fastdds.xml` | Fast DDS 共享内存（默认，或 `RMW_IMPLEMENTATION` 未设 / fastrtps） |
| `shm_cyclonedds.xml` | Cyclone DDS 共享内存（`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`） |

## 零拷贝 DDS 后端

同一 yaml（如 `*_zc.yaml`）可配合当前 shell 的 `RMW_IMPLEMENTATION` 使用；**发布端与订阅端须使用相同 RMW**。`zc_env.launch.py`（`setup_shm:=true` 时自动 include）会读取该环境变量并注入对应 SHM 配置，**不会**改写 `RMW_IMPLEMENTATION`。

```bash
# Fast DDS（默认，无需 export）
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml

# Cyclone DDS：与其它终端一致，只 export RMW
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch topsbot_usb_cam usb_cam.launch.py params_file:=mjpeg2nv12_640x480_zc.yaml
```

**Cyclone DDS：** 启用 `SharedMemory` 后底层走 iceoryx。`zc_env` 会在本机尚无 RouDi 时**自动启动** `iox-roudi`，并在本 launch 退出时**自动停止**（若启动前已有 RouDi 在跑，则复用且不负责关闭）。`tb_img_msgs` 为定长类型，满足 loaned 真零拷贝条件。

多终端各自 launch 时，每个终端都需 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；RouDi 由第一个带 `setup_shm` 的 launch 拉起即可。

| launch 参数 | 说明 | 默认 |
|-------------|------|------|
| `setup_shm` | 是否注入零拷贝 DDS 环境（并按需启停 iox-roudi） | `true` |

底层 launch：`zc_env.launch.py`（`setup_shm:=true` 时由 `usb_cam` / `usb_cam_multi` / demo 自动 include）。

多相机：`usb_cam_multi.launch.py` 最多 `cam0`～`cam3` 四路；**默认启动 2 路**（cam0/cam1）。四路需显式 `channel_count:=4` 并配置 `cam2_video_device` / `cam3_video_device`。

```bash
ros2 launch topsbot_usb_cam usb_cam.launch.py -s
ros2 param describe /usb_cam pixel_format
```
