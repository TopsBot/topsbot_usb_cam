# Changelog

## 0.3.0 (2026-07-24)

### Changed

- `usb_cam_multi.launch.py`: when `channel_count` is unset, default to **2** channels (no longer inferred from `cam2`/`cam3` device paths); up to **4** with `channel_count:=4`.
- Docs: default 2-channel multi-camera usage and four-channel override.

## 0.2.1 (2026-06-29)

### Changed

- Config: remove `config/presets/` and `config/examples/`; use full `config/*_*x*_*.yaml` files only.
- Launch: drop `preset` argument; `params_file` defaults to `mjpeg2rgb_640x480_zc.yaml`.
- `usb_cam_multi.launch.py`: up to four cameras (`cam0`…`cam3`); default **2** channels; `channel_count:=4` for four cameras.
- Built-in yaml: only `pixel_format` + `zero_copy` for publish path; topic names derived in node (not user-configurable).
- Config UX: remove auto-loaded `usb_cam_common.yaml`; hardware defaults live in C++ `declare_parameter`.
- Launch: explicit hardware defaults (hobot-style), optional `params_file`, documented merge order; all launch args have descriptions.
- Config file naming: `{pixel_format}_{width}x{height}_{mode}.yaml` (e.g. `mjpeg2nv12_640x480_zc.yaml`).
- Docs: install `QUICKSTART.md`, `config/README.md`, `config/params_reference.yaml`.
- Node: `ros2 param describe` support via `ParameterDescriptor` on all parameters.
- Non-zero-copy: capture and publish in one timer callback (fixes effective FPS being ~half of `framerate`).

### Removed

- `config/presets/` and `config/examples/` (replaced by `config/*_*x*_*.yaml`).
- Launch argument `preset`.
- `config/usb_cam_common.yaml` (replaced by `params_reference.yaml` + C++ defaults).
- Launch/user parameters `tbmem_topic`, `tb_jpeg_topic`, `tb_img_profile`, `camera_index`.


## 0.2.0 (2026-06-29)

### Added

- MJPEG zero-copy publishing via `TbJpegFrame` (`presets/mjpeg_zerocopy.yaml`).

## Unreleased

### Changed

- Restructure `config/` into `usb_cam_common.yaml`, `presets/`, and `devices/`.
- Replace launch files with `usb_cam.launch.py`, `usb_cam_multi.launch.py`, and internal `shm_env.launch.py`.
- Default preset is `presets/mjpeg2rgb_zerocopy.yaml`.
- Default to Release build with `-O3` when `CMAKE_BUILD_TYPE` is unset.
- Rewrite README for community users (apt install, resolution/format guide).

### Removed

- Legacy launch files and flat `params*.yaml` configs.
- Unused `config/examples/experimental/` presets (use `presets/` or launch overrides).

## 0.1.0 (2026-06-09)

- Initial release based on [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) v0.8.1.
- Add `tb_img_msgs` zero-copy publishing via `borrow_loaned_message()`.
- Support `TbMsg480P` / `TbMsg540P` / `TbMsg1080P` auto profile selection.
- Keep standard `sensor_msgs` + `image_transport` path when `zero_copy:=false`.
- Apache-2.0 distribution with BSD upstream attribution (`LICENSE.BSD`, `NOTICE`).
