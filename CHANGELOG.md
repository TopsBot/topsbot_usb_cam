# Changelog

## Unreleased

### Changed

- Restructure `config/` into `usb_cam_common.yaml`, `presets/`, and `devices/`.
- Replace launch files with `usb_cam.launch.py`, `dual_camera.launch.py`, and internal `shm_env.launch.py`.
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
