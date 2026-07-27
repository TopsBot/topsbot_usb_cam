# Copyright (c) 2026 TOPSBOT contributors.
#
# Zero-copy DDS environment for loaned tb_img_msgs / TbJpegFrame publishing.
#
# Detects the current RMW from the process environment (do NOT pass dds_backend):
#   unset / rmw_fastrtps_cpp → Fast DDS SHM (shm_fastdds.xml)
#   rmw_cyclonedds_cpp       → Cyclone DDS SHM (shm_cyclonedds.xml) + iox-roudi
#
# Switch DDS the same way as any other terminal:
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   ros2 launch topsbot_usb_cam usb_cam.launch.py
#
# For Cyclone: if iox-roudi is not already running, start it as a launch child so
# Ctrl+C / launch shutdown stops it. If one is already running, reuse it.

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable


def _rmw_name() -> str:
    return os.environ.get('RMW_IMPLEMENTATION', '').strip().lower()


def _is_cyclonedds(rmw: str) -> bool:
    return 'cyclonedds' in rmw


def _iox_roudi_running() -> bool:
    try:
        return subprocess.run(
            ['pgrep', '-x', 'iox-roudi'],
            capture_output=True,
            check=False,
        ).returncode == 0
    except OSError:
        return False


def _setup_env(context, *args, **kwargs):
    rmw = _rmw_name()
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    actions = []

    if _is_cyclonedds(rmw):
        config_path = os.path.join(pkg_share, 'config', 'shm_cyclonedds.xml')
        actions.append(
            SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{config_path}'))
        actions.append(
            LogInfo(msg=f'zc_env: RMW={rmw or "(unset)"} → Cyclone DDS SHM '
                        f'({config_path})'))

        if not shutil.which('iox-roudi'):
            raise RuntimeError(
                'zc_env: RMW_IMPLEMENTATION is cyclonedds but iox-roudi is not '
                'in PATH (install iceoryx / ros-humble-iceoryx-posh)')

        if _iox_roudi_running():
            actions.append(
                LogInfo(msg='zc_env: iox-roudi already running, reusing '
                            '(will not stop it on launch exit)'))
        else:
            actions.append(
                LogInfo(msg='zc_env: starting iox-roudi (stops with this launch)'))
            actions.append(
                ExecuteProcess(
                    cmd=['iox-roudi'],
                    name='iox-roudi',
                    output='screen',
                ))
        return actions

    # Default / Fast DDS (Humble default when RMW_IMPLEMENTATION is unset)
    if rmw and 'fastrtps' not in rmw and 'fastdds' not in rmw:
        actions.append(
            LogInfo(msg=f'zc_env: unrecognized RMW_IMPLEMENTATION={rmw!r}, '
                        'applying Fast DDS SHM profile'))

    config_path = os.path.join(pkg_share, 'config', 'shm_fastdds.xml')
    actions.extend([
        LogInfo(msg=f'zc_env: RMW={rmw or "rmw_fastrtps_cpp (default)"} → '
                    f'Fast DDS SHM ({config_path})'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', config_path),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup_env),
    ])
