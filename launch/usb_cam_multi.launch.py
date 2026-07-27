# Copyright (c) 2026 TOPSBOT contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Up to four usb_cam nodes under namespaces cam0 … cam3.
#
# channel_count:
#   Empty (default) — start 2 cameras (cam0/cam1).
#   N — explicitly start cam0 … cam(N-1), where 1 <= N <= max_channel_count.
#   Four cameras: channel_count:=4 cam2_video_device:=... cam3_video_device:=...

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MAX_CHANNELS = 4
DEFAULT_CHANNEL_COUNT = 2


def _resolve_params_path(pkg_share: str, params_file: str) -> str:
    if os.path.isabs(params_file):
        path = params_file
    else:
        path = os.path.join(pkg_share, 'config', params_file)
    if not os.path.isfile(path):
        raise RuntimeError(f'usb_cam_multi.launch: params_file not found: {path}')
    return path


def _optional_int(value: str):
    return int(value) if value else None


def _channel_count(context) -> int:
    requested = _optional_int(LaunchConfiguration('channel_count').perform(context))
    max_channels = int(LaunchConfiguration('max_channel_count').perform(context))
    if max_channels < 1 or max_channels > MAX_CHANNELS:
        raise RuntimeError(
            f'usb_cam_multi.launch: max_channel_count must be 1 … {MAX_CHANNELS}, '
            f'got {max_channels}')

    if requested is None:
        selected = DEFAULT_CHANNEL_COUNT
        if selected > max_channels:
            selected = max_channels
        return selected

    if requested < 1 or requested > max_channels:
        raise RuntimeError(
            f'usb_cam_multi.launch: channel_count must be 1 … {max_channels}, '
            f'got {requested}')
    return requested


def _camera_node(context, index: int):
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    params_path = _resolve_params_path(
        pkg_share, LaunchConfiguration('params_file').perform(context))
    device = LaunchConfiguration(f'cam{index}_video_device').perform(context)
    if not device:
        raise RuntimeError(
            f'usb_cam_multi.launch: cam{index}_video_device is required '
            f'when channel_count enables cam{index}')

    return Node(
        package='topsbot_usb_cam',
        executable='topsbot_usb_cam',
        name='usb_cam',
        namespace=f'cam{index}',
        output='screen',
        parameters=[
            params_path,
            {
                'video_device': device,
                'frame_id': LaunchConfiguration(f'cam{index}_frame_id').perform(context),
                'camera_name': LaunchConfiguration(f'cam{index}_camera_name').perform(context),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )


def _launch_setup(context, *args, **kwargs):
    return [_camera_node(context, index) for index in range(_channel_count(context))]


def generate_launch_description():
    pkg_share = FindPackageShare('topsbot_usb_cam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='mjpeg2rgb_640x480_zc.yaml',
            description='Shared params yaml for all cameras',
        ),
        DeclareLaunchArgument(
            'max_channel_count',
            default_value=str(MAX_CHANNELS),
            description=f'Maximum camera slots to consider (1 … {MAX_CHANNELS})',
        ),
        DeclareLaunchArgument(
            'channel_count',
            default_value='',
            description=(
                f'Cameras to start (1 … max_channel_count). Empty defaults to '
                f'{DEFAULT_CHANNEL_COUNT}. Use channel_count:=4 for four cameras.'
            ),
        ),
        DeclareLaunchArgument(
            'cam0_video_device',
            default_value='/dev/video0',
            description='V4L2 device for cam0 namespace',
        ),
        DeclareLaunchArgument(
            'cam0_frame_id',
            default_value='cam0',
            description='frame_id for cam0',
        ),
        DeclareLaunchArgument(
            'cam0_camera_name',
            default_value='cam0',
            description='camera_name for cam0',
        ),
        DeclareLaunchArgument(
            'cam1_video_device',
            default_value='/dev/video2',
            description='V4L2 device for cam1 namespace',
        ),
        DeclareLaunchArgument(
            'cam1_frame_id',
            default_value='cam1',
            description='frame_id for cam1',
        ),
        DeclareLaunchArgument(
            'cam1_camera_name',
            default_value='cam1',
            description='camera_name for cam1',
        ),
        DeclareLaunchArgument(
            'cam2_video_device',
            default_value='',
            description='Optional V4L2 device for cam2 namespace',
        ),
        DeclareLaunchArgument(
            'cam2_frame_id',
            default_value='cam2',
            description='frame_id for cam2',
        ),
        DeclareLaunchArgument(
            'cam2_camera_name',
            default_value='cam2',
            description='camera_name for cam2',
        ),
        DeclareLaunchArgument(
            'cam3_video_device',
            default_value='',
            description='Optional V4L2 device for cam3 namespace',
        ),
        DeclareLaunchArgument(
            'cam3_frame_id',
            default_value='cam3',
            description='frame_id for cam3',
        ),
        DeclareLaunchArgument(
            'cam3_camera_name',
            default_value='cam3',
            description='camera_name for cam3',
        ),
        DeclareLaunchArgument(
            'setup_shm',
            default_value='true',
            description=(
                'Include zero-copy DDS env (zc_env): auto-detects RMW_IMPLEMENTATION, '
                'starts/stops iox-roudi when using Cyclone DDS'
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'zc_env.launch.py']),
            ]),
            condition=IfCondition(LaunchConfiguration('setup_shm')),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
