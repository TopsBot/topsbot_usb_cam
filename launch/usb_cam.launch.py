# Copyright (c) 2026 TOPSBOT contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Parameter merge order (later wins):
#   1. params_file — full scene yaml under config/ (default: mjpeg2rgb_640x480_zc.yaml)
#   2. launch hardware args — device, resolution, framerate, frame_id
#   3. launch scene overrides — non-empty pixel_format / zero_copy
#
# List built-in configs: ls $(ros2 pkg prefix topsbot_usb_cam)/share/topsbot_usb_cam/config/*_*x*_*.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_bool(value: str):
    if not value:
        return None
    return value.lower() in ('true', '1', 'yes')


def _resolve_params_path(pkg_share: str, params_file: str) -> str:
    if not params_file:
        raise RuntimeError('usb_cam.launch: params_file is empty')
    if os.path.isabs(params_file):
        path = params_file
    else:
        path = os.path.join(pkg_share, 'config', params_file)
    if not os.path.isfile(path):
        raise RuntimeError(f'usb_cam.launch: params_file not found: {path}')
    return path


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    params_file = LaunchConfiguration('params_file').perform(context)
    params_path = _resolve_params_path(pkg_share, params_file)

    parameters = [params_path]

    parameters.append({
        'video_device': LaunchConfiguration('video_device').perform(context),
        'image_width': int(LaunchConfiguration('image_width').perform(context)),
        'image_height': int(LaunchConfiguration('image_height').perform(context)),
        'framerate': int(LaunchConfiguration('framerate').perform(context)),
        'frame_id': LaunchConfiguration('frame_id').perform(context),
    })

    scene_overrides = {}
    pixel_format = LaunchConfiguration('pixel_format').perform(context)
    if pixel_format:
        scene_overrides['pixel_format'] = pixel_format

    zero_copy = _optional_bool(LaunchConfiguration('zero_copy').perform(context))
    if zero_copy is not None:
        scene_overrides['zero_copy'] = zero_copy

    if scene_overrides:
        parameters.append(scene_overrides)

    namespace = LaunchConfiguration('namespace').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)

    return [
        Node(
            package='topsbot_usb_cam',
            executable='topsbot_usb_cam',
            name=node_name,
            namespace=namespace,
            output='screen',
            parameters=parameters,
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare('topsbot_usb_cam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='mjpeg2rgb_640x480_zc.yaml',
            description=(
                'ROS parameter yaml: {pixel_format}_{WxH}_{mode}.yaml under config/ '
                '(pixel_format matches yaml field), or absolute path. '
                'List: ls share/topsbot_usb_cam/config/*_*x*_*.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='V4L2 device path, e.g. /dev/video0',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Capture width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Capture height in pixels',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='15',
            description='Capture frame rate (Hz)',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='usb_cam',
            description='TF / message header frame_id',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='',
            description=(
                'Override params_file output format when non-empty. '
                'e.g. mjpeg2rgb, mjpeg2nv12, mjpeg'
            ),
        ),
        DeclareLaunchArgument(
            'zero_copy',
            default_value='',
            description='Override zero_copy when non-empty (true/false)',
        ),
        DeclareLaunchArgument(
            'setup_shm',
            default_value='true',
            description=(
                'Include zero-copy DDS env (zc_env): auto-detects RMW_IMPLEMENTATION, '
                'starts/stops iox-roudi when using Cyclone DDS'
            ),
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='usb_cam',
            description='ROS node name',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional ROS namespace',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'zc_env.launch.py']),
            ]),
            condition=IfCondition(LaunchConfiguration('setup_shm')),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
