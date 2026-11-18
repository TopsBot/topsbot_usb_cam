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
#   2. launch overrides — only arguments the user actually passed.
#      Empty means keep the yaml value. This includes device, resolution,
#      framerate, frame_id, camera_name, pixel_format, and zero_copy.
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


def _optional_int(value: str):
    if not value:
        return None
    return int(value)


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

    # Only keys the user passed on the command line. Empty keeps the yaml value,
    # so a 1280x720 params file is not silently replaced by the old 640x480 default.
    overrides = {}
    video_device = LaunchConfiguration('video_device').perform(context)
    if video_device:
        overrides['video_device'] = video_device
    image_width = _optional_int(LaunchConfiguration('image_width').perform(context))
    if image_width is not None:
        overrides['image_width'] = image_width
    image_height = _optional_int(LaunchConfiguration('image_height').perform(context))
    if image_height is not None:
        overrides['image_height'] = image_height
    framerate = _optional_int(LaunchConfiguration('framerate').perform(context))
    if framerate is not None:
        overrides['framerate'] = framerate
    frame_id = LaunchConfiguration('frame_id').perform(context)
    if frame_id:
        overrides['frame_id'] = frame_id
    camera_name = LaunchConfiguration('camera_name').perform(context)
    if camera_name:
        overrides['camera_name'] = camera_name
    pixel_format = LaunchConfiguration('pixel_format').perform(context)
    if pixel_format:
        overrides['pixel_format'] = pixel_format
    zero_copy = _optional_bool(LaunchConfiguration('zero_copy').perform(context))
    if zero_copy is not None:
        overrides['zero_copy'] = zero_copy
    if overrides:
        parameters.append(overrides)

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
            default_value='',
            description='Override yaml video_device when non-empty, e.g. /dev/video0',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='',
            description='Override yaml image_width when non-empty',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='',
            description='Override yaml image_height when non-empty',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='',
            description='Override yaml framerate when non-empty (Hz)',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='Override yaml frame_id when non-empty',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='',
            description='Override yaml camera_name when non-empty',
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
