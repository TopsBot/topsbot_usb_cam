# Copyright (c) 2026 TOPSBOT contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_int(value: str):
    if not value:
        return None
    return int(float(value))


def _optional_bool(value: str):
    if not value:
        return None
    return value.lower() in ('true', '1', 'yes')


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    preset = LaunchConfiguration('preset').perform(context)

    parameters = [
        os.path.join(pkg_share, 'config', 'usb_cam_common.yaml'),
        os.path.join(pkg_share, 'config', preset),
    ]

    overrides = {}
    string_overrides = {
        'video_device': LaunchConfiguration('video_device').perform(context),
        'frame_id': LaunchConfiguration('frame_id').perform(context),
        'pixel_format': LaunchConfiguration('pixel_format').perform(context),
        'tbmem_topic': LaunchConfiguration('tbmem_topic').perform(context),
        'tb_img_profile': LaunchConfiguration('tb_img_profile').perform(context),
    }
    for key, value in string_overrides.items():
        if value:
            overrides[key] = value

    width = _optional_int(LaunchConfiguration('image_width').perform(context))
    if width is not None:
        overrides['image_width'] = width

    height = _optional_int(LaunchConfiguration('image_height').perform(context))
    if height is not None:
        overrides['image_height'] = height

    framerate = _optional_int(LaunchConfiguration('framerate').perform(context))
    if framerate is not None:
        overrides['framerate'] = framerate

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
            'preset',
            default_value='presets/mjpeg2rgb_zerocopy.yaml',
            description=(
                'Preset yaml under topsbot_usb_cam/config/, '
                'e.g. presets/mjpeg2rgb_zerocopy.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='',
            description='Override V4L2 device, e.g. /dev/video0',
        ),
        DeclareLaunchArgument('image_width', default_value=''),
        DeclareLaunchArgument('image_height', default_value=''),
        DeclareLaunchArgument('framerate', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('pixel_format', default_value=''),
        DeclareLaunchArgument('zero_copy', default_value=''),
        DeclareLaunchArgument('tbmem_topic', default_value=''),
        DeclareLaunchArgument('tb_img_profile', default_value=''),
        DeclareLaunchArgument('setup_shm', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='usb_cam'),
        DeclareLaunchArgument('namespace', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'shm_env.launch.py']),
            ]),
            condition=IfCondition(LaunchConfiguration('setup_shm')),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
