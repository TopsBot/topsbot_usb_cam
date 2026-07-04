# Copyright (c) 2026 TOPSBOT contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Two usb_cam nodes under namespaces cam0 / cam1 (ROS multi-camera convention).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_params_path(pkg_share: str, params_file: str) -> str:
    if os.path.isabs(params_file):
        path = params_file
    else:
        path = os.path.join(pkg_share, 'config', params_file)
    if not os.path.isfile(path):
        raise RuntimeError(f'dual_camera.launch: params_file not found: {path}')
    return path


def _camera_node(context, namespace, device_arg, frame_id_arg, camera_name_arg):
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    params_path = _resolve_params_path(
        pkg_share, LaunchConfiguration('params_file').perform(context))

    return Node(
        package='topsbot_usb_cam',
        executable='topsbot_usb_cam',
        name='usb_cam',
        namespace=namespace,
        output='screen',
        parameters=[
            params_path,
            {
                'video_device': LaunchConfiguration(device_arg).perform(context),
                'frame_id': LaunchConfiguration(frame_id_arg).perform(context),
                'camera_name': LaunchConfiguration(camera_name_arg).perform(context),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )


def _launch_setup(context, *args, **kwargs):
    return [
        _camera_node(context, 'cam0', 'cam0_video_device', 'cam0_frame_id', 'cam0_camera_name'),
        _camera_node(context, 'cam1', 'cam1_video_device', 'cam1_frame_id', 'cam1_camera_name'),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare('topsbot_usb_cam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='mjpeg2rgb_640x480_zc.yaml',
            description='Shared params yaml for both cameras',
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
            'setup_shm',
            default_value='true',
            description='Enable Fast DDS shared-memory env (required for zero-copy)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'shm_env.launch.py']),
            ]),
            condition=IfCondition(LaunchConfiguration('setup_shm')),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
