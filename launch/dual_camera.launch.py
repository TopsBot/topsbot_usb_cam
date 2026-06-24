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


def _camera_node(context, namespace, node_name, device_yaml):
    pkg_share = get_package_share_directory('topsbot_usb_cam')
    preset = LaunchConfiguration('preset').perform(context)

    return Node(
        package='topsbot_usb_cam',
        executable='topsbot_usb_cam',
        name=node_name,
        namespace=namespace,
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'usb_cam_common.yaml'),
            os.path.join(pkg_share, 'config', preset),
            os.path.join(pkg_share, 'config', 'devices', device_yaml),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )


def _launch_setup(context, *args, **kwargs):
    return [
        _camera_node(context, 'cam0', 'usb_cam', 'camera_video0.yaml'),
        _camera_node(context, 'cam1', 'usb_cam', 'camera_video2.yaml'),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare('topsbot_usb_cam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'preset',
            default_value='presets/mjpeg2rgb_sensor.yaml',
            description='Shared preset for both cameras (sensor_msgs path recommended)',
        ),
        DeclareLaunchArgument(
            'setup_shm',
            default_value='false',
            description='Enable Fast DDS shared-memory env (not needed for sensor mode)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'shm_env.launch.py']),
            ]),
            condition=IfCondition(LaunchConfiguration('setup_shm')),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
