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
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('topsbot_usb_cam'),
        'config',
        'shm_fastdds.xml',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', config_path),
    ])
