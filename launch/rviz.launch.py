# 
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
# 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'ifm3d_ros2'

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'etc', 'ifm3d.rviz'
        )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config],
            output='screen',
            log_cmd=True
            ),
        ])
