#!/usr/bin/env python3

# 
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
# 

import yaml
import logging

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

ifm3d_ros2_dir = get_package_share_directory('ifm3d_ros2')

logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)

def add_camera_loop(context, *args, **kwargs):
    
    cameras=[]
    params_path = LaunchConfiguration('params').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    logging.debug(vars(context))

    with open(params_path) as p:
        params_data = yaml.load(p)

    try:
        print(params_data.keys())
        for key, value in params_data.items():
            logging.info('Camera port arguments: {}'.format(key.split(sep='/')[-1]))

            camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ifm3d_ros2_dir + '/launch/camera_managed.launch.py'),
                launch_arguments={"name": key.split(sep='{}/'.format(namespace))[-1],
                                "params": params_path,
                                "namespace": namespace,
                                }.items(),
            )
            logging.info('Launch argument: {}'.format(vars(camera)))

            # add camera to cameras list as parsed from namespace arg and yaml fields
            cameras.append(camera)
    except Exception as e:
        print('parsing the configuration yaml found an error: {}. Please check your namespace definitions.'.format(e))
    return cameras


def generate_launch_description():
    """Launch the camera_managed.launch.py launch file."""
    ld = LaunchDescription()
    DeclareLaunchArgument(
        'namespace', 
        default_value='ifm3d',
    )
    DeclareLaunchArgument(
        'params',
        default_value = ifm3d_ros2_dir + '/config/params.yaml',
        description='test arg that overlaps arg in included file',
    )
    ld.add_action(OpaqueFunction(function = add_camera_loop))
    return ld