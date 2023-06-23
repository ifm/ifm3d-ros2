#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

"""
This example script connects two CameraNodes to the 2d and 3d stream of one O3R.

The nodes are named camera_2d and camera_3d.

The Ports on the VPU should be connected as follows:
* Camera 2D: Port 0
* Camera 3D: Port 2

The configuration for the nodes is read from a two different files:
o3r_2d.yaml and o3r_3d.yaml
"""


def generate_launch_description():

    # Declare a log_level argument to showcase passing arguments to
    # included launch files further down in this example
    declared_argument = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="To change RCLCPP log level for the camera nodes. ['debug', 'info', 'warn', 'error']",
    )

    # Include the generic camera.launch.py but overwrite some launch arguments
    #
    # camera_name: Set destinct node name, as there will be multiple camera nodes
    # parameter_file_{package|directory|name}: using these arguments to set the
    #   YAML parameter file from which the node's config is read.
    #   Using two different parameter files for the different nodes.
    # log_level: pass declared log_level to the included launch scripts

    launch_script_2d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'camera_2d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'o3r_2d.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    launch_script_3d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'camera_3d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'o3r_3d.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    # Adding the arguments and included launch descriptions to one LaunchDescription
    ld = LaunchDescription()
    ld.add_entity(declared_argument)
    ld.add_action(launch_script_2d)
    ld.add_action(launch_script_3d)
    return ld
