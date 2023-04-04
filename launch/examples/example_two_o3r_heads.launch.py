#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

"""
This example script connects four CameraNodes to the 2d and 3d streams of two O3R.

The nodes are named left_camera_2d, left_camera_3d, right_camera_2d and
right_camera_3d. The differentiation in left and right is arbitrary and only acts
as illustration for a fictitious use case.

The Ports on the VPU should be connected as follows:
* Left Camera 2D: Port 0
* Left Camera 3D: Port 2
* Right Camera 2D: Port 1
* Right Camera 3D: Port 3

The configuration for all four nodes is read from a single file: two_o3r_heads.yaml
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
    #   The configuration for all four nodes is located in a singular parameter file.
    # log_level: pass declared log_level to the included launch scripts

    launch_script_left_2d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'left_camera_2d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'two_o3r_heads.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    launch_script_left_3d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'left_camera_3d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'two_o3r_heads.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    launch_script_right_2d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'right_camera_2d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'two_o3r_heads.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    launch_script_right_3d = IncludeLaunchDescription(
        [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
        launch_arguments={
            'camera_name': 'right_camera_3d',
            'parameter_file_package': 'ifm3d_ros2',
            'parameter_file_directory': 'config/examples',
            'parameter_file_name': 'two_o3r_heads.yaml',
            'log_level': LaunchConfiguration("log_level"),
        }.items(),
    )

    # Adding the arguments and included launch descriptions to one LaunchDescription
    ld = LaunchDescription()
    ld.add_entity(declared_argument)
    ld.add_action(launch_script_left_2d)
    ld.add_action(launch_script_left_3d)
    ld.add_action(launch_script_right_2d)
    ld.add_action(launch_script_right_3d)
    return ld
