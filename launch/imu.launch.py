#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2024 ifm electronic, gmbh
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    # ------------------------------------------------------------
    # Launch script arguments
    # ------------------------------------------------------------
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="RCLCPP log level for the IMU node. ['debug', 'info', 'warn', 'error']",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_name",
            default_value="imu",
            description="Name for the imu node and used as tf prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_namespace",
            default_value="ifm3d",
            description="Namespace to launch nodes into",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_package",
            default_value="ifm3d_ros2",
            description="Package containing the IMU YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_directory",
            default_value="config",
            description="Directory inside parameter_file_package containing the IMU YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_name",
            default_value="imu_default_parameters.yaml",
            description="YAML file with the imu configuration.",
        )
    )

    # ------------------------------------------------------------
    # Nodes, using substitutions to fill in arguments
    # ------------------------------------------------------------
    imu_node = LifecycleNode(
        package="ifm3d_ros2",
        executable="imu_standalone",
        namespace=LaunchConfiguration("imu_namespace"),
        name=LaunchConfiguration("imu_name"),
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("parameter_file_package")),
                    LaunchConfiguration("parameter_file_directory"),
                    LaunchConfiguration("parameter_file_name"),
                ]
            )
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        log_cmd=True,
    )

    # ------------------------------------------------------------
    # Lifecycle management
    # ------------------------------------------------------------

    # UNCONFIGURED to INACTIVE via ChangeState (emitted without delay)
    configure_imu = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(imu_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # INACTIVE to ACTIVE via Handler
    # Handler emits event after the transition from configuring to inactive
    # Emitted ChangeState event transitions the node from inactive to active
    activate_imu = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    ld = LaunchDescription()
    for declared_argument in declared_arguments:
        ld.add_entity(declared_argument)
    ld.add_action(imu_node)
    ld.add_action(configure_imu)
    ld.add_action(activate_imu)

    return ld
