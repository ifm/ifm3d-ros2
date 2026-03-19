#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2025 ifm electronic, gmbh
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
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
            description="RCLCPP log level for the ODS node. ['debug', 'info', 'warn', 'error']",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pds_name",
            default_value="pds",
            description="Name for the PDS node and used as tf prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pds_vis_name",
            default_value="pds_vis",
            description="Name for the visualization node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pds_namespace",
            default_value="ifm3d",
            description="Namespace to launch nodes into",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_package",
            default_value="ifm3d_ros2",
            description="Package containing the PDS YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_directory",
            default_value="config",
            description="Directory inside parameter_file_package containing the PDS YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_name",
            default_value="pds_default_parameters.yaml",
            description="YAML file with the PDS configuration.",
        )
    )

    # ------------------------------------------------------------
    # Nodes, using substitutions to fill in arguments
    # ------------------------------------------------------------
    pds_node = LifecycleNode(
        package="ifm3d_ros2",
        executable="pds_standalone",
        namespace=LaunchConfiguration("pds_namespace"),
        name=LaunchConfiguration("pds_name"),
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
    pds_vis_node = LifecycleNode(
        package="ifm3d_ros2",
        executable="pds_vis_standalone",
        namespace=LaunchConfiguration("pds_namespace"),
        name=LaunchConfiguration("pds_vis_name"),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        log_cmd=True,
        remappings=[
            (
                "~/pds_full_result",
                [
                    "/",
                    LaunchConfiguration("pds_namespace"),
                    "/",
                    LaunchConfiguration("pds_name"),
                    "/pds_full_result",
                ],
            ),
        ],
    )

    # ------------------------------------------------------------
    # Lifecycle management
    # ------------------------------------------------------------

    # UNCONFIGURED to INACTIVE via ChangeState (emitted without delay)
    configure_pds = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(pds_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    configure_pds_vis = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(pds_vis_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # INACTIVE to ACTIVE via Handler
    # Handler emits event after the transition from configuring to inactive
    # Emitted ChangeState event transitions the node from inactive to active
    activate_pds = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=pds_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(pds_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    activate_pds_vis = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=pds_vis_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(pds_vis_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    ld = LaunchDescription()
    for declared_argument in declared_arguments:
        ld.add_entity(declared_argument)
    ld.add_action(pds_node)
    ld.add_action(configure_pds)
    ld.add_action(activate_pds)
    ld.add_action(pds_vis_node)
    ld.add_action(configure_pds_vis)
    ld.add_action(activate_pds_vis)

    return ld
