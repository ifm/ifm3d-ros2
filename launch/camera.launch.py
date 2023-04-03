#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
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
            description="To change RCLCPP log level for the camera node. ['debug', 'info', 'warn', 'error']",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "visualization",
            default_value="false",
            description="If true, RViz2 with a predefined config is opened.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_name",
            default_value="camera",
            description="Name for the camera node and used as tf prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="ifm3d",
            description="Namespace to launch nodes into",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_package",
            default_value="ifm3d_ros2",
            description="Package containing the camera's YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_directory",
            default_value="config",
            description="Directory inside of parameter_file_package containing the camera's YAML configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameter_file_name",
            default_value="camera_default_parameters.yaml",
            description="YAML file with the camera configuration.",
        )
    )

    # ------------------------------------------------------------
    # Nodes, using substitutions to fill in arguments
    # ------------------------------------------------------------
    camera_node = LifecycleNode(
        package="ifm3d_ros2",
        executable="camera_standalone",
        namespace=LaunchConfiguration("camera_namespace"),
        name=LaunchConfiguration("camera_name"),
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

    tf_node = Node(
        executable="static_transform_publisher",
        package="tf2_ros",
        name=[LaunchConfiguration("camera_name"), "_internal_transform_publiser"],
        namespace=LaunchConfiguration("camera_namespace"),
        arguments=[
            '0',
            '0',
            '0',
            '0',
            '0',
            '0',
            [LaunchConfiguration("camera_name"), "_optical_link"],
            [LaunchConfiguration("camera_name"), "_link"],
        ],
        log_cmd=True,
    )

    # Launching RViz2 conditionally, depending on the "visualition" argument
    rviz_node = Node(
        executable="rviz2",
        package="rviz2",
        name=[LaunchConfiguration("camera_name"), "_rviz2"],
        arguments=[
            '-d',
            PathJoinSubstitution([FindPackageShare("ifm3d_ros2"), "etc", "ifm3d.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("visualization")),
        log_cmd=True,
    )

    # ------------------------------------------------------------
    # Lifecycle management
    # ------------------------------------------------------------

    # UNCONFIGURED to INACTIVE via ChangeState (emitted without delay)
    configure_camera = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(camera_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # INACTIVE to ACTIVE via Handler
    # Handler emits event after the transition from configuring to inactive
    # Emited ChangeState event transistions the node from inactive to active
    activate_camera = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=camera_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    ld = LaunchDescription()
    for declared_argument in declared_arguments:
        ld.add_entity(declared_argument)
    ld.add_action(camera_node)
    ld.add_action(tf_node)
    ld.add_action(rviz_node)
    ld.add_action(configure_camera)
    ld.add_action(activate_camera)

    return ld
