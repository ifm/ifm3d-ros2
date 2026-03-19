#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2024 ifm electronic, gmbh
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from launch.substitutions import PythonExpression
import os
import yaml


def generate_launch_description():
    # ============================================================
    # 1. Declare launch arguments (configurable from CLI)
    # ============================================================
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_uncompressed",
            default_value="",
            description="Optional override: true/false. Leave empty to use YAML's publish_uncompressed (if set). When effective true spawns republish node providing <ns>/<name>/rgb_uncompressed.",
        )
    )

    def build_entities(context):
        # ============================================================
        # 2. Build launch entities after resolving substitutions
        #    - This allows us to parse YAML and combine CLI args
        # ============================================================
        # Precedence for publish_uncompressed:
        #   1. Explicit launch argument value if user set (any value other than empty string)
        #   2. YAML value (if present under /<ns>/<name>.ros__parameters.publish_uncompressed)
        #   3. Default (false)

        # --- Resolve all launch arguments and parameter file path ---
        ns = LaunchConfiguration("camera_namespace").perform(context)
        cam_name = LaunchConfiguration("camera_name").perform(context)
        param_pkg = LaunchConfiguration("parameter_file_package").perform(context)
        param_dir = LaunchConfiguration("parameter_file_directory").perform(context)
        param_file = LaunchConfiguration("parameter_file_name").perform(context)
        pkg_share = FindPackageShare(param_pkg).perform(context)
        yaml_path = os.path.join(pkg_share, param_dir, param_file)
        yaml_publish_uncompressed = None
        has_jpeg = False

        # --- Parse YAML parameter file for additional config ---
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
                key = f"/{ns}/{cam_name}"
                if key in data:
                    params_section = data[key].get('ros__parameters', {})
                else:
                    params_section = {}
                    for v in data.values():
                        if isinstance(v, dict) and 'ros__parameters' in v:
                            params_section = v['ros__parameters']
                            break
                if 'publish_uncompressed' in params_section:
                    yaml_publish_uncompressed = bool(params_section['publish_uncompressed'])
                # Check if JPEG_IMAGE is in buffer_id_list (RGB camera check)
                if 'buffer_id_list' in params_section:
                    buffer_list = params_section['buffer_id_list']
                    has_jpeg = 'JPEG_IMAGE' in buffer_list
        except Exception as e:
            print(f"[ifm3d_ros2][launch] Warning: Could not read YAML '{yaml_path}': {e}")

        # --- Determine effective value for publish_uncompressed ---
        # Combines CLI arg and YAML value, with CLI taking precedence
        cli_value = LaunchConfiguration("publish_uncompressed").perform(context)
        true_set = {"1", "true", "True", "TRUE", "on", "yes", "y"}
        false_set = {"0", "false", "False", "FALSE", "off", "no", "n"}
        effective = False
        if cli_value != "":
            if cli_value in true_set:
                effective = True
            elif cli_value in false_set:
                effective = False
            else:
                print(f"[ifm3d_ros2][launch] Warning: Unrecognized value for publish_uncompressed '{cli_value}', defaulting to false")
        elif yaml_publish_uncompressed is not None:
            effective = yaml_publish_uncompressed
        # Only enable republish if JPEG_IMAGE is actually being published
        effective = effective and has_jpeg
        print(f"[ifm3d_ros2][launch] publish_uncompressed effective={effective} (cli='{cli_value}', yaml={yaml_publish_uncompressed}, has_jpeg={has_jpeg})")

        # --- Create camera node and supporting nodes ---
        camera_node = LifecycleNode(
            package="ifm3d_ros2",
            executable="camera_standalone",
            namespace=ns,
            name=cam_name,
            output='screen',
            parameters=[yaml_path],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            log_cmd=True,
        )

        # RViz node (optional, launches if visualization is true)
        rviz_node = Node(
            executable="rviz2",
            package="rviz2",
            name=f"{cam_name}_rviz2",
            arguments=[
                '-d',
                PathJoinSubstitution(
                    [FindPackageShare("ifm3d_ros2"), "config", "ifm3d.rviz"]
                ),
            ],
            condition=IfCondition(LaunchConfiguration("visualization")),
            log_cmd=True,
        )

        configure_camera = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(camera_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )

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

        entities = [camera_node, rviz_node, configure_camera, activate_camera]
        if effective:
            republish_node = Node(
                package="image_transport",
                executable="republish",
                name=f"{cam_name}_rgb_republish",
                arguments=["compressed", "raw"],
                remappings=[
                    ("/in/compressed", f"/{ns}/{cam_name}/rgb"),
                    ("/out", f"/{ns}/{cam_name}/rgb_uncompressed"),
                ],
                condition=IfCondition("true"),
            )
            entities.append(republish_node)
        return entities

    # ============================================================
    # 3. Add all launch arguments and entities to LaunchDescription
    # ============================================================
    ld = LaunchDescription()
    for declared_argument in declared_arguments:
        ld.add_entity(declared_argument)
    ld.add_action(OpaqueFunction(function=build_entities))
    return ld
