# 
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
# 



import os
from math import pi
import logging

import lifecycle_msgs.msg
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

deprecation_warning = LogInfo(
    msg="""

    ######################################################################################
    #                                                                                    #
    #  This launch script is deprecated. Use the parametrized camera.launch.py instead!  #
    #                                                                                    #
    ######################################################################################
    """
)

logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)

def launch_setup(context, *args, **kwargs):


    package_name = 'ifm3d_ros2'
    node_exe = 'camera_standalone'
    parameters = []
    remaps = []

    node_name = LaunchConfiguration('name').perform(context)
    params = LaunchConfiguration('params').perform(context)
    node_namespace = LaunchConfiguration('namespace').perform(context)

    parameters.append(params)

    #------------------------------------------------------------
    # Nodes
    #------------------------------------------------------------


    #
    # The camera component
    #
    camera_node = \
      LifecycleNode(
          package=package_name,
          executable=node_exe,
          namespace=node_namespace,
          name=node_name,
          output='screen',
          parameters=parameters,
          remappings=remaps,
          log_cmd=True,
          )

    logging.debug(vars(camera_node))
    #------------------------------------------------------------
    # Events we need to emit to induce state transitions
    #------------------------------------------------------------

    camera_configure_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(camera_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
              )
          )

    camera_activate_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(camera_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
              )
          )

    camera_cleanup_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(camera_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP
              )
          )

    camera_shutdown_evt = EmitEvent(event=launch.events.Shutdown())

    #------------------------------------------------------------
    # These are the edges of the state machine graph we want to autonomously
    # manage
    #------------------------------------------------------------

    #
    # unconfigured -> configuring -> inactive
    #
    camera_node_unconfigured_to_inactive_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = camera_node,
              start_state = 'configuring',
              goal_state = 'inactive',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_ACTIVATE' event"),
                  camera_activate_evt,
                  ],
              )
          )
    #
    # active -> deactivating -> inactive
    #
    camera_node_active_to_inactive_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = camera_node,
              start_state = 'deactivating',
              goal_state = 'inactive',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CLEANUP' event"),
                  camera_cleanup_evt,
                  ],
              )
          )
    #
    # inactive -> cleaningup -> unconfigured
    #
    camera_node_inactive_to_unconfigured_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = camera_node,
              start_state = 'cleaningup',
              goal_state = 'unconfigured',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CONFIGURE' event"),
                  camera_configure_evt,
                  ],
              )
          )
    #
    # * -> errorprocessing -> unconfigured
    #
    camera_node_errorprocessing_to_unconfigured_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = camera_node,
              start_state = 'errorprocessing',
              goal_state = 'unconfigured',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CONFIGURE' event"),
                  camera_configure_evt,
                  ],
              )
          )
    #
    # * -> shuttingdown -> finalized
    #
    camera_node_shuttingdown_to_finalized_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = camera_node,
              start_state = 'shuttingdown',
              goal_state = 'finalized',
              entities = [
                  LogInfo(msg = "Emitting 'SHUTDOWN' event"),
                  camera_shutdown_evt,
                  ],
              )
          )

    #
    # Coord frame transform from camera_optical_link to camera_link
    #
    tf_node = \
      ExecuteProcess(
          cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0',
                str(node_name + "_optical_link"), str(node_name + "_link")],
                # output='screen',
                log_cmd=True
          )
    logging.info("Publishing tf2 transform from {} to {}" .format(str(node_name + "_optical_link"), str(node_name + "_link")))

    #
    # (Dummy) Coord frame transform from camera_link to map frame
    #
    tf_map_link_node = \
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0',
                "map", str(node_name + "_optical_link")],
                # output='screen',
                log_cmd=True
        )
    logging.info("Publishing tf2 transform from {} to {}" .format("map", str(node_name + "_optical_link")))

    return camera_node_unconfigured_to_inactive_handler, \
        camera_node_active_to_inactive_handler, \
        camera_node_inactive_to_unconfigured_handler, \
        camera_node_errorprocessing_to_unconfigured_handler, \
        camera_node_shuttingdown_to_finalized_handler, \
        camera_node, \
        camera_configure_evt, \
        tf_node, \
        tf_map_link_node \


def generate_launch_description():

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = \
      "[{%s}] {%s} [{%s}]: {%s}" % ("severity", "time", "name", "message")
    

    return LaunchDescription(
        [
            deprecation_warning,
            DeclareLaunchArgument('name', default_value='camera'),
            DeclareLaunchArgument('params', default_value=[]),
            DeclareLaunchArgument('namespace', default_value='ifm3d'),
            OpaqueFunction(function=launch_setup),
        ],
        deprecated_reason="Deprecated in favor of camera.launch.py"
    )
