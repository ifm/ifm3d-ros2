#
# Copyright (C)  2019 ifm electronic, gmbh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distribted on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import sys
from math import pi

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

def generate_launch_description():
    package_name = 'ifm3d_ros2'
    node_namespace = 'ifm3d'
    node_name = 'camera'
    node_exe = 'camera_standalone'

    # os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = \
    #   "[{%s}] {%s} [{%s}]: {%s}\n({%s}() at {%s}:{%s})" % \
    #    ("severity", "time", "name", "message",
    #      "function_name", "file_name", "line_number")
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = \
      "[{%s}] {%s} [{%s}]: {%s}" % ("severity", "time", "name", "message")


    # XXX: This is a hack, there does not seem to be a nice way (or at least
    # finding the docs is not obvious) to do this with the ROS2 launch api
    #
    # Basically, we are trying to allow for passing through the command line
    # args to the launch file through to the node executable itself (like ROS
    # 1).
    #
    # My assumption is that either:
    # 1. This stuff exists somewhere in ROS2 and I don't know about it yet
    # 2. This stuff will exist in ROS2 soon, so, this will likely get factored
    #    out (hopefully soon)
    #
    parameters = []
    remaps = []
    for arg in sys.argv:
        if ':=' in arg:
            split_arg = arg.split(sep=':=', maxsplit=1)
            assert len(split_arg) == 2

            if arg.startswith("ns"):
                node_namespace = split_arg[1]
            elif arg.startswith("node"):
                node_name = split_arg[1]
            elif arg.startswith("params"):
                parameters.append(tuple(split_arg)[1])
            else:
                remaps.append(tuple(split_arg))

    def add_prefix(tup):
        assert len(tup) == 2
        if node_namespace.startswith("/"):
            prefix = "%s/%s" % (node_namespace, node_name)
        else:
            prefix = "/%s/%s" % (node_namespace, node_name)

        retval = [None, None]

        if not tup[0].startswith(prefix):
            retval[0] = prefix + '/' + tup[0]
        else:
            retval[0] = tup[0]

        if not tup[1].startswith(prefix):
            retval[1] = prefix + '/' + tup[1]
        else:
            retval[1] = tup[1]

        return tuple(retval)

    print("After add prefix")
    remaps = list(map(add_prefix, remaps))
    print("After remaps")
    #------------------------------------------------------------
    # Nodes
    #------------------------------------------------------------

    #
    # Coord frame transform from camera_optical_link to camera_link
    #
    tf_node = \
      ExecuteProcess(
          cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
               '0', '0', '0', str(-pi/2.), '0', str(-pi/2.),
               str(node_name + "_link"), str(node_name + "_optical_link")],
               #output='screen',
               log_cmd=True
          )
    print("After tf node exec")
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
    print("After camera node lifecycle def")
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
    print("After configure evt")

    camera_activate_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(camera_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
              )
          )
    print("After activate evt")

    camera_cleanup_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(camera_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP
              )
          )
    print("After cleanup evt")

    camera_shutdown_evt = EmitEvent(event=launch.events.Shutdown())
    print("After shutdown evt")

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
    print("After unconf to inactive")
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
    print("After active to inactive")
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
    print("After inactive to unconfigured")
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
    print("After error to unconfigured")
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
    print("After shutdown to finalized")
    #------------------------------------------------------------
    # Now, add all the actions to a launch description and return it
    #------------------------------------------------------------

    ld = LaunchDescription()
    ld.add_action(camera_node_unconfigured_to_inactive_handler)
    ld.add_action(camera_node_active_to_inactive_handler)
    ld.add_action(camera_node_inactive_to_unconfigured_handler)
    ld.add_action(camera_node_errorprocessing_to_unconfigured_handler)
    ld.add_action(camera_node_shuttingdown_to_finalized_handler)
    ld.add_action(tf_node)
    ld.add_action(camera_node)
    ld.add_action(camera_configure_evt)

    return ld
