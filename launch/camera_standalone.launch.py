# 
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
# 

import os
import sys
from math import pi

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import LifecycleNode

deprecation_warning = LogInfo(
    msg="""

    ######################################################################################
    #                                                                                    #
    #  This launch script is deprecated. Use the parametrized camera.launch.py instead!  #
    #                                                                                    #
    ######################################################################################
    """
)


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

    remaps = list(map(add_prefix, remaps))

    return LaunchDescription(
        [
            deprecation_warning,
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'tf2_ros',
                    'static_transform_publisher',
                    '0',
                    '0',
                    '0',
                    '0',
                    '0',
                    '0',
                    str(node_name + "_optical_link"),
                    str(node_name + "_link"),
                ],
                # output='screen',
                log_cmd=True,
            ),
            LifecycleNode(
                package=package_name,
                executable=node_exe,
                namespace=node_namespace,
                name=node_name,
                output='screen',
                parameters=parameters,
                remappings=remaps,
                log_cmd=True,
            ),
        ],
        deprecated_reason="Deprecated in favor of camera.launch.py"
    )
