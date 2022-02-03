# 
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2019 ifm electronic, gmbh
# 

import os
import unittest

from ament_index_python import get_package_prefix
import launch
from launch.actions import ExecuteProcess, OpaqueFunction
import launch_testing
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image

NS_ = "/ifm3d"
NN_ = "camera"
UVEC_TOPIC_ = "%s/%s/unit_vectors" % (NS_, NN_)
AMP_TOPIC_ = "%s/%s/amplitude" % (NS_, NN_)
SRV_TIMEOUT = 2.0 # seconds

def generate_test_description(ready_fn):
    package_name = 'ifm3d_ros2'
    node_exe = 'camera_standalone'
    namespace = NS_
    nodename = NN_

    exe = os.path.join(
        get_package_prefix(package_name),
        'lib', package_name, node_exe
        )

    return launch.LaunchDescription([
        ExecuteProcess(
            cmd=[exe, '__ns:=%s' % namespace, '__node:=%s' % nodename],
            log_cmd=True,
            output='screen'
            ),
        OpaqueFunction(function=lambda context: ready_fn()),
        ])


class TestLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node_ = rclpy.create_node('ifm3d_test_node')
        cls.exe_ = SingleThreadedExecutor()
        cls.get_state_client_ = \
          cls.node_.create_client(GetState, "%s/%s/get_state" % (NS_, NN_))
        cls.change_state_client_ = \
           cls.node_.create_client(
               ChangeState, "%s/%s/change_state" % (NS_, NN_))
        cls.got_uvec_ = False
        cls.got_amp_ = False

    @classmethod
    def tearDownClass(cls):
        cls.node_.destroy_client(cls.get_state_client_)
        cls.node_.destroy_node()
        rclpy.shutdown()

    def do_transition_(self, id):
        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = id
        fut = self.change_state_client_.call_async(req)
        rclpy.spin_until_future_complete(self.node_, fut, executor=self.exe_)
        resp = fut.result()
        self.assertTrue(resp is not None)
        self.assertTrue(resp.success)

    def do_get_state_(self, label=""):
        fut = self.get_state_client_.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node_, fut, executor=self.exe_)
        resp = fut.result()
        self.assertTrue(resp is not None)
        self.assertEqual(resp.current_state.label, label)

    def test_00000_node_ready(self):
        self.assertTrue(
            self.get_state_client_.wait_for_service(timeout_sec=SRV_TIMEOUT))
        self.assertTrue(
            self.change_state_client_.wait_for_service(timeout_sec=SRV_TIMEOUT))

    def test_00100_unconfigured(self):
        self.do_get_state_("unconfigured")

    def test_00200_configure(self):
        self.do_transition_(Transition.TRANSITION_CONFIGURE)

    def test_00300_inactive(self):
        self.do_get_state_("inactive")

    def test_00400_activate(self):
        self.do_transition_(Transition.TRANSITION_ACTIVATE)

    def test_00500_active(self):
        self.do_get_state_("active")

    def test_00600_latched_subscription(self):
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

        def uvec_cb_(msg):
            self.got_uvec_ = True

        sub = self.node_.create_subscription(
            Image, UVEC_TOPIC_, uvec_cb_, qos_profile
            )

        self.assertFalse(self.got_uvec_)

        n = 0
        while n < 5:
            rclpy.spin_once(self.node_, executor=self.exe_, timeout_sec=1.0)
            if self.got_uvec_:
                break
            n += 1

        self.assertTrue(self.got_uvec_)
        self.assertTrue(self.node_.destroy_subscription(sub))

    def test_00601_amplitude_subscription(self):
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
            )

        def amp_cb_(msg):
            self.got_amp_ = True

        sub = self.node_.create_subscription(
            Image, AMP_TOPIC_, amp_cb_, qos_profile
            )

        self.assertFalse(self.got_amp_)

        n = 0
        while n < 5:
            rclpy.spin_once(self.node_, executor=self.exe_, timeout_sec=1.0)
            if self.got_amp_:
                break
            n += 1

        self.assertTrue(self.got_amp_)
        self.assertTrue(self.node_.destroy_subscription(sub))

    # XXX: add other tests to topics/services here in the
    # test_006XX_XXX range (while the node is 'active')

    def test_00700_deactivate(self):
        self.do_transition_(Transition.TRANSITION_DEACTIVATE)

    def test_00800_inactive(self):
        self.do_get_state_("inactive")

    def test_00900_cleanup(self):
        self.do_transition_(Transition.TRANSITION_CLEANUP)

    def test_01000_unconfigured(self):
        self.do_get_state_("unconfigured")

    def test_01100_shutdown(self):
        self.do_transition_(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)

    def test_01200_finalized(self):
        self.do_get_state_("finalized")


@launch_testing.post_shutdown_test()
class TestLifecycleOutput(unittest.TestCase):

    def test_00000_exit_codes(self):
        launch_testing.asserts.assertExitCodes(self.proc_info)
