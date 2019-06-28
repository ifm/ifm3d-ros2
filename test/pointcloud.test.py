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
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os
import unittest
import numpy as np

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
from ifm3d_ros2.msg import Extrinsics
from cv_bridge import CvBridge

NS_ = "/ifm3d"
NN_ = "camera"
UVEC_TOPIC_ = "%s/%s/unit_vectors" % (NS_, NN_)
XYZ_TOPIC_ = "%s/%s/xyz_image" % (NS_, NN_)
EXTR_TOPIC_ = "%s/%s/extrinsics" % (NS_, NN_)
DIST_TOPIC_ = "%s/%s/distance" % (NS_, NN_)
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


class TestPointCloud(unittest.TestCase):

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
        cls.bridge_ = CvBridge()
        # These are all cached ROS message (not numpy arrays)
        cls.uvec_ = None
        cls.extr_ = None
        cls.cloud_ = None
        cls.dist_ = None

    @classmethod
    def tearDownClass(cls):
        cls.node_.destroy_client(cls.change_state_client_)
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
        self.do_get_state_("unconfigured")
        self.do_transition_(Transition.TRANSITION_CONFIGURE)
        self.do_get_state_("inactive")
        self.do_transition_(Transition.TRANSITION_ACTIVATE)
        self.do_get_state_("active")

    def test_00100_cache_unit_vectors(self):
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

        def uvec_cb_(msg):
            self.__class__.uvec_ = msg

        sub = self.node_.create_subscription(
            Image, UVEC_TOPIC_, uvec_cb_, qos_profile
            )

        self.assertTrue(self.__class__.uvec_ is None)

        n = 0
        while n < 5:
            rclpy.spin_once(self.node_, executor=self.exe_, timeout_sec=1.0)
            if self.__class__.uvec_ is not None:
                break
            n += 1

        self.assertTrue(self.__class__.uvec_ is not None)
        self.assertTrue(self.node_.destroy_subscription(sub))

    def test_00101_cache_extrinsics(self):
        # we assume for these unit tests that the extrinsics are
        # static (i.e., just chip to glass, nothing mutated via xmlrpc)
        # so, we do not time correleate them to the cloud or radial distance
        # image.
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
            )

        def extr_cb_(msg):
            self.__class__.extr_ = msg

        sub = self.node_.create_subscription(
            Extrinsics, EXTR_TOPIC_, extr_cb_, qos_profile
            )

        self.assertTrue(self.__class__.extr_ is None)

        n = 0
        while n < 5:
            rclpy.spin_once(self.node_, executor=self.exe_, timeout_sec=1.0)
            if self.__class__.extr_ is not None:
                break
            n += 1

        self.assertTrue(self.__class__.extr_ is not None)
        self.assertTrue(self.node_.destroy_subscription(sub))

    def test_00102_cache_cloud_and_dist(self):
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
            )

        def cloud_cb_(msg):
            self.__class__.cloud_ = msg

        def dist_cb_(msg):
            self.__class__.dist_ = msg

        cloud_sub = self.node_.create_subscription(
            Image, XYZ_TOPIC_, cloud_cb_, qos_profile
            )

        dist_sub = self.node_.create_subscription(
            Image, DIST_TOPIC_, dist_cb_, qos_profile
            )

        n = 0
        while n < 20:
            rclpy.spin_once(self.node_, executor=self.exe_, timeout_sec=1.0)
            if ((self.__class__.cloud_ is not None) and
                    (self.__class__.dist_ is not None)):
                if (self.__class__.cloud_.header.stamp ==
                        self.__class__.dist_.header.stamp):
                    break
            n += 1

        self.assertTrue(self.__class__.cloud_ is not None)
        self.assertTrue(self.__class__.dist_ is not None)

        self.assertTrue(self.node_.destroy_subscription(dist_sub))
        self.assertTrue(self.node_.destroy_subscription(cloud_sub))

    def test_00200_compute_cartesian(self):
        # ground truth Cartesian data (point cloud computed on camera) in mm
        cloud = self.bridge_.imgmsg_to_cv2(self.__class__.cloud_)
        uvec = self.bridge_.imgmsg_to_cv2(self.__class__.uvec_)
        dist = self.bridge_.imgmsg_to_cv2(self.__class__.dist_) # mm
        # translation units are in mm
        tx = self.__class__.extr_.tx
        ty = self.__class__.extr_.ty
        tz = self.__class__.extr_.tz

        ex = uvec[:,:,0]
        ey = uvec[:,:,1]
        ez = uvec[:,:,2]

        # cast distance image to float
        rdis_f = dist.astype(np.float32)
        if (dist.dtype == np.float32):
            # if dist values were float, assume they were meters and so convert
            # to mm for the comparisons
            rdis_f *= 1000.

        # compute Cartesian (in optical frame)
        x_ = ex * rdis_f + tx
        y_ = ey * rdis_f + ty
        z_ = ez * rdis_f + tz

        # Account for bad pixels
        bad_mask = dist == 0
        x_[bad_mask] = 0
        y_[bad_mask] = 0
        z_[bad_mask] = 0

        # convert to camera coord frame (we are mm, so we also convert to an int
        # type).
        x = z_.astype(np.int16)
        y = -(x_.astype(np.int16))
        z = -(y_.astype(np.int16))

        tol = 1 # mm
        x_mask = np.fabs(x - cloud[:,:,0]) > tol
        y_mask = np.fabs(y - cloud[:,:,1]) > tol
        z_mask = np.fabs(z - cloud[:,:,2]) > tol

        self.assertTrue(x_mask.sum() == 0)
        self.assertTrue(y_mask.sum() == 0)
        self.assertTrue(z_mask.sum() == 0)

    def test_09900_node_done(self):
        self.do_transition_(Transition.TRANSITION_DEACTIVATE)
        self.do_get_state_("inactive")
        self.do_transition_(Transition.TRANSITION_CLEANUP)
        self.do_get_state_("unconfigured")
        self.do_transition_(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
        self.do_get_state_("finalized")


@launch_testing.post_shutdown_test()
class TestOutput(unittest.TestCase):

    def test_00000_exit_codes(self):
        launch_testing.asserts.assertExitCodes(self.proc_info)
