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
import sys
import unittest

import rclpy
from rclpy.context import Context
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image

class TestLatchedSubscriber(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init()
        cls.node = rclpy.create_node('uvec_listener')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_latched_topic(self):
        topic = '/ifm3d/camera/unit_vectors'
        topic_type = Image

        self._got_uvec_flg = False
        def uvec_cb(msg):
            #print(msg)
            self._got_uvec_flg = True

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

        sub = self.node.create_subscription(
            topic_type,
            topic,
            uvec_cb,
            qos_profile
            )

        cycle_count = 0
        while cycle_count < 5:
            if self._got_uvec_flg:
                break

            rclpy.spin_once(self.node, timeout_sec=1)
            cycle_count += 1

        self.assertTrue(self._got_uvec_flg)
        self.assertTrue(self.node.destroy_subscription(sub))

if __name__ == '__main__':
    unittest.main()
