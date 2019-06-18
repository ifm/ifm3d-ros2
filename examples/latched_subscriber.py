#!/usr/bin/env python3

#
# Quick script to test that latching the unit vectors is working. Simply run
# the script once the camera_node is up. This will act as a late
# subscriber. You should see the unit vectors Image message serialized to the
# screen. Press Ctl-C to exit.
#

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image

def main(args=None):
    topic = '/ifm3d/camera/unit_vectors'
    topic_type = Image

    rclpy.init(args=args)
    qos_profile = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
    node = Node('uvec_listener')
    sub = node.create_subscription(
        topic_type,
        topic,
        lambda msg: print(msg),
        qos_profile)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

    finally:
        return 0

if __name__ == "__main__":
    sys.exit(main())
