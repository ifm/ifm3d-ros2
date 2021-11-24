#!/usr/bin/env python3

#
# Example showing how to compute the Cartesian data (point cloud) off-board the
# camera from the Unit Vectors, Extrinsics Calibration, and Distance Image.
#
import sys
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from ifm3d_ros2.msg import Extrinsics
from cv_bridge import CvBridge

#
# NOTE: As of the date of writing this script, the ROS2 python implementation
# of message_filters.TimeSchronizer had issues (i.e., did not handle QoS
# properly and a few other things). So, I am manually time syncing the
# extrinsics and the radial distance data.
#

class CartesianCompute(object):

    def __init__(self, node):
        self.node_ = node

        self.topic_uvec_ = "/ifm3d/camera/unit_vectors"
        self.topic_extr_ = "/ifm3d/camera/extrinsics"
        self.topic_dist_ = "/ifm3d/camera/distance"

        self.bridge_ = CvBridge()

        self.lock_ = threading.Lock()
        self.uvec_ = None
        self.extr_ = None

        self.sub_uvec_ = self.node_.create_subscription(
            Image,
            self.topic_uvec_,
            self.uvec_cb,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
                )
            )

        qos = QoSProfile(
                depth=2,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
                )

        self.sub_extr_ = self.node_.create_subscription(
            Extrinsics, self.topic_extr_, self.extr_cb, qos)

        self.sub_dist_ = self.node_.create_subscription(
            Image, self.topic_dist_, self.dist_cb, qos)

    def uvec_cb(self, msg):
        with self.lock_:
            self.uvec_ = self.bridge_.imgmsg_to_cv2(msg)

    def extr_cb(self, msg):
        with self.lock_:
            self.extr_ = msg

    def dist_cb(self, msg):
        dist = None
        tx = None
        ty = None
        tz = None

        with self.lock_:
            if ((self.extr_ is not None) and (self.uvec_ is not None)):
                if msg.header.stamp == self.extr_.header.stamp:
                    dist = self.bridge_.imgmsg_to_cv2(msg)
                    tx = self.extr_.tx
                    ty = self.extr_.ty
                    tz = self.extr_.tz
                else:
                    return
            else:
                return

        # unit vectors
        ex = self.uvec_[:,:,0]
        ey = self.uvec_[:,:,1]
        ez = self.uvec_[:,:,2]

        # cast distance image to float
        rdis_f = dist.astype(np.float32)
        if (dist.dtype == np.float32):
            # assume dist was in meters, convert to mm
            rdis_f *= 1000.

        # compute cartesian
        x_ = ex * rdis_f + tx
        y_ = ey * rdis_f + ty
        z_ = ez * rdis_f + tz

        # convert to camera frame
        x = z_
        y = -x_
        z = -y_

        # print results in meters
        print(np.dstack((x,y,z))/1000.)

def main():
    rclpy.init()
    node = Node('cartesian_compute')

    try:
        c = CartesianCompute(node)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0

if __name__ == '__main__':
    sys.exit(main())
