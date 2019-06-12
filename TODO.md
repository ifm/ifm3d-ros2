TODO
====

* It is not clear how to mimic ROS 1 "latching" behavior. To that end, we are
  currently publishing the Unit Vectors on each loop iteration. To be clear, we
  are not streaming the unit vectors continuously from the camera, we fetch
  them once and then cache them on the ROS node. However, we currently SPAM the
  ROS/DDS network with Unit Vector data. We need to investigate how to mimic a
  latched topic in ROS2 or perhaps move the Unit Vectors into a service
  call. The service call approach could be made backward compatibile once we
  have a latched topic -- i.e., maintain both the service call and latched
  topic. In either case, the same cached data are returned.

  Here is a useful test snippet for late subscription (add this to a unit
  test):

```
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def main(args=None):
    topic = '/ifm3d/camera/unit_vectors'
    topic_type = Image

    rclpy.init(args=args)
    qos_profile = QoSProfile(
        depth=1,
        # Guaranteed delivery is needed to send messages to late-joining subscription.
        reliability=QoSReliabilityPolicy.RELIABLE,
        # Store messages on the publisher so that they can be affected by Lifespan
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
    node = Node('listener')
    sub = node.create_subscription(
        topic_type,
        topic,
        lambda msg: print("hi"),
        qos_profile)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

* Expose the intrinsic camera calibration via CameraInfo
* Expose the on-camera extrinsic calibration
* Investigate performance through the ROS middleware, tune QoS for our camera
* Unit tests
* Write an inproc demo node
* Port ROS Services from our ROS 1 Node (e.g., dump, config, etc.). For now,
  users should use the underlying `ifm3d dump ...` and `ifm3d config ...`
  command line utilities.
* Investigate using/need for ImageTransport (compressed image topics?) like our
  ROS 1 node.
* Create an installable `snap` for `snapd` supported systems
