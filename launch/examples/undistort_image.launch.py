from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # undistortion node
        Node(
            package="image_proc",
            executable="image_proc",
            name="undistort_image",
            namespace="/ifm3d/camera_2d", # please change this to your camera namespace
            remappings=[
                ("image", "rgb_uncompressed"),         # input image topic relative to namespace (3d or rgb uncompressed image)
                ("camera_info", "camera_info")         # camera info topic relative to namespace
            ],
            parameters=[{
                "approx_sync": True,
                "use_sensor_qos": True,
                "publish_color": False,       
                "publish_mono": False,        # Disables image_mono
                "publish_rect": True,         # Ensures image_rect is published
            }]
        )
    ])