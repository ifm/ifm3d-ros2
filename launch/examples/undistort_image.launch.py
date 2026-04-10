from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # 2d camera_node
            IncludeLaunchDescription(
                [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
                launch_arguments={
                    'camera_name': 'camera_2d',
                    'parameter_file_package': 'ifm3d_ros2',
                    'parameter_file_directory': 'config/examples',
                    'parameter_file_name': 'combined_2d_3d.yaml',
                }.items(),
            ),
            # 2d undistortion node
            Node(
                package="image_proc",
                executable="image_proc",
                name="undistort_image",
                namespace="/ifm3d/camera_2d",  # please change this to your camera namespace
                remappings=[
                    (
                        "image",
                        "rgb_uncompressed",
                    ),  # input image topic relative to namespace (3d or rgb uncompressed image)
                ],
                parameters=[
                    {
                        "approx_sync": True,
                        "use_sensor_qos": True,
                        "publish_color": False,
                        "publish_mono": False,  # Disables image_mono
                        "publish_rect": True,  # Ensures image_rect is published
                        "qos_overrides./parameter_events.publisher.reliability": "best_effort",
                    }
                ],
            ),
            # 3d camera_node
            IncludeLaunchDescription(
                [FindPackageShare("ifm3d_ros2"), '/launch', '/camera.launch.py'],
                launch_arguments={
                    'camera_name': 'camera_3d',
                    'parameter_file_package': 'ifm3d_ros2',
                    'parameter_file_directory': 'config/examples',
                    'parameter_file_name': 'combined_2d_3d.yaml',
                }.items(),
            ),
            # 3d undistortion node
            Node(
                package="image_proc",
                executable="image_proc",
                name="undistort_image",
                namespace="/ifm3d/camera_3d",  # please change this to your camera namespace
                remappings=[
                    (
                        "image",
                        "distance",
                    ),  # input image topic relative to namespace (3d or rgb uncompressed image)
                ],
                parameters=[
                    {
                        "approx_sync": True,
                        "use_sensor_qos": True,
                        "publish_color": False,
                        "publish_mono": False,  # Disables image_mono
                        "publish_rect": True,  # Ensures image_rect is published
                        "qos_overrides./parameter_events.publisher.reliability": "best_effort",
                    }
                ],
            ),
        ]
    )
