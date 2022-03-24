
echo "Setting up transorfms"
(trap 'kill 0' SIGINT; 
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera0_optical_link map &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera1_optical_link camera0_optical_link &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera2_optical_link camera1_optical_link &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera3_optical_link camera2_optical_link
    )
