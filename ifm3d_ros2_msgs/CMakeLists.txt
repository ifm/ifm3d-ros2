cmake_minimum_required(VERSION 3.5)
project(ifm3d_ros2_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Extrinsics.msg"
  "srv/Dump.srv"
  "srv/Config.srv"
  "srv/Softoff.srv"
  "srv/Softon.srv"
  )
#   DEPENDENCIES builtin_interfaces std_msgs

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
