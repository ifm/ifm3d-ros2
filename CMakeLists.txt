cmake_minimum_required(VERSION 3.5)
project(ifm3d_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IFM3D_ROS2_DEPS
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rcl_interfaces
  rmw
  rosidl_default_generators
  sensor_msgs
  std_msgs
  tf2_ros
  )

find_package(ifm3d 1.2.6 CONFIG REQUIRED COMPONENTS
  device
  framegrabber
  )
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${IFM3D_ROS2_DEPS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Extrinsics.msg"
  "msg/Intrinsics.msg"
  "msg/InverseIntrinsics.msg"
  "msg/RGBInfo.msg"
  "msg/TOFInfo.msg"
  "msg/Zones.msg"
  "srv/Dump.srv"
  "srv/Config.srv"
  "srv/GetDiag.srv"
  "srv/Softoff.srv"
  "srv/Softon.srv"
  DEPENDENCIES builtin_interfaces std_msgs
  )

#############
##  Build  ##
#############

include_directories(include)

# Target for messages included in this package
rosidl_get_typesupport_target(msgs_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_library(ifm3d_ros2_buffer_conversions SHARED
  src/lib/buffer_conversions.cpp)
target_link_libraries(ifm3d_ros2_buffer_conversions
  "${msgs_typesupport_target}"
  ifm3d::framegrabber)
ament_target_dependencies(ifm3d_ros2_buffer_conversions ${IFM3D_ROS2_DEPS})

add_library(ifm3d_ros2_buffer_id_utils SHARED
  src/lib/buffer_id_utils.cpp)
target_link_libraries(ifm3d_ros2_buffer_id_utils 
  "${msgs_typesupport_target}"
  ifm3d::framegrabber)
ament_target_dependencies(ifm3d_ros2_buffer_id_utils ${IFM3D_ROS2_DEPS})

# ifm3d camera "component" (.so) state machine ("lifecycle node")
#
add_library(ifm3d_ros2_camera_node SHARED
  src/lib/camera_node.cpp
  src/lib/diag_module.cpp
  src/lib/function_module.cpp
  src/lib/rgb_module.cpp
  src/lib/camera_tf_publisher.cpp
  src/lib/tof_module.cpp
  src/lib/services.cpp)
target_link_libraries(ifm3d_ros2_camera_node
  "${msgs_typesupport_target}"
  ifm3d_ros2_buffer_conversions
  ifm3d_ros2_buffer_id_utils
  ifm3d::device
  ifm3d::framegrabber
  )
ament_target_dependencies(ifm3d_ros2_camera_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_camera_node "ifm3d_ros2::CameraNode"
  )

#
# ifm3d ods node
#
add_library(ifm3d_ros2_ods_node SHARED
  src/lib/ods_node.cpp
  src/lib/function_module.cpp
  src/lib/diag_module.cpp
  src/lib/ods_module.cpp
  src/lib/services.cpp)
target_link_libraries(ifm3d_ros2_ods_node
  "${msgs_typesupport_target}"
  ifm3d_ros2_buffer_conversions
  ifm3d_ros2_buffer_id_utils
  ifm3d::device
  ifm3d::framegrabber
  )
ament_target_dependencies(ifm3d_ros2_ods_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_ods_node "ifm3d_ros2::OdsNode"
  )

#
# Exe to bring up the camera component in a standalone way allow us to manually
# drive the state machine or some other external "manager" can do so (e.g.,
# launch_ros).
#
ament_auto_add_executable(camera_standalone src/bin/camera_standalone.cpp)
target_link_libraries(camera_standalone
  "${msgs_typesupport_target}"
  ifm3d_ros2_camera_node)
ament_target_dependencies(camera_standalone ${IFM3D_ROS2_DEPS})
target_link_libraries(ifm3d_ros2_camera_node ${Boost_LIBRARIES})


ament_auto_add_executable(ods_standalone src/bin/ods_standalone.cpp)
target_link_libraries(ods_standalone
  "${msgs_typesupport_target}"
  ifm3d_ros2_ods_node)
ament_target_dependencies(ods_standalone ${IFM3D_ROS2_DEPS})
target_link_libraries(ifm3d_ros2_ods_node ${Boost_LIBRARIES}) # TODO needed?


##############
## Install  ##
##############

install(
  TARGETS camera_standalone ods_standalone
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  TARGETS ifm3d_ros2_camera_node ifm3d_ros2_ods_node ifm3d_ros2_buffer_conversions ifm3d_ros2_buffer_id_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(
  DIRECTORY etc
  DESTINATION share/${PROJECT_NAME}/
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

#############
##  Test   ##
#############
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/lifecycle.test.py)
  #
  # At the moment, the test(s) below cannot be run with `colcon test` b/c the
  # python scripts which implement the tests cannot load `ifm3d_ros2.msg`
  # module. I am not sure if this is a problem with colon or how I have set up
  # my tests.
  #
  # Despite not being able to run the tests with `colcon test`, they can be
  # direclty run with `launch_test`. In this method, the tests should pass.
  #
  #add_launch_test(test/pointcloud.test.py)
endif(BUILD_TESTING)

#############
ament_auto_package()
