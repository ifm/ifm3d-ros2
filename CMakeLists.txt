cmake_minimum_required(VERSION 3.5)
project(ifm3d_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IFM3D_ROS2_DEPS
  cv_bridge
  lifecycle_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rmw
  sensor_msgs
  std_msgs
  vision_opencv
  )

find_package(ifm3d 0.12.0 CONFIG REQUIRED COMPONENTS
  camera
  framegrabber
  image
  )
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${IFM3D_ROS2_DEPS})

#############
##  Build  ##
#############

include_directories(include)

#
# ifm3d camera "component" (.so) state machine ("lifecycle node")
#
add_library(ifm3d_ros2_camera_node SHARED src/lib/camera_node.cpp)
target_link_libraries(ifm3d_ros2_camera_node
  ifm3d::camera
  ifm3d::framegrabber
  ifm3d::image
  )
ament_target_dependencies(ifm3d_ros2_camera_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_camera_node "ifm3d_ros2::CameraNode"
  )

#
# Exe to bring up the camera component in a standalone way allow us to manually
# drive the state machine or some other external "manager" can do so (e.g.,
# launch_ros).
#
ament_auto_add_executable(camera_standalone src/bin/camera_standalone.cpp)
target_link_libraries(camera_standalone ifm3d_ros2_camera_node)
ament_target_dependencies(camera_standalone ${IFM3D_ROS2_DEPS})

##############
## Install  ##
##############

install(
  TARGETS camera_standalone
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  TARGETS ifm3d_ros2_camera_node
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
  DIRECTORY examples
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "examples/*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
  )

#############
##  Test   ##
#############
#find_package(launch_testing_ament_cmake)
#add_launch_test(test/latched_subscriber.test.py)


#############
ament_auto_package()
