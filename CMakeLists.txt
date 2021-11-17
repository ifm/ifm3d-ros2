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
  cv_bridge
  lifecycle_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rcl_interfaces
  rmw
  rosidl_default_generators
  sensor_msgs
  std_msgs
  vision_opencv
  )

find_package(ifm3d 0.91.0 CONFIG REQUIRED COMPONENTS
  camera
  framegrabber
  image
  )
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${IFM3D_ROS2_DEPS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Extrinsics.msg"
  "srv/Dump.srv"
  "srv/Config.srv"
  DEPENDENCIES builtin_interfaces std_msgs
  )

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
rosidl_target_interfaces(ifm3d_ros2_camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )

#
# Exe to bring up the camera component in a standalone way allow us to manually
# drive the state machine or some other external "manager" can do so (e.g.,
# launch_ros).
#
ament_auto_add_executable(camera_standalone src/bin/camera_standalone.cpp)
target_link_libraries(camera_standalone ifm3d_ros2_camera_node)
ament_target_dependencies(camera_standalone ${IFM3D_ROS2_DEPS})
rosidl_target_interfaces(camera_standalone
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )

find_package(Boost 1.65.0 REQUIRED COMPONENTS system)
target_link_libraries(ifm3d_ros2_camera_node ${Boost_LIBRARIES})

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
  PROGRAMS scripts/dump scripts/config
  DESTINATION lib/${PROJECT_NAME}/
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
