cmake_minimum_required(VERSION 3.5)
project(ifm3d_ros2)

set(USE_SCOPED_HEADER_INSTALL_DIR ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IFM3D_ROS2_DEPS
  action_msgs
  ament_index_cpp
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
  tf2_geometry_msgs
  tf2_ros
  unique_identifier_msgs
  visualization_msgs
  )

# Find ifm3d dependencies
find_package(ifm3d REQUIRED CONFIG)
if(ifm3d_VERSION VERSION_GREATER_EQUAL "1.6.12")
  set(IFM3D_LIBS ifm3d::ifm3d)
else()
  find_package(ifm3d REQUIRED CONFIG COMPONENTS device framegrabber deserialize swupdater)
  set(IFM3D_LIBS ifm3d::device ifm3d::framegrabber ifm3d::deserialize ifm3d::swupdater)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${IFM3D_ROS2_DEPS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GetPallet.action"
  "action/GetRack.action"
  "action/VolumeCheck.action"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxStamped.msg"
  "msg/GetPalletRequest.msg"
  "msg/GetRackRequest.msg"
  "msg/Extrinsics.msg"
  "msg/ExtrinsicsCalibrationCorrection.msg"
  "msg/ImuBurst.msg"
  "msg/Intrinsics.msg"
  "msg/InverseIntrinsics.msg"
  "msg/PalletDetection.msg"
  "msg/PalletDetectionArray.msg"
  "msg/PdsFullResult.msg"
  "msg/RGBInfo.msg"
  "msg/RackDetection.msg"
  "msg/TOFInfo.msg"
  "msg/VolumeCheck.msg"
  "msg/Zones.msg"
  "srv/Dump.srv"
  "srv/Config.srv"
  "srv/GetDiag.srv"
  "srv/SetPdsMode.srv"
  "srv/Softoff.srv"
  "srv/Softon.srv"
  DEPENDENCIES action_msgs builtin_interfaces sensor_msgs std_msgs unique_identifier_msgs
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
  ${IFM3D_LIBS})
ament_target_dependencies(ifm3d_ros2_buffer_conversions ${IFM3D_ROS2_DEPS})

add_library(ifm3d_ros2_buffer_id_utils SHARED
  src/lib/buffer_id_utils.cpp)
target_link_libraries(ifm3d_ros2_buffer_id_utils 
  "${msgs_typesupport_target}"
  ${IFM3D_LIBS})
ament_target_dependencies(ifm3d_ros2_buffer_id_utils ${IFM3D_ROS2_DEPS})

# ifm3d camera "component" (.so) state machine ("lifecycle node")
#
add_library(ifm3d_ros2_camera_node SHARED
  src/lib/camera_node.cpp
  src/lib/camera_tf_publisher.cpp
  src/lib/diag_module.cpp
  src/lib/function_module.cpp
  src/lib/rgb_module.cpp
  src/lib/sensor_tf_publisher.cpp
  src/lib/tof_module.cpp
  src/lib/services.cpp)
target_link_libraries(ifm3d_ros2_camera_node
  "${msgs_typesupport_target}"
  ifm3d_ros2_buffer_conversions
  ifm3d_ros2_buffer_id_utils
  ${IFM3D_LIBS}
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
  ${IFM3D_LIBS}
  )
ament_target_dependencies(ifm3d_ros2_ods_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_ods_node "ifm3d_ros2::OdsNode"
  )

#
# ifm3d pds node
#
add_library(ifm3d_ros2_pds_node SHARED
  src/lib/pds_node.cpp
  src/lib/pds_vis_node.cpp
  src/lib/function_module.cpp
  src/lib/diag_module.cpp
  src/lib/pds_module.cpp
  src/lib/services.cpp)
target_link_libraries(ifm3d_ros2_pds_node
  "${msgs_typesupport_target}"
  ${IFM3D_LIBS}
  )
ament_target_dependencies(ifm3d_ros2_pds_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_pds_node "ifm3d_ros2::PdsNode"
  )

#
# ifm3d imu node
#
add_library(ifm3d_ros2_imu_node SHARED
  src/lib/imu_node.cpp
  src/lib/function_module.cpp
  src/lib/diag_module.cpp
  src/lib/imu_module.cpp
  src/lib/sensor_tf_publisher.cpp
  src/lib/imu_tf_publisher.cpp
  src/lib/services.cpp
)
target_link_libraries(ifm3d_ros2_imu_node
  "${msgs_typesupport_target}"
  ifm3d_ros2_buffer_conversions
  ifm3d_ros2_buffer_id_utils
  ${IFM3D_LIBS}
)
ament_target_dependencies(ifm3d_ros2_imu_node ${IFM3D_ROS2_DEPS})
rclcpp_components_register_nodes(
  ifm3d_ros2_imu_node "ifm3d_ros2::ImuNode"
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

ament_auto_add_executable(pds_standalone src/bin/pds_standalone.cpp)
target_link_libraries(pds_standalone
  "${msgs_typesupport_target}"
  ifm3d_ros2_pds_node)
ament_target_dependencies(pds_standalone ${IFM3D_ROS2_DEPS})

ament_auto_add_executable(pds_vis_standalone src/bin/pds_vis_standalone.cpp)
target_link_libraries(pds_vis_standalone
  "${msgs_typesupport_target}"
  ifm3d_ros2_pds_node)
ament_target_dependencies(pds_vis_standalone ${IFM3D_ROS2_DEPS})

ament_auto_add_executable(imu_standalone src/bin/imu_standalone.cpp)
target_link_libraries(imu_standalone
  ifm3d_ros2_imu_node
)
ament_target_dependencies(imu_standalone ${IFM3D_ROS2_DEPS})

##############
## Install  ##
##############

install(
  TARGETS camera_standalone ods_standalone pds_standalone imu_standalone
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  TARGETS ifm3d_ros2_camera_node ifm3d_ros2_ods_node ifm3d_ros2_pds_node ifm3d_ros2_imu_node ifm3d_ros2_buffer_conversions ifm3d_ros2_buffer_id_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY launch
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
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
