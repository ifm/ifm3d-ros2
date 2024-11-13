// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_
#define IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_

#include <ifm3d/fg.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/deserialize/struct_calibration.hpp>

#include <ifm3d_ros2/msg/extrinsics.hpp>
#include <ifm3d_ros2/msg/intrinsics.hpp>
#include <ifm3d_ros2/msg/inverse_intrinsics.hpp>
#include <ifm3d_ros2/msg/rgb_info.hpp>
#include <ifm3d_ros2/msg/tof_info.hpp>
#include <ifm3d_ros2/msg/zones.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>

using TimePointT = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

namespace ifm3d_ros2
{
sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer& image, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer& image,
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format,
                                                                const rclcpp::Logger& logger);
sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer&& image,
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format,
                                                                const rclcpp::Logger& logger);
sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer& image, const std_msgs::msg::Header& header,
                                                 const rclcpp::Logger& logger);
sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                                 const rclcpp::Logger& logger);
nav_msgs::msg::MapMetaData ifm3d_to_ros_map_meta_data(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                      const rclcpp::Logger& logger);
nav_msgs::msg::MapMetaData ifm3d_to_ros_map_meta_data(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                      const rclcpp::Logger& logger);
nav_msgs::msg::OccupancyGrid ifm3d_to_ros_occupancy_grid(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                         const rclcpp::Logger& logger);
nav_msgs::msg::OccupancyGrid ifm3d_to_ros_occupancy_grid(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                         const rclcpp::Logger& logger);
ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger);
ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger);
bool ifm3d_rgb_info_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg);
bool ifm3d_rgb_info_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg);
bool ifm3d_tof_info_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg);
bool ifm3d_tof_info_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg);
sensor_msgs::msg::CameraInfo ifm3d_to_camera_info(ifm3d::calibration::IntrinsicCalibration intrinsic,
                                                  const std_msgs::msg::Header& header, const uint32_t height,
                                                  const uint32_t width, const rclcpp::Logger& logger);
ifm3d_ros2::msg::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger);
ifm3d_ros2::msg::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger);
ifm3d_ros2::msg::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer& buffer,
                                                               const std_msgs::msg::Header& header,
                                                               const rclcpp::Logger& logger);
ifm3d_ros2::msg::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer&& buffer,
                                                               const std_msgs::msg::Header& header,
                                                               const rclcpp::Logger& logger);
ifm3d_ros2::msg::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
ifm3d_ros2::msg::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
ifm3d_ros2::msg::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
ifm3d_ros2::msg::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger);
nav2_msgs::msg::Costmap ifm3d_to_ros_costmap(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                             const rclcpp::Logger& logger);
nav2_msgs::msg::Costmap ifm3d_to_ros_costmap(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                             const rclcpp::Logger& logger);
rclcpp::Time ifm3d_to_ros_time(const TimePointT& time_point);
geometry_msgs::msg::TransformStamped trans_rot_to_optical_mount_link(std::vector<double> trans_rot,
                                                                     std::uint64_t timestamp,
                                                                     std::string mounting_frame_name,
                                                                     std::string optical_frame_name);
bool ifm3d_extrinsic_opt_to_user_to_optical_mount_link(ifm3d::calibration::ExtrinsicOpticToUser opt_to_user,
                                                       std::uint64_t timestamp, std::string mounting_frame_name,
                                                       std::string optical_frame_name, const rclcpp::Logger& logger,
                                                       geometry_msgs::msg::TransformStamped& tf);
bool ifm3d_rgb_info_to_optical_mount_link(ifm3d::Buffer& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf);
bool ifm3d_rgb_info_to_optical_mount_link(ifm3d::Buffer&& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf);
bool ifm3d_tof_info_to_optical_mount_link(ifm3d::Buffer& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf);
bool ifm3d_tof_info_to_optical_mount_link(ifm3d::Buffer&& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf);
}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_
