// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg.h>
#include <ifm3d/deserialize.h>
#include <ifm3d/deserialize/struct_calibration.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>
#include <ifm3d/deserialize/struct_tof_info_v4.hpp>

#include <ifm3d_ros2/msg/extrinsics.hpp>
#include <ifm3d_ros2/msg/intrinsics.hpp>
#include <ifm3d_ros2/msg/inverse_intrinsics.hpp>
#include <ifm3d_ros2/msg/rgb_info.hpp>
#include <ifm3d_ros2/msg/tof_info.hpp>
#include <ifm3d_ros2/msg/zones.hpp>
#include <ifm3d_ros2/buffer_conversions.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace ifm3d_ros2
{
sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                  // image.end() don't have const overloads.
                                           const std_msgs::msg::Header& header, const rclcpp::Logger& logger)
{
  static constexpr auto max_pixel_format = static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32F3);
  static constexpr auto image_format_info = [] {
    auto image_format_info = std::array<const char*, max_pixel_format + 1>{};

    {
      using namespace ifm3d;
      using namespace sensor_msgs::image_encodings;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8U)] = TYPE_8UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8S)] = TYPE_8SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U)] = TYPE_16UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16S)] = TYPE_16SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32U)] = "32UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32S)] = TYPE_32SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F)] = TYPE_32FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64U)] = "64UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64F)] = TYPE_64FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U2)] = TYPE_16UC2;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F3)] = TYPE_32FC3;
    }

    return image_format_info;
  }();

  const auto format = static_cast<std::size_t>(image.dataFormat());

  sensor_msgs::msg::Image result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = 0;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (format >= max_pixel_format)
  {
    RCLCPP_ERROR(logger, "Pixel format out of range (%ld >= %ld)", format, max_pixel_format);
    return result;
  }

  result.encoding = image_format_info.at(format);
  result.step = result.width * sensor_msgs::image_encodings::bitDepth(image_format_info.at(format)) / 8;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.step * result.height));

  if (result.encoding.empty())
  {
    RCLCPP_WARN(logger, "Can't handle encoding %ld (32U == %ld, 64U == %ld)", format,
                static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32U),
                static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_64U));
    result.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  }

  return result;
}

sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_image(image, header, logger);
}

sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer& image,  // Need non-const image because
                                                                                       // image.begin(), image.end()
                                                                                       // don't have const overloads.
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format,  // "jpeg" or "png"
                                                                const rclcpp::Logger& logger)
{
  sensor_msgs::msg::CompressedImage result{};
  result.header = header;
  result.format = format;

  if (const auto dataFormat = image.dataFormat();
      dataFormat != ifm3d::pixel_format::FORMAT_8S && dataFormat != ifm3d::pixel_format::FORMAT_8U)
  {
    RCLCPP_ERROR(logger, "Invalid data format for %s data (%ld)", format.c_str(), static_cast<std::size_t>(dataFormat));
    return result;
  }

  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), image.width() * image.height()));
  return result;
}

sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer&& image,
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format, const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_compressed_image(image, header, format, logger);
}

sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                        // image.end() don't have const overloads.
                                                 const std_msgs::msg::Header& header, const rclcpp::Logger& logger)
{
  sensor_msgs::msg::PointCloud2 result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = false;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (image.dataFormat() != ifm3d::pixel_format::FORMAT_32F3 && image.dataFormat() != ifm3d::pixel_format::FORMAT_32F)
  {
    RCLCPP_ERROR(logger, "Unsupported pixel format %ld for point cloud", static_cast<std::size_t>(image.dataFormat()));
    return result;
  }

  sensor_msgs::msg::PointField x_field{};
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::msg::PointField y_field{};
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  sensor_msgs::msg::PointField z_field{};
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  result.fields = {
    x_field,
    y_field,
    z_field,
  };

  result.point_step = result.fields.size() * sizeof(float);
  result.row_step = result.point_step * result.width;
  result.is_dense = true;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.row_step * result.height));

  return result;
}

sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                                 const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_cloud(image, header, logger);
}

nav_msgs::msg::MapMetaData ifm3d_to_ros_map_meta_data(ifm3d::ODSOccupancyGridV1 grid, const rclcpp::Logger& logger)
{
  // values of matrix 2x3 affine mapping between grid cell and user coordinate system
  // e.g, multiplying the matrix with [0,0,1] gives the user coordinate of the center of upper left cell
  const std::array<float, 6> mapping_matrix = grid.transform_cell_center_to_user;
  RCLCPP_ERROR_EXPRESSION(logger, mapping_matrix[0] != mapping_matrix[4],
                          "Cells of the received ODSOccupancyGrid are not quadratic!");

  nav_msgs::msg::MapMetaData map_meta_data;
  map_meta_data.map_load_time = rclcpp::Time(grid.timestamp_ns);
  map_meta_data.resolution = mapping_matrix[0];
  map_meta_data.width = grid.width;
  map_meta_data.height = grid.height;

  // The origin of the costmap [m, m, rad].
  // This is the real-world pose of the cell (0,0) in the map.
  map_meta_data.origin.position.x = mapping_matrix[2];
  map_meta_data.origin.position.y = mapping_matrix[5];
  map_meta_data.origin.position.z = 0.0;
  map_meta_data.origin.orientation.x = 0;  // we assume no rotation is present
  map_meta_data.origin.orientation.y = 0;
  map_meta_data.origin.orientation.z = 0;
  map_meta_data.origin.orientation.w = 1;

  return map_meta_data;
}

nav_msgs::msg::OccupancyGrid ifm3d_to_ros_occupancy_grid(ifm3d::Buffer& image, const std_msgs::msg::Header& header,
                                                         const rclcpp::Logger& logger)
{
  RCLCPP_DEBUG(logger, "Deserializing occupancy grid data");
  auto occupancy_grid_data = ifm3d::ODSOccupancyGridV1::Deserialize(image);
  nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
  occupancy_grid_msg.header = header;
  occupancy_grid_msg.header.stamp = rclcpp::Time(occupancy_grid_data.timestamp_ns);
  occupancy_grid_msg.info = ifm3d_to_ros_map_meta_data(occupancy_grid_data, logger);
  // Allocate a single contiguous block of memory for the 2D array
  std::vector<int8_t> data(occupancy_grid_data.width * occupancy_grid_data.height);

  for (u_int32_t i = 0; i < occupancy_grid_data.width; i++)
  {
    for (u_int32_t j = 0; j < occupancy_grid_data.height; j++)
    {
      // Get the value from the image
      uint8_t value = occupancy_grid_data.image.at<uint8_t>(i, j);

      // Scale the value to the range 0-100
      int8_t scaled_value = static_cast<int8_t>((value * 100) / 255);

      // Assume values at or below 20 to be free, set their value to zero
      scaled_value = (scaled_value <= 20) ? 0 : scaled_value;

      data[i * occupancy_grid_data.height + j] = scaled_value;
    }
  }
  occupancy_grid_msg.data = data;
  return occupancy_grid_msg;
}

nav_msgs::msg::OccupancyGrid ifm3d_to_ros_occupancy_grid(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                                         const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_occupancy_grid(image, header, logger);
}

nav2_msgs::msg::Costmap ifm3d_to_ros_costmap(ifm3d::Buffer& image, const std_msgs::msg::Header& header,
                                             const rclcpp::Logger& logger)
{
  RCLCPP_DEBUG(logger, "Generating costmap...");
  auto occupancy_grid_data = ifm3d::ODSOccupancyGridV1::Deserialize(image);
  nav2_msgs::msg::Costmap costmap_msg;
  costmap_msg.header = header;
  costmap_msg.header.stamp = rclcpp::Time(occupancy_grid_data.timestamp_ns);
  RCLCPP_ERROR_EXPRESSION(logger, costmap_msg.header.frame_id.empty(),
                          "frame_id is not set in the header provided to ifm3d_to_ros_costmap()!");

  costmap_msg.metadata.update_time = costmap_msg.header.stamp;
  costmap_msg.metadata.layer = "ods";

  // Use MapMetaData to populate Costmap metadata
  const nav_msgs::msg::MapMetaData meta_data = ifm3d_to_ros_map_meta_data(occupancy_grid_data, logger);
  costmap_msg.metadata.resolution = meta_data.resolution;
  costmap_msg.metadata.size_x = meta_data.width;
  costmap_msg.metadata.size_y = meta_data.height;
  costmap_msg.metadata.origin = meta_data.origin;

  // Allocate a single contiguous block of memory for the 2D array
  std::vector<uint8_t> data(occupancy_grid_data.width * occupancy_grid_data.height);

  for (uint32_t i = 0; i < occupancy_grid_data.width; i++)
  {
    for (uint32_t j = 0; j < occupancy_grid_data.height; j++)
    {
      // Get the value from the image
      uint8_t value = occupancy_grid_data.image.at<uint8_t>(i, j);
      // Scale the value to the range 0-100
      int8_t scaled_value = static_cast<int8_t>((value * 100) / 255);
      // data[i * occupancy_grid_data.height + j] = occupancy_grid_data.image.at<uint8_t>(i, occupancy_grid_data.height
      // - 1 - j);
      data[i * occupancy_grid_data.height + j] = scaled_value;
    }
  }
  costmap_msg.data = data;

  RCLCPP_DEBUG(logger, "Costmap generated.");
  return costmap_msg;
}

nav2_msgs::msg::Costmap ifm3d_to_ros_costmap(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                             const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_costmap(image, header, logger);
}

ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::Extrinsics extrinsics_msg;
  extrinsics_msg.header = header;
  try
  {
    extrinsics_msg.tx = buffer.at<double>(0);
    extrinsics_msg.ty = buffer.at<double>(1);
    extrinsics_msg.tz = buffer.at<double>(2);
    extrinsics_msg.rot_x = buffer.at<double>(3);
    extrinsics_msg.rot_y = buffer.at<double>(4);
    extrinsics_msg.rot_z = buffer.at<double>(5);
  }
  catch (const std::out_of_range& ex)
  {
    RCLCPP_WARN(logger, "Out-of-range error fetching extrinsics");
  }
  return extrinsics_msg;
}

ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)
{
  return ifm3d_to_extrinsics(buffer, header, logger);
}

bool ifm3d_rgb_info_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  camera_info_msg.header = header;

  try
  {
    auto rgb_info = ifm3d::RGBInfoV1::Deserialize(buffer);
    camera_info_msg = ifm3d_to_camera_info(rgb_info.intrinsic_calibration, header, height, width, logger);
    return true;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read rgb info.");
    return false;
  }
}

bool ifm3d_rgb_info_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  return ifm3d_rgb_info_to_camera_info(buffer, header, height, width, logger, camera_info_msg);
}

bool ifm3d_tof_info_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  camera_info_msg.header = header;

  try
  {
    auto tof_info = ifm3d::TOFInfoV4::Deserialize(buffer);
    camera_info_msg = ifm3d_to_camera_info(tof_info.intrinsic_calibration, header, height, width, logger);
    return true;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read rgb info.");
    return false;
  }
}

bool ifm3d_tof_info_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header, const uint32_t height,
                                   const uint32_t width, const rclcpp::Logger& logger,
                                   sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  return ifm3d_tof_info_to_camera_info(buffer, header, height, width, logger, camera_info_msg);
}

sensor_msgs::msg::CameraInfo ifm3d_to_camera_info(ifm3d::calibration::IntrinsicCalibration intrinsic,
                                                  const std_msgs::msg::Header& header, const uint32_t height,
                                                  const uint32_t width, const rclcpp::Logger& logger)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header = header;

  try
  {
    camera_info_msg.height = height;
    camera_info_msg.width = width;
    if (intrinsic.model_id == 2) // This is a fish eye lens
    {
      // Fill in the message with fish eye model params
      camera_info_msg.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;

      // Read data from buffer
      const float fx = intrinsic.model_parameters[0];
      const float fy = intrinsic.model_parameters[1];
      
      const float mx = intrinsic.model_parameters[2];
      const float my = intrinsic.model_parameters[3];
      const float alpha = intrinsic.model_parameters[4];
      const float k1 = intrinsic.model_parameters[5];
      const float k2 = intrinsic.model_parameters[6];
      const float k3 = intrinsic.model_parameters[7];
      const float k4 = intrinsic.model_parameters[8];
      const float theta_max = intrinsic.model_parameters[9];

      const float ix = width - 1;
      const float iy = height - 1;
      const float cy = (iy + 0.5 - my) / fy;
      const float cx = (ix + 0.5 - mx) / fx - alpha * cy;
      const float r2 = cx * cx + cy * cy;
      const float h = 2 * cx * cy;
      const float tx = k3 * h + k4 * (r2 + 2 * cx * cx);
      const float ty = k3 * (r2 + 2 * cy * cy) + k4 * h;

      // Distortion parameters
      camera_info_msg.d.resize(5);
      camera_info_msg.d[0] = k1;
      camera_info_msg.d[1] = k2;
      camera_info_msg.d[2] = tx;  // TODO t1 == tx ?
      camera_info_msg.d[3] = ty;  // TODO t2 == ty ?
      camera_info_msg.d[4] = k3;

      // Intrinsic camera matrix, in row-major order
      //     [ fx 0  cx]
      // K = [ 0  fy cy]
      //     [ 0  0  1 ]
      camera_info_msg.k[0] = fx;
      camera_info_msg.k[2] = cx;
      camera_info_msg.k[4] = fy;
      camera_info_msg.k[5] = cy;
      camera_info_msg.k[8] = 1.0;  // fixed to 1.0

      // Projection matrix, row-major
      //     [fx' 0   cx' Tx]
      // P = [ 0  fy' cy' Ty]
      //     [ 0  0   1   0 ]
      camera_info_msg.p[0] = fx;
      camera_info_msg.p[5] = fy;
      camera_info_msg.p[2] = cx;
      camera_info_msg.p[6] = cy;
      camera_info_msg.p[10] = 1.0;  // fixed to 1.0

      RCLCPP_DEBUG_ONCE(logger,
                        "Intrinsics:\nfx=%f \nfy=%f \nmx=%f \nmy=%f \nalpha=%f \nk1=%f \nk2=%f \nk3=%f \nk4=%f "
                        "\nCalculated:\nix=%f \niy=%f \ncx=%f \ncy=%f \nr2=%f \nh=%f \ntx=%f \nty=%f",
                        fx, fy, mx, my, alpha, k1, k2, k3, k4, ix, iy, cx, cy, r2, h, tx, ty);
    }
    else if (intrinsic.model_id == 0) 
    {    
      camera_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

      // Read data from buffer
      const float fx = intrinsic.model_parameters[0];
      const float fy = intrinsic.model_parameters[1];

      const float mx = intrinsic.model_parameters[2];
      const float my = intrinsic.model_parameters[3];
      const float alpha = intrinsic.model_parameters[4];
      const float k1 = intrinsic.model_parameters[5];
      const float k2 = intrinsic.model_parameters[6];
      const float k3 = intrinsic.model_parameters[7];
      const float k4 = intrinsic.model_parameters[8];
      const float k5 = intrinsic.model_parameters[9];

      const float ix = width - 1;
      const float iy = height - 1;
      const float cy = (iy + 0.5 - my) / fy;
      const float cx = (ix + 0.5 - mx) / fx - alpha * cy;
      const float r2 = cx * cx + cy * cy;
      const float h = 2 * cx * cy;
      const float tx = k3 * h + k4 * (r2 + 2 * cx * cx);
      const float ty = k3 * (r2 + 2 * cy * cy) + k4 * h;

      // Distortion parameters
      camera_info_msg.d.resize(5);
      camera_info_msg.d[0] = k1;
      camera_info_msg.d[1] = k2;
      camera_info_msg.d[2] = tx;  // TODO t1 == tx ?
      camera_info_msg.d[3] = ty;  // TODO t2 == ty ?
      camera_info_msg.d[4] = k3;

      // Intrinsic camera matrix
      camera_info_msg.k[0] = fx;
      camera_info_msg.k[4] = fy;
      camera_info_msg.k[2] = cx;
      camera_info_msg.k[5] = cy;
      camera_info_msg.k[8] = 1.0;  // fixed to 1.0

      // Projection matrix
      camera_info_msg.p[0] = fx;
      camera_info_msg.p[5] = fy;
      camera_info_msg.p[2] = cx;
      camera_info_msg.p[6] = cy;
      camera_info_msg.p[10] = 1.0;  // fixed to 1.0

      RCLCPP_DEBUG_ONCE(logger,
                        "Intrinsics:\nfx=%f \nfy=%f \nmx=%f \nmy=%f \nalpha=%f \nk1=%f \nk2=%f \nk3=%f \nk4=%f "
                        "\nCalculated:\nix=%f \niy=%f \ncx=%f \ncy=%f \nr2=%f \nh=%f \ntx=%f \nty=%f",
                        fx, fy, mx, my, alpha, k1, k2, k3, k4, ix, iy, cx, cy, r2, h, tx, ty);
    }
    else{
      RCLCPP_ERROR(logger, "Unknown intrinsic calibration model");
    }
  }
  catch (const std::out_of_range& ex)
  {
    RCLCPP_WARN(logger, "Out-of-range error fetching intrinsics");
  }
  return camera_info_msg;
}

ifm3d_ros2::msg::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::Intrinsics intrinsics_msg;
  intrinsics_msg.header = header;

  ifm3d::calibration::IntrinsicCalibration intrinsics;

  try
  {
    intrinsics.Read(buffer.ptr<uint8_t>(0));
    intrinsics_msg.model_id = intrinsics.model_id;
    intrinsics_msg.model_parameters = intrinsics.model_parameters;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read intrinsics.");
  }

  return intrinsics_msg;
}

ifm3d_ros2::msg::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)

{
  return ifm3d_to_intrinsics(buffer, header, logger);
}

ifm3d_ros2::msg::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer& buffer,
                                                               const std_msgs::msg::Header& header,
                                                               const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::InverseIntrinsics inverse_intrinsics_msg;
  inverse_intrinsics_msg.header = header;

  ifm3d::calibration::InverseIntrinsicCalibration inverse_intrinsics;

  try
  {
    inverse_intrinsics.Read(buffer.ptr<uint8_t>(0));
    inverse_intrinsics_msg.model_id = inverse_intrinsics.model_id;
    inverse_intrinsics_msg.model_parameters = inverse_intrinsics.model_parameters;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read inverse intrinsics.");
  }

  return inverse_intrinsics_msg;
}

ifm3d_ros2::msg::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer&& buffer,
                                                               const std_msgs::msg::Header& header,
                                                               const rclcpp::Logger& logger)

{
  return ifm3d_to_inverse_intrinsics(buffer, header, logger);
}

ifm3d_ros2::msg::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::RGBInfo rgb_info_msg;
  rgb_info_msg.header = header;

  try
  {
    auto rgb_info = ifm3d::RGBInfoV1::Deserialize(buffer);

    rgb_info_msg.version = rgb_info.version;
    rgb_info_msg.frame_counter = rgb_info.frame_counter;
    rgb_info_msg.timestamp_ns = rgb_info.timestamp_ns;
    rgb_info_msg.exposure_time = rgb_info.exposure_time;

    rgb_info_msg.extrinsics.header = header;
    rgb_info_msg.extrinsics.tx = rgb_info.extrinsic_optic_to_user.trans_x;
    rgb_info_msg.extrinsics.ty = rgb_info.extrinsic_optic_to_user.trans_y;
    rgb_info_msg.extrinsics.tz = rgb_info.extrinsic_optic_to_user.trans_z;
    rgb_info_msg.extrinsics.rot_x = rgb_info.extrinsic_optic_to_user.rot_x;
    rgb_info_msg.extrinsics.rot_y = rgb_info.extrinsic_optic_to_user.rot_y;
    rgb_info_msg.extrinsics.rot_z = rgb_info.extrinsic_optic_to_user.rot_z;

    rgb_info_msg.intrinsics.header = header;
    rgb_info_msg.intrinsics.model_id = rgb_info.intrinsic_calibration.model_id;
    rgb_info_msg.intrinsics.model_parameters = rgb_info.intrinsic_calibration.model_parameters;

    rgb_info_msg.inverse_intrinsics.header = header;
    rgb_info_msg.inverse_intrinsics.model_id = rgb_info.inverse_intrinsic_calibration.model_id;
    rgb_info_msg.inverse_intrinsics.model_parameters = rgb_info.inverse_intrinsic_calibration.model_parameters;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read rgb info.");
  }

  return rgb_info_msg;
}

ifm3d_ros2::msg::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)

{
  return ifm3d_to_rgb_info(buffer, header, logger);
}

ifm3d_ros2::msg::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::TOFInfo tof_info_msg;
  tof_info_msg.header = header;

  try
  {
    auto tof_info = ifm3d::TOFInfoV4::Deserialize(buffer);
    tof_info_msg.measurement_block_index = tof_info.measurement_block_index;
    tof_info_msg.measurement_range_min = tof_info.measurement_range_min;
    tof_info_msg.measurement_range_max = tof_info.measurement_range_max;
    tof_info_msg.version = tof_info.version;
    tof_info_msg.distance_resolution = tof_info.distance_resolution;
    tof_info_msg.amplitude_resolution = tof_info.amplitude_resolution;
    tof_info_msg.amp_normalization_factors = tof_info.amp_normalization_factors;
    tof_info_msg.exposure_timestamps_ns = tof_info.exposure_timestamps_ns;
    tof_info_msg.exposure_times_s = tof_info.exposure_times_s;
    tof_info_msg.illu_temperature = tof_info.illu_temperature;
    tof_info_msg.mode = std::string(std::begin(tof_info.mode), std::end(tof_info.mode));
    tof_info_msg.imager = std::string(std::begin(tof_info.imager), std::end(tof_info.imager));

    tof_info_msg.extrinsics.header = header;
    tof_info_msg.extrinsics.tx = tof_info.extrinsic_optic_to_user.trans_x;
    tof_info_msg.extrinsics.ty = tof_info.extrinsic_optic_to_user.trans_y;
    tof_info_msg.extrinsics.tz = tof_info.extrinsic_optic_to_user.trans_z;
    tof_info_msg.extrinsics.rot_x = tof_info.extrinsic_optic_to_user.rot_x;
    tof_info_msg.extrinsics.rot_y = tof_info.extrinsic_optic_to_user.rot_y;
    tof_info_msg.extrinsics.rot_z = tof_info.extrinsic_optic_to_user.rot_z;

    tof_info_msg.intrinsics.header = header;
    tof_info_msg.intrinsics.model_id = tof_info.intrinsic_calibration.model_id;
    tof_info_msg.intrinsics.model_parameters = tof_info.intrinsic_calibration.model_parameters;

    tof_info_msg.inverse_intrinsics.header = header;
    tof_info_msg.inverse_intrinsics.model_id = tof_info.inverse_intrinsic_calibration.model_id;
    tof_info_msg.inverse_intrinsics.model_parameters = tof_info.inverse_intrinsic_calibration.model_parameters;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read tof info.");
  }

  return tof_info_msg;
}

ifm3d_ros2::msg::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)

{
  return ifm3d_to_tof_info(buffer, header, logger);
}

rclcpp::Time ifm3d_to_ros_time(const TimePointT& time_point)
{
  auto duration = time_point.time_since_epoch();
  int64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  return rclcpp::Time(nanoseconds, RCL_SYSTEM_TIME);
}

geometry_msgs::msg::TransformStamped trans_rot_to_optical_mount_link(std::vector<double> trans_rot,
                                                                     std::uint64_t timestamp,
                                                                     std::string mounting_frame_name,
                                                                     std::string optical_frame_name)
{
  /*
   * tf2::Quaternion assumes extrinsic euler angles as roll pitch yaw for setRPY(),
   * meaning all three rotations happen in relation to a fixed coordinate system.
   * But the RPY angles from ifm3d are performing an intrinsic euler rotation,
   * the reference coordinate system is updated after each individual rotation.
   * Therefore, we split the received rotation into three different quaternions
   * and perform the rotations in R->P->Y order.
   */
  tf2::Quaternion q_roll, q_pitch, q_yaw, q_combined;
  q_roll.setRPY(trans_rot[3], 0.0, 0.0);
  q_pitch.setRPY(0.0, trans_rot[4], 0.0);
  q_yaw.setRPY(0.0, 0.0, trans_rot[5]);
  q_combined = q_roll * q_pitch * q_yaw;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Time(timestamp);
  t.header.frame_id = mounting_frame_name;
  t.child_frame_id = optical_frame_name;
  t.transform.translation.x = trans_rot[0];
  t.transform.translation.y = trans_rot[1];
  t.transform.translation.z = trans_rot[2];
  t.transform.rotation.x = q_combined.x();
  t.transform.rotation.y = q_combined.y();
  t.transform.rotation.z = q_combined.z();
  t.transform.rotation.w = q_combined.w();
  return t;
}

bool ifm3d_extrinsic_opt_to_user_to_optical_mount_link(ifm3d::calibration::ExtrinsicOpticToUser opt_to_user,
                                                       std::uint64_t timestamp, std::string mounting_frame_name,
                                                       std::string optical_frame_name, const rclcpp::Logger& logger,
                                                       geometry_msgs::msg::TransformStamped& tf)
{
  try
  {
    std::vector<double> trans_rot = { opt_to_user.trans_x, opt_to_user.trans_y, opt_to_user.trans_z,
                                      opt_to_user.rot_x,   opt_to_user.rot_y,   opt_to_user.rot_z };
    tf = trans_rot_to_optical_mount_link(trans_rot, timestamp, mounting_frame_name, optical_frame_name);
    return true;
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read tof info.");
    return false;
  }
}

bool ifm3d_rgb_info_to_optical_mount_link(ifm3d::Buffer& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf)
{
  try
  {
    auto rgb_info = ifm3d::RGBInfoV1::Deserialize(buffer);
    return ifm3d_extrinsic_opt_to_user_to_optical_mount_link(rgb_info.extrinsic_optic_to_user, rgb_info.timestamp_ns,
                                                             mounting_frame_name, optical_frame_name, logger, tf);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read rgb info.");
    return false;
  }
}

bool ifm3d_rgb_info_to_optical_mount_link(ifm3d::Buffer&& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf)
{
  return ifm3d_rgb_info_to_optical_mount_link(buffer, mounting_frame_name, optical_frame_name, logger, tf);
}

bool ifm3d_tof_info_to_optical_mount_link(ifm3d::Buffer& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf)
{
  try
  {
    auto tof_info = ifm3d::TOFInfoV4::Deserialize(buffer);
    return ifm3d_extrinsic_opt_to_user_to_optical_mount_link(tof_info.extrinsic_optic_to_user,
                                                             tof_info.exposure_timestamps_ns[0], mounting_frame_name,
                                                             optical_frame_name, logger, tf);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger, "Failed to read tof info.");
    return false;
  }
}

bool ifm3d_tof_info_to_optical_mount_link(ifm3d::Buffer&& buffer, std::string mounting_frame_name,
                                          std::string optical_frame_name, const rclcpp::Logger& logger,
                                          geometry_msgs::msg::TransformStamped& tf)
{
  return ifm3d_tof_info_to_optical_mount_link(buffer, mounting_frame_name, optical_frame_name, logger, tf);
}

}  // namespace ifm3d_ros2
