// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_
#define IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_

#include <ifm3d/fg.h>
#include <ifm3d_ros2/msg/extrinsics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

sensor_msgs::msg::CameraInfo ifm3d_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                  const uint32_t height, const uint32_t width,
                                                  const rclcpp::Logger& logger)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header = header;

  try
  {
    camera_info_msg.height = height;
    camera_info_msg.width = width;
    camera_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;  // TODO

    // Read data from buffer
    const double fx = buffer.at<double>(0);
    const double fy = buffer.at<double>(1);
    const double mx = buffer.at<double>(2);
    const double my = buffer.at<double>(3);
    const double alpha = buffer.at<double>(4);
    const double k1 = buffer.at<double>(5);
    const double k2 = buffer.at<double>(6);
    const double k3 = buffer.at<double>(7);
    const double k4 = buffer.at<double>(8);
    // next in buffer is k5 for bouguet or theta_max for fisheye model, both not needed here

    const double ix = width - 1;
    const double iy = height - 1;
    const double cy = (iy + 0.5 - my) / fy;
    const double cx = (ix + 0.5 - mx) / fx - alpha * cy;
    const double r2 = cx * cx + cy * cy;
    const double h = 2 * cx * cy;
    const double tx = k3 * h + k4 * (r2 + 2 * cx * cx);
    const double ty = k3 * (r2 + 2 * cy * cy) + k4 * h;

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

    RCLCPP_INFO_ONCE(logger,
                     "Intrinsics:\nfx=%f \nfy=%f \nmx=%f \nmy=%f \nalpha=%f \nk1=%f \nk2=%f \nk3=%f \nk4=%f "
                     "\nCalculated:\nix=%f \niy=%f \ncx=%f \ncy=%f \nr2=%f \nh=%f \ntx=%f \nty=%f",
                     fx, fy, mx, my, alpha, k1, k2, k3, k4, ix, iy, cx, cy, r2, h, tx, ty);
  }
  catch (const std::out_of_range& ex)
  {
    RCLCPP_WARN(logger, "Out-of-range error fetching intrinsics");
  }

  return camera_info_msg;
}

sensor_msgs::msg::CameraInfo ifm3d_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                  const uint32_t height, const uint32_t width,
                                                  const rclcpp::Logger& logger)

{
  return ifm3d_to_camera_info(buffer, header, height, width, logger);
}

}  // namespace ifm3d_ros2

#endif