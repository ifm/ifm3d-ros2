// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_polar_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_extrinsic_calibration_correction_v1.hpp>

#include <cmath>
#include <limits>

#include <ifm3d_ros2/ods_module.hpp>
#include <ifm3d_ros2/buffer_conversions.hpp>
#include <ifm3d_ros2/buffer_id_utils.hpp>
#include <ifm3d_ros2/qos.hpp>

namespace ifm3d_ros2
{
OdsModule::OdsModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr)
  : FunctionModule(logger), node_ptr_(node_ptr), frame_id_("ifm_base_link")
{
  RCLCPP_INFO(logger_, "OdsModule contructor called.");

  RCLCPP_DEBUG(logger_, "Declaring parameter...");
  frame_id_descriptor_.name = "ods.frame_id";
  frame_id_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  frame_id_descriptor_.description = "Frame_id field used for grid and zones messages (Default='ifm_base_link').";
  if (!node_ptr_->has_parameter(frame_id_descriptor_.name))
  {
    node_ptr_->declare_parameter(frame_id_descriptor_.name, frame_id_, frame_id_descriptor_);
  }

  publish_occupancy_grid_descriptor_.name = "ods.publish_occupancy_grid";
  publish_occupancy_grid_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_occupancy_grid_descriptor_.description = "Set module to publish nav_msgs/OccupancyGrid (Default='True').";
  if (!node_ptr_->has_parameter(publish_occupancy_grid_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_occupancy_grid_descriptor_.name, true, publish_occupancy_grid_descriptor_);
  }

  publish_polar_occupancy_grid_descriptor_.name = "ods.publish_polar_occupancy_grid";
  publish_polar_occupancy_grid_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_polar_occupancy_grid_descriptor_.description = "Set module to publish sensor_msgs/LaserScan from polar distance data (Default='True').";
  if (!node_ptr_->has_parameter(publish_polar_occupancy_grid_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_polar_occupancy_grid_descriptor_.name, true,
                                 publish_polar_occupancy_grid_descriptor_);
  }

  publish_extrinsics_calibration_correction_descriptor_.name = "ods.publish_extrinsics_calibration_correction";
  publish_extrinsics_calibration_correction_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_extrinsics_calibration_correction_descriptor_.description = "Set module to publish Extrinsics calibration correction values (Default='True').";
  if (!node_ptr_->has_parameter(publish_extrinsics_calibration_correction_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_extrinsics_calibration_correction_descriptor_.name, true,
                                 publish_extrinsics_calibration_correction_descriptor_);
  }

  publish_costmap_descriptor_.name = "ods.publish_costmap";
  publish_costmap_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_costmap_descriptor_.description = "Set module to publish nav3_msgs/Costmap (Default='False').";
  if (!node_ptr_->has_parameter(publish_costmap_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_costmap_descriptor_.name, false, publish_costmap_descriptor_);
  }
}

nav_msgs::msg::OccupancyGrid OdsModule::extract_ros_occupancy_grid(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handling ods occupancy grid as nav_msgs/OccupancyGrid");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID))
  {
    RCLCPP_INFO(logger_, "OdsModule: No ods occupancy grid in frame");
  }
  auto header = std_msgs::msg::Header();
  header.frame_id = frame_id_;
  return ifm3d_ros2::ifm3d_to_ros_occupancy_grid(frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID), header,
                                                 logger_);
}

nav2_msgs::msg::Costmap OdsModule::extract_ros_costmap(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handling ods occupancy grid as nav2_msgs/Costmap");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID))
  {
    RCLCPP_INFO(logger_, "OdsModule: No ods occupancy grid in frame");
  }
  auto header = std_msgs::msg::Header();
  header.frame_id = frame_id_;
  return ifm3d_ros2::ifm3d_to_ros_costmap(frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID), header, logger_);
}

ifm3d_ros2::msg::Zones OdsModule::extract_zones(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handling zones");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_ODS_INFO))
  {
    RCLCPP_INFO(logger_, "OdsModule: No zones in frame");
  }
  RCLCPP_DEBUG(logger_, "Deserializing zones data");
  auto zones_data = ifm3d::ODSInfoV1::Deserialize(frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_INFO));

  RCLCPP_DEBUG(logger_, "Filling the ROS message with zones data");
  ifm3d_ros2::msg::Zones zones_msg;
  // Define the header
  zones_msg.header = std_msgs::msg::Header();
  zones_msg.header.frame_id = frame_id_;
  zones_msg.header.stamp = rclcpp::Time(zones_data.timestamp_ns);

  zones_msg.zone_config_id = zones_data.zone_config_id;
  zones_msg.zone_occupied = zones_data.zone_occupied;

  RCLCPP_DEBUG(logger_, "Zones messages ready");
  return zones_msg;
}

sensor_msgs::msg::LaserScan OdsModule::extract_ros_polar_occupancy_grid(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Converting polar distance data to LaserScan");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_ODS_POLAR_OCC_GRID))
  {
    RCLCPP_INFO(logger_, "OdsModule: No polar distance data in frame for LaserScan");
  }
  RCLCPP_DEBUG(logger_, "Deserializing polar distance data");
  auto polarOccGrid_data = ifm3d::ODSPolarOccupancyGridV1::Deserialize(frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_POLAR_OCC_GRID));

  sensor_msgs::msg::LaserScan laser_scan_msg;

  laser_scan_msg.header = std_msgs::msg::Header();
  laser_scan_msg.header.frame_id = frame_id_;
  laser_scan_msg.header.stamp = rclcpp::Time(polarOccGrid_data.timestamp_ns);

  const auto n = polarOccGrid_data.polarOccGrid.size();
  const double two_pi = 2.0 * std::acos(-1.0);
  const double angle_increment = (n > 0) ? (two_pi / static_cast<double>(n)) : 0.0;

  // Convention: first element is 0° and increases counter-clockwise.
  laser_scan_msg.angle_min = 0.0;
  laser_scan_msg.angle_increment = angle_increment;
  laser_scan_msg.angle_max = (n > 0) ? (laser_scan_msg.angle_min + (static_cast<double>(n) - 1.0) * angle_increment) : 0.0;

  laser_scan_msg.time_increment = 0.0;
  laser_scan_msg.scan_time = 0.0;

  laser_scan_msg.range_min = 0.0;
  // 65535 mm is reserved as "no object" => max valid is 65534 mm.
  laser_scan_msg.range_max = 65.534;

  laser_scan_msg.ranges.resize(n);
  laser_scan_msg.intensities.resize(n);

  for (size_t i = 0; i < n; ++i)
  {
    const uint16_t distance_mm = polarOccGrid_data.polarOccGrid[i];

    if (distance_mm == 65535)
    {
      laser_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      laser_scan_msg.intensities[i] = 0.0f;
    }
    else
    {
      laser_scan_msg.ranges[i] = static_cast<float>(distance_mm) / 1000.0f;
      laser_scan_msg.intensities[i] = 1.0f;
    }
  }

  RCLCPP_DEBUG(logger_, "LaserScan conversion completed");
  return laser_scan_msg;
}

ifm3d_ros2::msg::ExtrinsicsCalibrationCorrection OdsModule::extract_ros_extrinsics_calibration_correction(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handling Polar Occupancy Grid");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION))
  {
    RCLCPP_INFO(logger_, "OdsModule: No extrinsics calibration correction information in frame");
  }
  RCLCPP_DEBUG(logger_, "Deserializing extrinsics calibration correction data");
  auto extrinsicsCalibrationCorrection_data = ifm3d::ODSExtrinsicCalibrationCorrectionV1::Deserialize(frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION));

  RCLCPP_DEBUG(logger_, "Filling the ROS message with extrinsics calibration correction data");
  ifm3d_ros2::msg::ExtrinsicsCalibrationCorrection extrinsicsCalibrationCorrection_msg;
  // Define the header
  extrinsicsCalibrationCorrection_msg.header = std_msgs::msg::Header();
  extrinsicsCalibrationCorrection_msg.header.frame_id = frame_id_;
  
  // No timestamps in deserialized structs explicitly so we use frame timestamps
  std::vector<ifm3d::TimePointT> timestamps = frame->TimeStamps();
  if (!timestamps.empty())
  {
      auto ts = timestamps[0];  // Or whichever index is meaningful
      auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(ts.time_since_epoch()).count();

      // Construct from nanoseconds
      extrinsicsCalibrationCorrection_msg.header.stamp = rclcpp::Time(ns_since_epoch);
  }else
  {
      RCLCPP_WARN(logger_, "Frame timestamps are empty; header.stamp not set");
  }

  // Extrinsics calibration correction data
  extrinsicsCalibrationCorrection_msg.version = extrinsicsCalibrationCorrection_data.version;
  extrinsicsCalibrationCorrection_msg.completion_rate = extrinsicsCalibrationCorrection_data.completion_rate;
  extrinsicsCalibrationCorrection_msg.rot_delta_value = extrinsicsCalibrationCorrection_data.rot_delta_value;
  extrinsicsCalibrationCorrection_msg.rot_delta_valid = extrinsicsCalibrationCorrection_data.rot_delta_valid;
  extrinsicsCalibrationCorrection_msg.rot_head_to_user = extrinsicsCalibrationCorrection_data.rot_head_to_user;

  RCLCPP_DEBUG(logger_, "Extrinsics calibration correction ready");
  return extrinsicsCalibrationCorrection_msg;
}

void OdsModule::handle_frame(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handle frame");

  RCLCPP_DEBUG(logger_, "Creating ods zones message.");
  ZonesMsg zones_msg;
  zones_msg = this->extract_zones(frame);
  zones_publisher_->publish(zones_msg);

  if (publish_occupancy_grid_)
  {
    RCLCPP_DEBUG(logger_, "Creating occupancy grid message.");
    OccupancyGridMsg grid_msg;
    grid_msg = this->extract_ros_occupancy_grid(frame);
    ros_occupancy_grid_publisher_->publish(grid_msg);
  }

  if (publish_polar_occupancy_grid_)
  {
    RCLCPP_DEBUG(logger_, "Creating polar occupancy grid message.");
    PolarOccupancyGridMsg polar_occ_grid_msg;
    polar_occ_grid_msg = this->extract_ros_polar_occupancy_grid(frame);
    ros_polar_occupancy_grid_publisher_->publish(polar_occ_grid_msg);
  }

  if (publish_extrinsics_calibration_correction_)
  {
    RCLCPP_DEBUG(logger_, "Creating extrinsic calibration correction message.");
    ExtrinsicsCalibrationCorrectionMsg extrinsics_calibration_correction_msg;
    extrinsics_calibration_correction_msg = this->extract_ros_extrinsics_calibration_correction(frame);
    ros_extrinsics_calibration_correction_publisher_->publish(extrinsics_calibration_correction_msg);
  }

  if (publish_costmap_)
  {
    RCLCPP_DEBUG(logger_, "Creating costmap message.");
    CostmapMsg costmap_msg;
    costmap_msg = this->extract_ros_costmap(frame);
    ros_costmap_publisher_->publish(costmap_msg);
  }

  RCLCPP_DEBUG(logger_, "Finished publishing ods messages.");
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_configure called");
  (void)previous_state;

  node_ptr_->get_parameter(frame_id_descriptor_.name, frame_id_);
  node_ptr_->get_parameter(publish_occupancy_grid_descriptor_.name, publish_occupancy_grid_);
  node_ptr_->get_parameter(publish_polar_occupancy_grid_descriptor_.name, publish_polar_occupancy_grid_);
  node_ptr_->get_parameter(publish_extrinsics_calibration_correction_descriptor_.name, publish_extrinsics_calibration_correction_);
  node_ptr_->get_parameter(publish_costmap_descriptor_.name, publish_costmap_);
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", frame_id_descriptor_.name.c_str(), frame_id_.c_str());
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_occupancy_grid_descriptor_.name.c_str(),
              publish_occupancy_grid_ ? "true" : "false");
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_polar_occupancy_grid_descriptor_.name.c_str(),
              publish_polar_occupancy_grid_ ? "true" : "false");
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_extrinsics_calibration_correction_descriptor_.name.c_str(),
              publish_extrinsics_calibration_correction_ ? "true" : "false");
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_costmap_descriptor_.name.c_str(),
              publish_costmap_ ? "true" : "false");

  const auto qos = ifm3d_ros2::LowLatencyQoS();

  if (publish_occupancy_grid_)
  {
    ros_occupancy_grid_publisher_ = node_ptr_->create_publisher<OccupancyGridMsg>(
        "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID], qos);
  }
  if (publish_polar_occupancy_grid_)
  {
    ros_polar_occupancy_grid_publisher_ = node_ptr_->create_publisher<PolarOccupancyGridMsg>(
        "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_POLAR_OCC_GRID], qos);
  }
  if (publish_extrinsics_calibration_correction_)
  {
    ros_extrinsics_calibration_correction_publisher_ = node_ptr_->create_publisher<ExtrinsicsCalibrationCorrectionMsg>(
        "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION], qos);
  }
  if (publish_costmap_)
  {
    ros_costmap_publisher_ = node_ptr_->create_publisher<CostmapMsg>(
        "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID] + "_costmap", qos);
  }
  zones_publisher_ = node_ptr_->create_publisher<ZonesMsg>(
      "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_INFO], qos);

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_cleanup called");
  (void)previous_state;

  ros_occupancy_grid_publisher_.reset();
  ros_polar_occupancy_grid_publisher_.reset();
  ros_extrinsics_calibration_correction_publisher_.reset();
  ros_costmap_publisher_.reset();
  zones_publisher_.reset();

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_shutdown called");
  (void)previous_state;
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_activate called");
  (void)previous_state;

  if (publish_occupancy_grid_)
  {
    this->ros_occupancy_grid_publisher_->on_activate();
  }
  if (publish_polar_occupancy_grid_)
  {
    this->ros_polar_occupancy_grid_publisher_->on_activate();
  }
  if (publish_extrinsics_calibration_correction_)
  {
    this->ros_extrinsics_calibration_correction_publisher_->on_activate();
  }
  if (publish_costmap_)
  {
    this->ros_costmap_publisher_->on_activate();
  }
  this->zones_publisher_->on_activate();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_deactivate called");
  (void)previous_state;

  if (publish_occupancy_grid_)
  {
    this->ros_occupancy_grid_publisher_->on_deactivate();
  }
  if (publish_polar_occupancy_grid_)
  {
    this->ros_polar_occupancy_grid_publisher_->on_deactivate();
  }
  if (publish_extrinsics_calibration_correction_)
  {
    this->ros_extrinsics_calibration_correction_publisher_->on_deactivate();
  }
  if (publish_costmap_)
  {
    this->ros_costmap_publisher_->on_deactivate();
  }
  this->zones_publisher_->on_deactivate();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn OdsModule::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "OdsModule: on_error called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}  // namespace ifm3d_ros2