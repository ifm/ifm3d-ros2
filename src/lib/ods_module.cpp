// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>

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
  node_ptr_->declare_parameter(frame_id_descriptor_.name, frame_id_, frame_id_descriptor_);

  publish_occupancy_grid_descriptor_.name = "ods.publish_occupancy_grid";
  publish_occupancy_grid_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_occupancy_grid_descriptor_.description = "Set module to publish nav_msgs/OccupancyGrid (Default='True').";
  node_ptr_->declare_parameter(publish_occupancy_grid_descriptor_.name, true, publish_occupancy_grid_descriptor_);

  publish_costmap_descriptor_.name = "ods.publish_costmap";
  publish_costmap_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_costmap_descriptor_.description = "Set module to publish nav3_msgs/Costmap (Default='False').";
  node_ptr_->declare_parameter(publish_costmap_descriptor_.name, false, publish_costmap_descriptor_);
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
  node_ptr_->get_parameter(publish_costmap_descriptor_.name, publish_costmap_);
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", frame_id_descriptor_.name.c_str(), frame_id_.c_str());
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_occupancy_grid_descriptor_.name.c_str(),
              publish_occupancy_grid_ ? "true" : "false");
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_costmap_descriptor_.name.c_str(),
              publish_costmap_ ? "true" : "false");

  const auto qos = ifm3d_ros2::LowLatencyQoS();

  if (publish_occupancy_grid_)
  {
    ros_occupancy_grid_publisher_ = node_ptr_->create_publisher<OccupancyGridMsg>(
        "~/" + buffer_id_utils::topic_name_map[ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID], qos);
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