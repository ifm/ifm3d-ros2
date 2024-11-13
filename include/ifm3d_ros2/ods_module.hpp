// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_ODS_MODULE_HPP_
#define IFM3D_ROS2_ODS_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>

#include <ifm3d_ros2/function_module.hpp>
#include <ifm3d_ros2/msg/zones.hpp>

#include <ifm3d/fg/frame.h>

namespace ifm3d_ros2
{
/**
 * @brief Wraps the ODS application.
 */
class OdsModule : public FunctionModule, public std::enable_shared_from_this<OdsModule>
{
  using CostmapMsg = nav2_msgs::msg::Costmap;
  using CostmapPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<CostmapMsg>>;

  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
  using OccupancyGridPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<OccupancyGridMsg>>;

  using ZonesMsg = ifm3d_ros2::msg::Zones;
  using ZonesPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ZonesMsg>>;

public:
  OdsModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr);
  // Main functions that take care of deserializing
  // and publishing the ODS data
  void handle_frame(ifm3d::Frame::Ptr frame);
  nav_msgs::msg::OccupancyGrid extract_ros_occupancy_grid(ifm3d::Frame::Ptr frame);
  nav2_msgs::msg::Costmap extract_ros_costmap(ifm3d::Frame::Ptr frame);
  ifm3d_ros2::msg::Zones extract_zones(ifm3d::Frame::Ptr frame);

  const std::string get_name()
  {
    return "ods_module";
  };

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;

  OccupancyGridPublisher ros_occupancy_grid_publisher_;
  CostmapPublisher ros_costmap_publisher_;
  ZonesPublisher zones_publisher_;

  std::string frame_id_;
  bool publish_occupancy_grid_;
  bool publish_costmap_;
  rcl_interfaces::msg::ParameterDescriptor frame_id_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor publish_occupancy_grid_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor publish_costmap_descriptor_;
};

}  // namespace ifm3d_ros2

#endif