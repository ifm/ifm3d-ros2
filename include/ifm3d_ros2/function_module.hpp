// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_FUNCTION_MODULE_HPP_
#define IFM3D_ROS2_FUNCTION_MODULE_HPP_

#include <string>

#include <ifm3d/fg/frame.h>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace ifm3d_ros2
{
/**
 * @brief Abstract representation of encapsulated camera functionality.
 *
 * Wraps a set of sub-functions of a camera.
 * Follows the lifecycle of rclcpp_lifecycle.
 */
class FunctionModule : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                       public std::enable_shared_from_this<FunctionModule>
{
public:
  FunctionModule(const rclcpp::Logger& logger);

  virtual void handle_frame(ifm3d::Frame::Ptr frame);

  virtual const std::string get_name() = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) = 0;

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) = 0;

protected:
  rclcpp::Logger logger_;
};

}  // namespace ifm3d_ros2

#endif