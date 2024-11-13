// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_SERVICES_HPP_
#define IFM3D_ROS2_SERVICES_HPP_

#include <mutex>

#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <ifm3d/device.h>

#include <ifm3d_ros2/srv/config.hpp>
#include <ifm3d_ros2/srv/dump.hpp>
#include <ifm3d_ros2/srv/get_diag.hpp>
#include <ifm3d_ros2/srv/softoff.hpp>
#include <ifm3d_ros2/srv/softon.hpp>
#include <string>

using DumpRequest = std::shared_ptr<ifm3d_ros2::srv::Dump::Request>;
using DumpResponse = std::shared_ptr<ifm3d_ros2::srv::Dump::Response>;
using DumpService = ifm3d_ros2::srv::Dump;
using DumpServer = rclcpp::Service<ifm3d_ros2::srv::Dump>::SharedPtr;

using ConfigRequest = std::shared_ptr<ifm3d_ros2::srv::Config::Request>;
using ConfigResponse = std::shared_ptr<ifm3d_ros2::srv::Config::Response>;
using ConfigService = ifm3d_ros2::srv::Config;
using ConfigServer = rclcpp::Service<ifm3d_ros2::srv::Config>::SharedPtr;

using SoftoffRequest = std::shared_ptr<ifm3d_ros2::srv::Softoff::Request>;
using SoftoffResponse = std::shared_ptr<ifm3d_ros2::srv::Softoff::Response>;
using SoftoffService = ifm3d_ros2::srv::Softoff;
using SoftoffServer = rclcpp::Service<ifm3d_ros2::srv::Softoff>::SharedPtr;

using SoftonRequest = std::shared_ptr<ifm3d_ros2::srv::Softon::Request>;
using SoftonResponse = std::shared_ptr<ifm3d_ros2::srv::Softon::Response>;
using SoftonService = ifm3d_ros2::srv::Softon;
using SoftonServer = rclcpp::Service<ifm3d_ros2::srv::Softon>::SharedPtr;

using GetDiagRequest = std::shared_ptr<ifm3d_ros2::srv::GetDiag::Request>;
using GetDiagResponse = std::shared_ptr<ifm3d_ros2::srv::GetDiag::Response>;
using GetDiagService = ifm3d_ros2::srv::GetDiag;
using GetDiagServer = rclcpp::Service<ifm3d_ros2::srv::GetDiag>::SharedPtr;

namespace ifm3d_ros2
{
class BaseServices : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
                     public std::enable_shared_from_this<BaseServices>
{
public:
  BaseServices(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr cam,
               ifm3d::PortInfo port_info, std::shared_ptr<std::mutex> ifm3d_mutex);

protected:
  void Dump(std::shared_ptr<rmw_request_id_t> request_header, DumpRequest req, DumpResponse resp);
  void Config(std::shared_ptr<rmw_request_id_t> request_header, ConfigRequest req, ConfigResponse resp);
  void Softoff(std::shared_ptr<rmw_request_id_t> request_header, SoftoffRequest req, SoftoffResponse resp);
  void Softon(std::shared_ptr<rmw_request_id_t> request_header, SoftonRequest req, SoftonResponse resp);
  void GetDiag(std::shared_ptr<rmw_request_id_t> request_header, GetDiagRequest req, GetDiagResponse resp);

  rclcpp::Logger logger_;

private:
  // Need a pointer to node to create services
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;

  /// global mutex on ifm3d core data structures `fg_`
  std::shared_ptr<std::mutex> ifm3d_mutex_{};

  ifm3d::O3R::Ptr cam_{};
  ifm3d::PortInfo port_info_{};

  // Service Servers
  DumpServer dump_srv_{};
  ConfigServer config_srv_{};
  SoftoffServer soft_off_srv_{};
  SoftonServer soft_on_srv_{};
  GetDiagServer get_diag_srv_{};
};

}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_BUFFER_CONVERSIONS_HPP_