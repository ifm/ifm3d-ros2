/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2025 ifm electronic, gmbh
 */

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ifm3d_ros2/pds_node.hpp>

int main(int argc, char** argv)
{
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto pds_node = std::make_shared<ifm3d_ros2::PdsNode>(options);
  exec.add_node(pds_node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
