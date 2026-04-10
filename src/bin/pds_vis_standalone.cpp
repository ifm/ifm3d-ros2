/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2025 ifm electronic, gmbh
 */

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ifm3d_ros2/pds_vis_node.hpp>

int main(int argc, char** argv)
{
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto pds_vis_node = std::make_shared<ifm3d_ros2::PdsVisNode>(options);
  exec.add_node(pds_vis_node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
