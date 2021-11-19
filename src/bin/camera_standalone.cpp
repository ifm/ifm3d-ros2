/*
* SPDX-License-Identifier: Apache-2.0
* Copyright (C) 2019 ifm electronic, gmbh
*/

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ifm3d_ros2/camera_node.hpp>

int main(int argc, char **argv)
{
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  std::cout << "Before INIT" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  std::cout << "Before creating CameraNode instance" << std::endl;
  auto cam = std::make_shared<ifm3d_ros2::CameraNode>(options);
  std::cout << "Before adding node" << std::endl;
  exec.add_node(cam->get_node_base_interface());
  std::cout << "Before spinning" << std::endl;
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
