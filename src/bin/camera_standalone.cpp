/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
