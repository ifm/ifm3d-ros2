// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_CAMERA_TF_PUBLISHER_HPP_
#define IFM3D_ROS2_CAMERA_TF_PUBLISHER_HPP_

#include <ifm3d_ros2/sensor_tf_publisher.hpp>

namespace ifm3d_ros2
{

  /**
   * @brief Wrapper around tf_static_broadcaser to easily publish ifm_base -> mounting_frame -> sensor_frame transforms
   * 
   */
class CameraTfPublisher : public SensorTfPublisher
{
public:
  CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr, std::string port,
                    std::string base_frame_name, std::string mounting_frame_name, std::string sensor_frame_name);
  CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr, std::string port);

  bool update_and_publish_tf_if_changed(const geometry_msgs::msg::TransformStamped& new_tf_base_to_sensor);

  geometry_msgs::msg::TransformStamped read_tf_base_to_mounting_from_config(builtin_interfaces::msg::Time stamp);

private:
  ifm3d::O3R::Ptr o3r_ptr_;
  std::string port_;
  geometry_msgs::msg::TransformStamped tf_base_to_sensor_, tf_base_to_mounting_, tf_mounting_to_sensor_;
};

}  // namespace ifm3d_ros2
#endif