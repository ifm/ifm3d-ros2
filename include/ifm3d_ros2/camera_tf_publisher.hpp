// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_CAMERA_TF_PUBLISHER_HPP_
#define IFM3D_ROS2_CAMERA_TF_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ifm3d/device.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ifm3d_ros2
{
class CameraTfPublisher
{
public:
  CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr, std::string port,
                    std::string base_frame_name, std::string mounting_frame_name, std::string optical_frame_name);
  CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr, std::string port);

  bool update_and_publish_tf_if_changed(const geometry_msgs::msg::TransformStamped& new_tf_base_to_optical);

  std::string tf_to_string(geometry_msgs::msg::TransformStamped tf);

  std::string tf_base_link_frame_name_;
  std::string tf_mounting_link_frame_name_;
  std::string tf_optical_link_frame_name_;
  bool tf_publish_mounting_to_optical_;
  bool tf_publish_base_to_mounting_;

protected:
  bool transform_identical(geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2);

  geometry_msgs::msg::TransformStamped read_tf_base_to_mounting_from_device_config(builtin_interfaces::msg::Time stamp);

  geometry_msgs::msg::TransformStamped get_tf_mounting_to_optical(
      builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_optical,
      geometry_msgs::msg::TransformStamped tf_base_to_mounting);

  geometry_msgs::msg::TransformStamped calculate_tf_base_to_optical(
      builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_mounting,
      geometry_msgs::msg::TransformStamped tf_mounting_to_optical);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;
  ifm3d::O3R::Ptr o3r_ptr_;
  std::string port_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_base_to_optical_, tf_base_to_mounting_, tf_mounting_to_optical_;
};

}  // namespace ifm3d_ros2
#endif