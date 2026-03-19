// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_SENSOR_TF_PUBLISHER_HPP_
#define IFM3D_ROS2_SENSOR_TF_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ifm3d/device.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace ifm3d_ros2
{

/**
 * @brief Wrapper around tf_static_broadcaser to easily publish ifm_base -> mounting_frame -> sensor_frame transforms
 *
 */
class SensorTfPublisher
{
public:
  SensorTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, std::string base_frame_name,
                    std::string mounting_frame_name, std::string sensor_frame_name);
  SensorTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr);

  /**
   * @brief Creates human readable string from transform
   *
   * @param tf
   * @return std::string
   */
  std::string tf_to_string(geometry_msgs::msg::TransformStamped tf) const;

  /**
   * @brief Returns true if child_frame_id, parent_frame_id, and transform are identical; ignores stamp
   *
   * @param tf1
   * @param tf2
   * @return true
   * @return false
   */
  bool transform_identical(geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2) const;

  /**
   * @brief concatinates tf_base_to_mounting and tf_mounting_to_sensor, new tf uses provided stamp as timestamp
   *
   * @param stamp
   * @param tf_base_to_mounting
   * @param tf_mounting_to_sensor
   * @return geometry_msgs::msg::TransformStamped
   */
  geometry_msgs::msg::TransformStamped calculate_tf_base_to_sensor(
      builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_mounting,
      geometry_msgs::msg::TransformStamped tf_mounting_to_sensor) const;

  /**
   * @brief Calculates tf_mounting_to_sensor from tf_base_to_sensor and tf_base_to_mounting, new tf uses provided stamp
   * as timestamp
   *
   * @param stamp
   * @param tf_base_to_sensor
   * @param tf_base_to_mounting
   * @return geometry_msgs::msg::TransformStamped
   */
  geometry_msgs::msg::TransformStamped calculate_tf_mounting_to_sensor(
      builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_sensor,
      geometry_msgs::msg::TransformStamped tf_base_to_mounting) const;

  /**
   * @brief declares ROS 2 parameter for frame_names and publication
   *
   * @param sensor_name
   */
  void declare_parameters(std::string sensor_name = "sensor");

  /**
   * @brief reads parameters and sets member variables accordingly
   *
   */
  void parse_parameters();

  /**
   * @brief Provides quaternion for given intrinsic Euler RPY angles e.g., from ifm extrinsic calibration
   *
   * ifm3d provides roll, pitch, yaw angles for intrisic rotations while tf2 expects them for extrinsic rotation.
   * Therefore, a tf2 quaternion needs to be created from 3 separate rotations.
   *
   * @param roll
   * @param pitch
   * @param yaw
   * @return tf2::Quaternion
   */
  tf2::Quaternion quaternion_from_ifm_rpy(const double roll, const double pitch, const double yaw) const;

  std::string tf_base_link_frame_name_;
  std::string tf_mounting_link_frame_name_;
  std::string tf_sensor_link_frame_name_;
  bool tf_publish_mounting_to_sensor_;
  bool tf_publish_base_to_mounting_;

  rcl_interfaces::msg::ParameterDescriptor tf_base_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_mounting_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_sensor_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_base_to_mounting_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_mounting_to_sensor_descriptor_;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

}  // namespace ifm3d_ros2
#endif