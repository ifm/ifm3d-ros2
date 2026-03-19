// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_IMU_TF_PUBLISHER_HPP_
#define IFM3D_ROS2_IMU_TF_PUBLISHER_HPP_

#include <ifm3d_ros2/sensor_tf_publisher.hpp>

namespace ifm3d_ros2
{

/**
 * @brief Wrapper around tf_static_broadcaser to easily publish ifm_base -> mounting_frame -> sensor_frame transforms
 *
 */
class ImuTfPublisher : public SensorTfPublisher
{
public:
  ImuTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, std::string base_frame_name,
                 std::string mounting_frame_name, std::string sensor_frame_name);
  ImuTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr);

  /**
   * @brief Updates internal transforms and publishes them, if changes are detected
   * 
   * @param new_tf_base_to_mounting 
   * @param new_tf_mounting_to_sensor 
   * @param stamp 
   * @return true 
   * @return false 
   */
  bool update_and_publish_tf_if_changed(const geometry_msgs::msg::Transform& new_tf_base_to_mounting,
                                        const geometry_msgs::msg::Transform& new_tf_base_to_sensor,
                                        const rclcpp::Time& stamp);

private:
  geometry_msgs::msg::TransformStamped tf_base_to_mounting_, tf_mounting_to_sensor_;
};

}  // namespace ifm3d_ros2
#endif