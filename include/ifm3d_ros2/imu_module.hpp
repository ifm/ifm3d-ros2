// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_IMU_MODULE_HPP_
#define IFM3D_ROS2_IMU_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <ifm3d_ros2/function_module.hpp>
#include <ifm3d_ros2/imu_tf_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <ifm3d_ros2/msg/imu_burst.hpp>

#include <ifm3d/fg/frame.h>

namespace ifm3d_ros2
{
/**
 * @brief Exposes IMU measurements
 */
class ImuModule : public FunctionModule, public std::enable_shared_from_this<ImuModule>
{
  using ImuMsg = sensor_msgs::msg::Imu;
  using ImuBurstMsg = ifm3d_ros2::msg::ImuBurst;
  using ImuPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ImuMsg>>;
  using ImuBurstPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ImuBurstMsg>>;
  using TfMsg = geometry_msgs::msg::Transform;

public:
  ImuModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr);
  // Main functions that take care of deserializing
  // and publishing the IMU data
  void handle_frame(ifm3d::Frame::Ptr frame);
  bool extract_imu_data(ifm3d::Frame::Ptr frame, std::vector<ImuMsg>& imu_msgs_output, TfMsg& mounting_tf_output,
                        TfMsg& base_tf_output);

  const std::string get_name()
  {
    return "imu_module";
  };

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;
  ImuTfPublisher tf_publisher_;

  ImuPublisher average_imu_publisher_;
  ImuBurstPublisher imu_burst_publisher_;

  bool publish_averaged_data_;
  rcl_interfaces::msg::ParameterDescriptor publish_averaged_data_descriptor_;

  bool publish_bulk_data_;
  rcl_interfaces::msg::ParameterDescriptor publish_bulk_data_descriptor_;

  ImuMsg average(std::vector<ImuMsg>& messages);
};

}  // namespace ifm3d_ros2

#endif