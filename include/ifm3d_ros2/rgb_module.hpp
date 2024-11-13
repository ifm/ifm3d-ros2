// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_RGB_MODULE_HPP_
#define IFM3D_ROS2_RGB_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ifm3d_ros2/function_module.hpp>
#include <ifm3d_ros2/camera_tf_publisher.hpp>
#include <ifm3d_ros2/msg/rgb_info.hpp>
#include <type_traits>

#include <ifm3d/fg/frame.h>

using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using CompressedImagePublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<CompressedImageMsg>>;

using RGBInfoMsg = ifm3d_ros2::msg::RGBInfo;
using RGBInfoPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<RGBInfoMsg>>;

using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using CameraInfoPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<CameraInfoMsg>>;

namespace ifm3d_ros2
{
class RgbModule : public FunctionModule, public std::enable_shared_from_this<RgbModule>
{
public:
  RgbModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr,
            std::string port, uint32_t width, uint32_t height);
  void handle_frame(ifm3d::Frame::Ptr frame);

  const std::string get_name()
  {
    return "rgb_module";
  };

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

  // The list is accessed in the node, to start the FrameGrabber
  std::vector<ifm3d::buffer_id> buffer_id_list_{};

protected:
  void parse_params();
  void set_parameter_event_callbacks();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;
  CameraTfPublisher tf_publisher_;

  // Values read from incoming image buffers
  uint32_t width_;
  uint32_t height_;

  // Boolean to track first time publishing, so that we only
  // publish the static transforms once
  bool first_;

  // Publishers
  CompressedImagePublisher rgb_publisher_;
  RGBInfoPublisher rgb_info_publisher_;
  CameraInfoPublisher camera_info_publisher_;

  // Parameters
  rcl_interfaces::msg::ParameterDescriptor buffer_id_list_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_base_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_mounting_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_optical_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_mounting_to_optical_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_base_to_mounting_descriptor_;

  // TF handling

  /// Subscription to parameter changes
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  /// Callbacks need to be stored to work properly; using a map with parameter name as key
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> registered_param_callbacks_;
};

}  // namespace ifm3d_ros2

#endif