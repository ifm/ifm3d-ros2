// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_TOF_MODULE_HPP_
#define IFM3D_ROS2_TOF_MODULE_HPP_

#include <ifm3d/fg/frame.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ifm3d_ros2/function_module.hpp>
#include <ifm3d_ros2/camera_tf_publisher.hpp>
#include <ifm3d_ros2/msg/extrinsics.hpp>
#include <ifm3d_ros2/msg/intrinsics.hpp>
#include <ifm3d_ros2/msg/inverse_intrinsics.hpp>
#include <ifm3d_ros2/msg/tof_info.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using ImagePublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ImageMsg>>;

using PCLMsg = sensor_msgs::msg::PointCloud2;
using PCLPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<PCLMsg>>;

using ExtrinsicsMsg = ifm3d_ros2::msg::Extrinsics;
using ExtrinsicsPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ExtrinsicsMsg>>;

using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using CameraInfoPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<CameraInfoMsg>>;

using IntrinsicsMsg = ifm3d_ros2::msg::Intrinsics;
using IntrinsicsPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<IntrinsicsMsg>>;

using InverseIntrinsicsMsg = ifm3d_ros2::msg::InverseIntrinsics;
using InverseIntrinsicsPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<InverseIntrinsicsMsg>>;

using TOFInfoMsg = ifm3d_ros2::msg::TOFInfo;
using TOFInfoPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TOFInfoMsg>>;

namespace ifm3d_ros2
{
class TofModule : public FunctionModule, public std::enable_shared_from_this<TofModule>
{
public:
  TofModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr,
            std::string port, uint32_t width, uint32_t height);
  /**
   * @brief Unpacks data from the received frame, and publish
   * the topics corresponding to the requested image buffers.
   *
   * @param frame
   */

  void handle_frame(ifm3d::Frame::Ptr frame);

  const std::string get_name()
  {
    return "tof_module";
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

  // Value used to only publish the static transform once
  bool first_;

  // Publishers
  // We create lists of publishers for same image types, instead
  // of creating a publisher for each buffer. This way we limit the
  // number of publishers to the number of image types.
  std::map<ifm3d::buffer_id, ImagePublisher> image_publishers_;
  std::map<ifm3d::buffer_id, PCLPublisher> pcl_publishers_;
  std::map<ifm3d::buffer_id, ExtrinsicsPublisher> extrinsics_publishers_;
  std::map<ifm3d::buffer_id, CameraInfoPublisher> camera_info_publishers_;
  std::map<ifm3d::buffer_id, TOFInfoPublisher> tof_info_publishers_;
  std::map<ifm3d::buffer_id, IntrinsicsPublisher> intrinsics_publishers_;
  std::map<ifm3d::buffer_id, InverseIntrinsicsPublisher> inverse_intrinsics_publishers_;

  // Parameters
  rcl_interfaces::msg::ParameterDescriptor buffer_id_list_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_base_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_mounting_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_optical_frame_name_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_mounting_to_optical_descriptor_;
  rcl_interfaces::msg::ParameterDescriptor tf_publish_base_to_mounting_descriptor_;

  /// Subscription to parameter changes
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  /// Callbacks need to be stored to work properly; using a map with parameter name as key
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> registered_param_callbacks_;
};

}  // namespace ifm3d_ros2

#endif