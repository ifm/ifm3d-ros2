// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2026 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_PDS_VIS_NODE_HPP_
#define IFM3D_ROS2_PDS_VIS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ifm3d_ros2/services.hpp>
#include <ifm3d_ros2/msg/pds_full_result.hpp>
#include <ifm3d_ros2/visibility_control.h>

using TC_RETVAL = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ifm3d_ros2
{
/**
 * Managed node that implements the publication of visualization messages for PDS full result messages.
 */
class IFM3D_ROS2_PUBLIC PdsVisNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Instantiates the LifecycleNode. At the completion of the ctor, the
   * following initialization (beyond calling the parent ctor) has been done:
   *
   * - A named logger for this node has been initialized
   * - All parameters have been declared and a `set` callback has been
   *   registered
   * - All publishers have been created.
   */
  explicit PdsVisNode(const std::string& node_name, const rclcpp::NodeOptions& opts);

  /**
   * Delegates construction to the above ctor.
   */
  explicit PdsVisNode(const rclcpp::NodeOptions& opts);

  /**
   * RAII deallocations. As of this writing, given that all structures are
   * handled by various types of smart pointers no "real work" needs to be
   * done here. However, for debugging purposes we emit a log message so we
   * know when the dtor has actually been called and hence when all
   * deallocations actually occur.
   */
  ~PdsVisNode() override;

  /**
   * Implements the "configuring" transition state
   *
   * The following operations are performed:
   *
   * - Parameters are parsed and held locally in instance variables.
   */
  TC_RETVAL on_configure(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "activating" transition state
   *
   * The following operations are performed:
   *
   * - The `on_activate()` method is called on all publishers
   * - Starts publising visualization markers for received PDS results
   */
  TC_RETVAL on_activate(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "deactivating" transition state
   *
   * The following operations are performed:
   *
   * - All publishers can their `on_deactivate()` method called
   * - Stops publising visualization markers for received PDS results
   */
  TC_RETVAL on_deactivate(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "cleaningup" transition state
   */
  TC_RETVAL on_cleanup(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "shuttingdown" transition state
   */
  TC_RETVAL on_shutdown(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "errorprocessing" transition state
   *
   * The following operations are performed:
   *
   * - The message publish is stopped (if running)
   */
  TC_RETVAL on_error(const rclcpp_lifecycle::State& prev_state) override;

  visualization_msgs::msg::MarkerArray
  pallet_visualization_markers(const ifm3d_ros2::msg::PalletDetectionArray& pallet_array) const;
  visualization_msgs::msg::MarkerArray rack_visualization_markers(const ifm3d_ros2::msg::RackDetection& rack_detection,
                                                                  const std::string& last_command_input) const;
  visualization_msgs::msg::MarkerArray volume_visualization_markers(const ifm3d_ros2::msg::VolumeCheck& volume_check,
                                                                    const std::string& last_command_input) const;

  visualization_msgs::msg::MarkerArray bounding_box_marker(const ifm3d_ros2::msg::BoundingBoxStamped bb,
                                                           const std_msgs::msg::ColorRGBA color = WHITE) const;
  visualization_msgs::msg::MarkerArray pallet_markers(const geometry_msgs::msg::PoseStamped& left_pose,
                                                      const geometry_msgs::msg::PoseStamped& center_pose,
                                                      const geometry_msgs::msg::PoseStamped& right_pose) const;

  /**
   * @brief Subscriber to handle incoming pds results and publishing visualization markers if node is active.
   *
   */
  void pds_result_callback(ifm3d_ros2::msg::PdsFullResult::UniquePtr msg);

private:
  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<ifm3d_ros2::msg::PdsFullResult>::SharedPtr pds_result_subcriber_;
  size_t currently_active_markers_;

  // Color constants
  static inline const std_msgs::msg::ColorRGBA GREEN = [] {
    std_msgs::msg::ColorRGBA c;
    c.r = 0.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    return c;
  }();
  static inline const std_msgs::msg::ColorRGBA BLUE = [] {
    std_msgs::msg::ColorRGBA c;
    c.r = 0.0f;
    c.g = 0.0f;
    c.b = 1.0f;
    c.a = 1.0f;
    return c;
  }();
  static inline const std_msgs::msg::ColorRGBA RED = [] {
    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 0.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    return c;
  }();
  static inline const std_msgs::msg::ColorRGBA WHITE = [] {
    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 1.0f;
    c.b = 1.0f;
    c.a = 1.0f;
    return c;
  }();
  static inline const std_msgs::msg::ColorRGBA YELLOW = [] {
    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    return c;
  }();

};  // end: class PdsVisNode

}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_PDS_NODE_HPP_
