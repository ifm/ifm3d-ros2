// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_DIAG_MODULE_HPP_
#define IFM3D_ROS2_DIAG_MODULE_HPP_

#include <ifm3d/device/o3r.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d_ros2/function_module.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ifm3d_ros2
{
class DiagModule : public FunctionModule, public std::enable_shared_from_this<DiagModule>
{
  using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticArrayPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<DiagnosticArrayMsg>>;

public:
  DiagModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
             std::shared_ptr<ifm3d::O3R> o3r);
  /**
   * @brief Main function that handles diagnostic messages.
   * When an error is received, it is directly published to the /diagnostic topic.
   *
   * @param i Error code
   * @param s Error description in JSON format
   */
  void handle_error(int i, const std::string& s);
  /**
   * @brief Main function that handles notifications.
   * When a notification is received, it is directly published to the /diagnostic topic
   * with the level OK.
   *
   * @param s1 Notification id
   * @param s2 Notification description in JSON
   */
  void handle_notification(const std::string& s1, const std::string& s2);

  /**
   * @brief Callback called every second, as defined by the diagnostic_timer_.
   * It formats and publishes the list of diagnostic messages coming from the device.
   *
   */
  void periodic_diag_callback();

  /**
   * Utility functions to creates a DiagnosticStatusMsg and a DiagnosticArrayMsg
   * from the received JSON string.
   *
   */
  DiagnosticStatusMsg create_diagnostic_status(const uint8_t level, ifm3d::json parsed_json);
  DiagnosticArrayMsg create_diagnostic_message(const uint8_t level, const std::string& json_msg);

  const std::string get_name()
  {
    return "diag_module";
  };

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

private:
  /// Pointer to the shared O3R object to access diagnostic using the GetDiagnostic methods.
  std::shared_ptr<ifm3d::O3R> o3r_;

  /// Pointer to the node to access the ROS2 API.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;

  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  DiagnosticArrayPublisher diagnostic_publisher_;
  /// A unique identifier for the diagnostic messages.
  std::string hardware_id_;
};

}  // namespace ifm3d_ros2

#endif