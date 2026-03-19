// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2025 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_PDS_MODULE_HPP_
#define IFM3D_ROS2_PDS_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <ifm3d_ros2/action/get_pallet.hpp>
#include <ifm3d_ros2/action/get_rack.hpp>
#include <ifm3d_ros2/action/volume_check.hpp>
#include <ifm3d_ros2/msg/pallet_detection_array.hpp>
#include <ifm3d_ros2/msg/pds_full_result.hpp>
#include <ifm3d_ros2/msg/rack_detection.hpp>
#include <ifm3d_ros2/msg/volume_check.hpp>
#include <ifm3d_ros2/srv/set_pds_mode.hpp>
#include <ifm3d_ros2/function_module.hpp>

#include <ifm3d/device/o3r.h>
#include <ifm3d/fg/frame.h>

namespace ifm3d_ros2
{
/**
 * @brief Wraps the PDS application.
 */
class PdsModule : public FunctionModule, public std::enable_shared_from_this<PdsModule>
{
  using PalletDetectionArrayMsg = ifm3d_ros2::msg::PalletDetectionArray;
  using PalletDetectionArrayPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<PalletDetectionArrayMsg>>;

  using RackDetectionMsg = ifm3d_ros2::msg::RackDetection;
  using RackDetectionPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<RackDetectionMsg>>;

  using VolumeCheckMsg = ifm3d_ros2::msg::VolumeCheck;
  using VolumeCheckPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<VolumeCheckMsg>>;

  using FullResultMsg = ifm3d_ros2::msg::PdsFullResult;
  using FullResultPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<FullResultMsg>>;

  using SetPdsModeSrv = ifm3d_ros2::srv::SetPdsMode;

  using GetPalletAction = ifm3d_ros2::action::GetPallet;
  using GoalHandleGetPallet = rclcpp_action::ServerGoalHandle<GetPalletAction>;
  using GetPalletActionServer = rclcpp_action::Server<GetPalletAction>::SharedPtr;

  using GetRackAction = ifm3d_ros2::action::GetRack;
  using GoalHandleGetRack = rclcpp_action::ServerGoalHandle<GetRackAction>;
  using GetRackActionServer = rclcpp_action::Server<GetRackAction>::SharedPtr;

  using VolumeCheckAction = ifm3d_ros2::action::VolumeCheck;
  using GoalHandleVolumeCheck = rclcpp_action::ServerGoalHandle<VolumeCheckAction>;
  using VolumeCheckActionServer = rclcpp_action::Server<VolumeCheckAction>::SharedPtr;

  enum action_state
  {
    NOT_RUNNING,
    ACCEPTED_GET_PALLET,
    ACCEPTED_GET_RACK,
    ACCEPTED_VOLUME_CHECK,
    RUNNING_GET_PALLET,
    RUNNING_GET_RACK,
    RUNNING_VOLUME_CHECK,
    CANCELLING_GET_PALLET,
    CANCELLING_GET_RACK,
    CANCELLING_VOLUME_CHECK,
  };

public:
  PdsModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, std::shared_ptr<ifm3d::O3R> o3r,
            std::string app_instance);

  // Main functions that take care of deserializing
  // and publishing the PDS data
  void handle_frame(ifm3d::Frame::Ptr frame);

  const std::string get_name()
  {
    return "pds_module";
  };

  ifm3d::json create_get_pallet_command(const std::string& app_instance,
                                        const ifm3d_ros2::msg::GetPalletRequest& request) const;

  ifm3d::json create_get_rack_command(const std::string& app_instance,
                                      const ifm3d_ros2::msg::GetRackRequest& request) const;

  ifm3d::json create_vol_check_command(const std::string& app_instance,
                                       const ifm3d_ros2::msg::BoundingBox& request) const;

  /**
   * @brief Populates provides message with GetPallet result data from json.
   *
   * @param result_json parsed json containing the result information
   * @param out_msg ROS 2 message with the parsed data
   * @return true if parsed without execption
   * @return false if parsring exeption was raised
   */
  bool parse_get_pallet_result(const ifm3d::json result_json, ifm3d_ros2::msg::PalletDetectionArray& out_msg) const;

  /**
   * @brief Populates provides message with GetRack result data from json.
   *
   * @param result_json parsed json containing the result information
   * @param out_msg ROS 2 message with the parsed data
   * @return true if parsed without execption
   * @return false if parsring exeption was raised
   */
  bool parse_get_rack_result(const ifm3d::json result_json, ifm3d_ros2::msg::RackDetection& out_msg) const;

  /**
   * @brief Populates provides message with VolumeCheck result data from json.
   *
   * @param result_json parsed json containing the result information
   * @param out_msg ROS 2 message with the parsed data
   * @return true if parsed without execption
   * @return false if parsring exeption was raised
   */
  bool parse_volume_check_result(const ifm3d::json result_json, ifm3d_ros2::msg::VolumeCheck& out_msg) const;

  /**
   * @brief Populates provides message with all result data from json.
   *
   * @param result_json parsed json containing the result information
   * @param out_msg ROS 2 message with the parsed data
   * @return true if parsed without execption
   * @return false if parsring exeption was raised
   */
  bool parse_full_result(const ifm3d::json result_json, ifm3d_ros2::msg::PdsFullResult& out_msg) const;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

protected:
  size_t get_mode() const;

  /**
   * @brief Transition to new PDS mode
   *
   * @param new_mode
   * @return true if transitioned successful or new_mode == current_mode
   * @return false if new_mode is not valid or transition failed
   */
  bool
  transition_mode(const uint8_t new_mode,
                  const ifm3d_ros2::msg::GetPalletRequest& get_pallet_request = ifm3d_ros2::msg::GetPalletRequest(),
                  const ifm3d_ros2::msg::GetRackRequest& get_rack_request = ifm3d_ros2::msg::GetRackRequest(),
                  const ifm3d_ros2::msg::BoundingBox& volume_check_request = ifm3d_ros2::msg::BoundingBox());

  void execute_get_pallet_action(const std::shared_ptr<GoalHandleGetPallet> goal_handle);
  void execute_get_rack_action(const std::shared_ptr<GoalHandleGetRack> goal_handle);
  void execute_volume_check_action(const std::shared_ptr<GoalHandleVolumeCheck> goal_handle);
  void handle_get_pallet_result(const ifm3d::json result_json, const std::shared_ptr<GoalHandleGetPallet> goal_handle);
  void handle_get_rack_result(const ifm3d::json result_json, const std::shared_ptr<GoalHandleGetRack> goal_handle);
  void handle_volume_check_result(const ifm3d::json result_json,
                                  const std::shared_ptr<GoalHandleVolumeCheck> goal_handle);

  /**
   * @brief Sends a command to the device
   *
   * The command is passed to the device using the Set() function to pass JSON.
   * Catches potentially occuring ifm3d::Error,
   * for example Algo errors for depth_hints outside of measurement range.
   *
   * @param command
   * @return true if command was accepted
   * @return false if error is encountered
   */
  bool execute_command(const ifm3d::json& command) const;

  /**
   * @brief Set the PDS application state of the device
   *
   * @param state might be "IDLE", "RUN", "CONF"
   * @throws ifm3d::Error in the unlikely case that the state switch fails
   */
  void set_application_state(const std::string& state);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;
  PalletDetectionArrayPublisher pallet_detection_array_publisher_;
  RackDetectionPublisher rack_detection_publisher_;
  VolumeCheckPublisher volume_check_publisher_;
  FullResultPublisher full_result_publisher_;
  rclcpp::Service<SetPdsModeSrv>::SharedPtr set_pds_mode_srv_;
  GetPalletActionServer get_pallet_action_server_;
  GetRackActionServer get_rack_action_server_;
  VolumeCheckActionServer volume_check_action_server_;

  std::shared_ptr<ifm3d::O3R> o3r_;
  std::string app_instance_;
  std::string frame_id_;
  rcl_interfaces::msg::ParameterDescriptor frame_id_descriptor_;
  u_int8_t mode_;

  action_state action_state_;
  std::shared_ptr<GoalHandleGetPallet> current_get_pallet_goal_handle_;
  std::shared_ptr<GoalHandleGetRack> current_get_rack_goal_handle_;
  std::shared_ptr<GoalHandleVolumeCheck> current_volume_check_goal_handle_;
};

}  // namespace ifm3d_ros2

#endif