// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2025 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>

#include <ifm3d_ros2/pds_module.hpp>
#include <ifm3d_ros2/buffer_conversions.hpp>
#include <ifm3d_ros2/buffer_id_utils.hpp>
#include <ifm3d_ros2/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ifm3d_ros2
{
PdsModule::PdsModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                     std::shared_ptr<ifm3d::O3R> o3r, std::string app_instance)
  : FunctionModule(logger)
  , node_ptr_(node_ptr)
  , o3r_(o3r)
  , app_instance_(app_instance)
  , frame_id_("ifm_pds_link")
  , mode_(SetPdsModeSrv::Request::ACTION_ONLY)
  , action_state_(PdsModule::action_state::NOT_RUNNING)
{
  RCLCPP_INFO(logger_, "PdsModule contructor called.");

  RCLCPP_DEBUG(logger_, "Declaring parameter...");
  frame_id_descriptor_.name = "pds.frame_id";
  frame_id_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  frame_id_descriptor_.description = "Frame_id field used for pds messages (Default='ifm_pds_link').";
  if (!node_ptr_->has_parameter(frame_id_descriptor_.name))
  {
    node_ptr_->declare_parameter(frame_id_descriptor_.name, frame_id_, frame_id_descriptor_);
  }

  const auto qos = ifm3d_ros2::LowLatencyQoS();
  pallet_detection_array_publisher_ = node_ptr_->create_publisher<PalletDetectionArrayMsg>("~/pallet_detection", qos);
  rack_detection_publisher_ = node_ptr_->create_publisher<RackDetectionMsg>("~/rack_detection", qos);
  volume_check_publisher_ = node_ptr_->create_publisher<VolumeCheckMsg>("~/volume_check", qos);
  full_result_publisher_ = node_ptr_->create_publisher<FullResultMsg>("~/pds_full_result", qos);

  /*
   * Set Mode Service
   */
  auto service_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetPdsModeSrv::Request> request,
                                 std::shared_ptr<SetPdsModeSrv::Response> response) -> void {
    (void)request_header;
    RCLCPP_DEBUG(logger_, "Received set_mode service request");
    response->success = transition_mode(request->mode, request->get_pallet_request, request->get_rack_request,
                                        request->volume_check_request);
  };
  set_pds_mode_srv_ = node_ptr_->create_service<SetPdsModeSrv>("~/set_pds_mode", service_callback);

  /*
   * get_pallet Action Server
   */
  auto handle_pallet_goal = [this](const rclcpp_action::GoalUUID& uuid,
                                   std::shared_ptr<const GetPalletAction::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(logger_, "Received get_pallet request.");

    if (mode_ != SetPdsModeSrv::Request::ACTION_ONLY)
    {
      RCLCPP_WARN(logger_,
                  "PDS module not in mode `ACTION_ONLY`, use set_pds_mode action to switch modes, goal rejected.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (action_state_ != action_state::NOT_RUNNING)
    {
      RCLCPP_WARN(logger_, "PDS module is already executing an action, goal rejected.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Setting state to ACCEPTED to reject incoming goals
    action_state_ = action_state::ACCEPTED_GET_PALLET;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_pallet_cancel = [this](const std::shared_ptr<GoalHandleGetPallet> goal_handle) {
    (void)goal_handle;

    RCLCPP_INFO(logger_, "Received request to cancel get_pallet goal");

    if ((action_state_ != action_state::ACCEPTED_GET_PALLET) && (action_state_ != action_state::RUNNING_GET_PALLET))
    {
      RCLCPP_WARN(logger_, "No running get_pallet action to cancel.");
      return rclcpp_action::CancelResponse::REJECT;
    }

    action_state_ = action_state::CANCELLING_GET_PALLET;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_pallet_accepted = [this](const std::shared_ptr<GoalHandleGetPallet> goal_handle) {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle]() { return this->execute_get_pallet_action(goal_handle); };
    std::thread{ execute_in_thread }.detach();
  };

  get_pallet_action_server_ = rclcpp_action::create_server<GetPalletAction>(
      node_ptr_, "~/get_pallet", handle_pallet_goal, handle_pallet_cancel, handle_pallet_accepted);

  /*
   * get_rack Action Server
   */
  auto handle_rack_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GetRackAction::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(logger_, "Received get_rack request.");

    if (mode_ != SetPdsModeSrv::Request::ACTION_ONLY)
    {
      RCLCPP_WARN(logger_, "PDS module not in mode `ACTION_ONLY`, use set_pds_mode action to switch modes");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (action_state_ != action_state::NOT_RUNNING)
    {
      RCLCPP_WARN(logger_, "PDS module is already executing an action, goal rejected.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Setting state to ACCEPTED to reject incoming goals
    action_state_ = action_state::ACCEPTED_GET_RACK;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_rack_cancel = [this](const std::shared_ptr<GoalHandleGetRack> goal_handle) {
    (void)goal_handle;

    RCLCPP_INFO(logger_, "Received request to cancel get_rack goal");

    if ((action_state_ != action_state::ACCEPTED_GET_RACK) && (action_state_ != action_state::RUNNING_GET_RACK))
    {
      RCLCPP_WARN(logger_, "No running get_rack action to cancel.");
      return rclcpp_action::CancelResponse::REJECT;
    }

    action_state_ = action_state::CANCELLING_GET_RACK;

    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_rack_accepted = [this](const std::shared_ptr<GoalHandleGetRack> goal_handle) {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle]() { return this->execute_get_rack_action(goal_handle); };
    std::thread{ execute_in_thread }.detach();
  };

  get_rack_action_server_ = rclcpp_action::create_server<GetRackAction>(node_ptr_, "~/get_rack", handle_rack_goal,
                                                                        handle_rack_cancel, handle_rack_accepted);

  /*
   * volume_check Action Server
   */
  auto handle_volume_goal = [this](const rclcpp_action::GoalUUID& uuid,
                                   std::shared_ptr<const VolumeCheckAction::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(logger_, "Received volume_check request.");

    if (mode_ != SetPdsModeSrv::Request::ACTION_ONLY)
    {
      RCLCPP_WARN(logger_, "PDS module not in mode `ACTION_ONLY`, use set_pds_mode action to switch modes");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (action_state_ != action_state::NOT_RUNNING)
    {
      RCLCPP_WARN(logger_, "PDS module is already executing an action, goal rejected.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Setting state to ACCEPTED to reject incoming goals
    action_state_ = action_state::ACCEPTED_VOLUME_CHECK;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_volume_cancel = [this](const std::shared_ptr<GoalHandleVolumeCheck> goal_handle) {
    (void)goal_handle;

    RCLCPP_INFO(logger_, "Received request to cancel volume_check goal");

    if ((action_state_ != action_state::ACCEPTED_VOLUME_CHECK) && (action_state_ != action_state::RUNNING_VOLUME_CHECK))
    {
      RCLCPP_WARN(logger_, "No running volume_check action to cancel.");
      return rclcpp_action::CancelResponse::REJECT;
    }

    action_state_ = action_state::CANCELLING_VOLUME_CHECK;

    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_volume_accepted = [this](const std::shared_ptr<GoalHandleVolumeCheck> goal_handle) {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle]() { return this->execute_volume_check_action(goal_handle); };
    std::thread{ execute_in_thread }.detach();
  };

  volume_check_action_server_ = rclcpp_action::create_server<VolumeCheckAction>(
      node_ptr_, "~/volume_check", handle_volume_goal, handle_volume_cancel, handle_volume_accepted);
}

void PdsModule::handle_frame(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handle frame");

  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_RESULT_JSON))
  {
    RCLCPP_WARN(logger_, "PdsModule received frame without result json");
    return;
  }

  auto result_json_buffer = frame->GetBuffer(ifm3d::buffer_id::O3R_RESULT_JSON);
  std::string result_json_string =
      std::string(result_json_buffer.ptr<char>(0), strnlen(result_json_buffer.ptr<char>(0), result_json_buffer.size()));

  ifm3d::json result_json;
  try
  {
    result_json = ifm3d::json::parse(result_json_string);
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_ERROR(logger_, "Could not parse result json, skipping frame.");
    return;
  }

  RCLCPP_DEBUG_STREAM(logger_, "Received result: " << result_json.dump(2));

  if (mode_ == SetPdsModeSrv::Request::ACTION_ONLY)
  {
    // Received results are for a called action
    switch (action_state_)
    {
      case RUNNING_GET_PALLET:
        handle_get_pallet_result(result_json, current_get_pallet_goal_handle_);
        break;
      case RUNNING_GET_RACK:
        handle_get_rack_result(result_json, current_get_rack_goal_handle_);
        break;
      case RUNNING_VOLUME_CHECK:
        handle_volume_check_result(result_json, current_volume_check_goal_handle_);
        break;
      case CANCELLING_GET_PALLET:
      case CANCELLING_GET_RACK:
      case CANCELLING_VOLUME_CHECK:
        RCLCPP_DEBUG(logger_, "Received result for canceled goal, do nothing");
        break;
      default:
        RCLCPP_ERROR(logger_, "Received result in non-running state. Node might be in undefined state!");
    }
  }
  else
  {
    RCLCPP_DEBUG(logger_, "Received frame in continuous mode.");

    ifm3d_ros2::msg::PdsFullResult full_result_parsed;
    if (parse_full_result(result_json, full_result_parsed))
    {
      full_result_publisher_->publish(full_result_parsed);
    }
    else
    {
      RCLCPP_ERROR(logger_, "Could not parse full result message, skipping publishing.");
      return;
    }

    switch (mode_)
    {
      case SetPdsModeSrv::Request::GET_PALLET_CONTINUOUS:
        pallet_detection_array_publisher_->publish(full_result_parsed.pallet_detection_array);
        break;
      case SetPdsModeSrv::Request::GET_RACK_CONTINUOUS:
        rack_detection_publisher_->publish(full_result_parsed.rack_detection);
        break;
      case SetPdsModeSrv::Request::VOLUME_CHECK_CONTINUOUS:
        volume_check_publisher_->publish(full_result_parsed.volume_check);
        break;
      default:
        RCLCPP_WARN(logger_, "Recieved data in unknown state '%d', only publishing full result message.", mode_);
    }
  }

  RCLCPP_DEBUG(logger_, "Finished publishing pds messages.");
}

ifm3d::json PdsModule::create_get_pallet_command(const std::string& app_instance,
                                                 const ifm3d_ros2::msg::GetPalletRequest& request) const
{
  std::string pallet_order = request.pallet_order;
  uint8_t pallet_index = request.pallet_index;
  double depthHint = request.depth_hint;

  // Order options are white-listed, default to first option if given string is invalid
  const std::vector<std::string> order_options = { "scoreDescending", "zDescending", "zAscending", "yDescending",
                                                   "yAscending" };
  if (std::find(order_options.begin(), order_options.end(), pallet_order) == order_options.end())
  {
    // given pallet_order not a valid option
    RCLCPP_WARN(logger_, "pallet_order option %s is not a valid option, using %s instead.", pallet_order.c_str(),
                order_options[0].c_str());
    pallet_order = order_options[0];
  }

  // pallet index has to be in range [0..9]
  if (pallet_index > 9)
  {
    RCLCPP_WARN(logger_, "pallet_index of %d is out of range, using 0 instead!", pallet_index);
    pallet_index = 0;
  }

  // Contruct command JSON
  ifm3d::json command = { { "applications",
                            { { "instances",
                                { { app_instance,
                                    { { "configuration",
                                        { { "customization",
                                            { { "command", "getPallet" },
                                              { "getPallet",
                                                {
                                                    { "depthHint", depthHint },
                                                    { "palletIndex", pallet_index },
                                                    { "palletOrder", pallet_order },
                                                } } } } } } } } } } } } };

  return command;
}

ifm3d::json PdsModule::create_get_rack_command(const std::string& app_instance,
                                               const ifm3d_ros2::msg::GetRackRequest& request) const
{
  std::string horizontalDropPosition = request.horizontal_drop_position;
  std::string verticalDropPosition = request.vertical_drop_position;
  double depthHint = request.depth_hint;

  // Horizontal drop positions are white-listed, default to first option if given string is invalid
  const std::vector<std::string> horizontal_options = { "left", "center", "right" };
  if (std::find(horizontal_options.begin(), horizontal_options.end(), horizontalDropPosition) ==
      horizontal_options.end())
  {
    // given horizontal drop position not a valid option
    RCLCPP_WARN(logger_, "horizontalDropPosition option %s is not a valid option, using %s instead.",
                horizontalDropPosition.c_str(), horizontal_options[0].c_str());
    horizontalDropPosition = horizontal_options[0];
  }

  // Vertical drop positions are white-listed, default to first option if given string is invalid
  const std::vector<std::string> vertical_options = { "interior", "floor" };
  if (std::find(vertical_options.begin(), vertical_options.end(), verticalDropPosition) == vertical_options.end())
  {
    // given horizontal drop position not a valid option
    RCLCPP_WARN(logger_, "verticalDropPosition option %s is not a valid option, using %s instead.",
                verticalDropPosition.c_str(), vertical_options[0].c_str());
    verticalDropPosition = vertical_options[0];
  }

  // depthHint has a minimum value of 0.0
  if (depthHint < 0.0)
  {
    RCLCPP_WARN(logger_, "depthHint can't be negative, setting it to 0.0.");
    depthHint = 0.0;
  }

  // Contruct command JSON
  ifm3d::json command = { { "applications",
                            { { "instances",
                                { { app_instance,
                                    { { "configuration",
                                        { { "customization",
                                            { { "command", "getRack" },
                                              { "getRack",
                                                { { "horizontalDropPosition", horizontalDropPosition },
                                                  { "verticalDropPosition", verticalDropPosition },
                                                  { "depthHint", depthHint },
                                                  { "zHint", request.z_hint },
                                                  { "clearingVolume",
                                                    {
                                                        { "xMin", request.clearing_volume.x_min },
                                                        { "xMax", request.clearing_volume.x_max },
                                                        { "yMin", request.clearing_volume.y_min },
                                                        { "yMax", request.clearing_volume.y_max },
                                                        { "zMin", request.clearing_volume.z_min },
                                                        { "zMax", request.clearing_volume.z_max },
                                                    } } } } } } } } } } } } } } };

  return command;
}

ifm3d::json PdsModule::create_vol_check_command(const std::string& app_instance,
                                                const ifm3d_ros2::msg::BoundingBox& request) const
{
  // Contruct command JSON
  ifm3d::json command = { { "applications",
                            { { "instances",
                                { { app_instance,
                                    { { "configuration",
                                        { { "customization",
                                            { { "command", "volCheck" },
                                              { "volCheck",
                                                {
                                                    { "xMin", request.x_min },
                                                    { "xMax", request.x_max },
                                                    { "yMin", request.y_min },
                                                    { "yMax", request.y_max },
                                                    { "zMin", request.z_min },
                                                    { "zMax", request.z_max },
                                                } } } } } } } } } } } } };

  return command;
}

bool PdsModule::parse_get_pallet_result(const ifm3d::json result_json,
                                        ifm3d_ros2::msg::PalletDetectionArray& out_msg) const
{
  try
  {
    out_msg.header.stamp = rclcpp::Time(std::stoul(result_json["timeStamp"].get<std::string>()));
    out_msg.header.frame_id = frame_id_;

    out_msg.depth_estimation_voi.header = out_msg.header;
    out_msg.depth_estimation_voi.box.x_min = result_json["getPallet"]["depthEstimationVoi"]["xMin"].get<double>();
    out_msg.depth_estimation_voi.box.x_max = result_json["getPallet"]["depthEstimationVoi"]["xMax"].get<double>();
    out_msg.depth_estimation_voi.box.y_min = result_json["getPallet"]["depthEstimationVoi"]["yMin"].get<double>();
    out_msg.depth_estimation_voi.box.y_max = result_json["getPallet"]["depthEstimationVoi"]["yMax"].get<double>();
    out_msg.depth_estimation_voi.box.z_min = result_json["getPallet"]["depthEstimationVoi"]["zMin"].get<double>();
    out_msg.depth_estimation_voi.box.z_max = result_json["getPallet"]["depthEstimationVoi"]["zMax"].get<double>();

    out_msg.projection_voi.header = out_msg.header;
    out_msg.projection_voi.box.x_min = result_json["getPallet"]["projectionVoi"]["xMin"].get<double>();
    out_msg.projection_voi.box.x_max = result_json["getPallet"]["projectionVoi"]["xMax"].get<double>();
    out_msg.projection_voi.box.y_min = result_json["getPallet"]["projectionVoi"]["yMin"].get<double>();
    out_msg.projection_voi.box.y_max = result_json["getPallet"]["projectionVoi"]["yMax"].get<double>();
    out_msg.projection_voi.box.z_min = result_json["getPallet"]["projectionVoi"]["zMin"].get<double>();
    out_msg.projection_voi.box.z_max = result_json["getPallet"]["projectionVoi"]["zMax"].get<double>();

    for (auto pallet : result_json["getPallet"]["pallet"])
    {
      ifm3d_ros2::msg::PalletDetection detection;
      detection.header = out_msg.header;

      // orientation quaternion from intrinsic euler RPY
      tf2::Quaternion q_roll, q_pitch, q_yaw, q_combined;
      q_roll.setRPY(pallet["angles"]["rotX"].get<double>(), 0.0, 0.0);
      q_pitch.setRPY(0.0, pallet["angles"]["rotY"].get<double>(), 0.0);
      q_yaw.setRPY(0.0, 0.0, pallet["angles"]["rotZ"].get<double>());
      q_combined = q_roll * q_pitch * q_yaw;
      const geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q_combined);

      detection.center_position.header = detection.header;
      detection.center_position.pose.position.x = pallet["center"]["position"]["x"].get<double>();
      detection.center_position.pose.position.y = pallet["center"]["position"]["y"].get<double>();
      detection.center_position.pose.position.z = pallet["center"]["position"]["z"].get<double>();
      detection.center_position.pose.orientation = orientation;
      detection.center_height = pallet["center"]["height"].get<double>();
      detection.center_width = pallet["center"]["width"].get<double>();

      detection.left_position.header = detection.header;
      detection.left_position.pose.position.x = pallet["left"]["position"]["x"].get<double>();
      detection.left_position.pose.position.y = pallet["left"]["position"]["y"].get<double>();
      detection.left_position.pose.position.z = pallet["left"]["position"]["z"].get<double>();
      detection.left_position.pose.orientation = orientation;
      detection.left_height = pallet["left"]["height"].get<double>();
      detection.left_width = pallet["left"]["width"].get<double>();

      detection.right_position.header = detection.header;
      detection.right_position.pose.position.x = pallet["right"]["position"]["x"].get<double>();
      detection.right_position.pose.position.y = pallet["right"]["position"]["y"].get<double>();
      detection.right_position.pose.position.z = pallet["right"]["position"]["z"].get<double>();
      detection.right_position.pose.orientation = orientation;
      detection.right_height = pallet["right"]["height"].get<double>();
      detection.right_width = pallet["right"]["width"].get<double>();

      out_msg.detections.push_back(detection);
    }
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_WARN(logger_, "Could not parse PalletDetectionArray message from result json.");
    return false;
  }

  return true;
}

bool PdsModule::parse_get_rack_result(const ifm3d::json result_json, ifm3d_ros2::msg::RackDetection& out_msg) const
{
  try
  {
    out_msg.header.stamp = rclcpp::Time(std::stoul(result_json["timeStamp"].get<std::string>()));
    out_msg.header.frame_id = frame_id_;

    out_msg.position.header = out_msg.header;
    out_msg.position.pose.position.x = result_json["getRack"]["position"]["x"].get<double>();
    out_msg.position.pose.position.y = result_json["getRack"]["position"]["y"].get<double>();
    out_msg.position.pose.position.z = result_json["getRack"]["position"]["z"].get<double>();
    // orientation quaternion from intrinsic euler RPY
    tf2::Quaternion q_roll, q_pitch, q_yaw, q_combined;
    q_roll.setRPY(result_json["getRack"]["angles"]["rotX"].get<double>(), 0.0, 0.0);
    q_pitch.setRPY(0.0, result_json["getRack"]["angles"]["rotY"].get<double>(), 0.0);
    q_yaw.setRPY(0.0, 0.0, result_json["getRack"]["angles"]["rotZ"].get<double>());
    q_combined = q_roll * q_pitch * q_yaw;
    out_msg.position.pose.orientation = tf2::toMsg(q_combined);

    const uint64_t flags = result_json["getRack"]["flags"].get<uint64_t>();
    out_msg.no_beam_flag = ((flags & 1) > 0);
    out_msg.multiple_beams_flag = ((flags & 2) > 0);
    out_msg.beam_coverage_flag = ((flags & 4) > 0);
    out_msg.no_upright_flag = ((flags & 8) > 0);
    out_msg.multiple_uprights_flag = ((flags & 16) > 0);
    out_msg.upright_coverage_flag = ((flags & 32) > 0);
    out_msg.no_join_flag = ((flags & 64) > 0);
    out_msg.bad_transform_flag = ((flags & 128) > 0);
    out_msg.shelf_obstacle_flag = ((flags & 256) > 0);

    out_msg.horizontal_beam_voi.header = out_msg.header;
    out_msg.horizontal_beam_voi.box.x_min = result_json["getRack"]["horBeamVoi"]["xMin"].get<double>();
    out_msg.horizontal_beam_voi.box.x_max = result_json["getRack"]["horBeamVoi"]["xMax"].get<double>();
    out_msg.horizontal_beam_voi.box.y_min = result_json["getRack"]["horBeamVoi"]["yMin"].get<double>();
    out_msg.horizontal_beam_voi.box.y_max = result_json["getRack"]["horBeamVoi"]["yMax"].get<double>();
    out_msg.horizontal_beam_voi.box.z_min = result_json["getRack"]["horBeamVoi"]["zMin"].get<double>();
    out_msg.horizontal_beam_voi.box.z_max = result_json["getRack"]["horBeamVoi"]["zMax"].get<double>();

    out_msg.number_of_pixels = result_json["getRack"]["numPixels"].get<uint64_t>();
    out_msg.side = result_json["getRack"]["side"].get<std::string>();

    out_msg.upright_voi.header = out_msg.header;
    out_msg.upright_voi.box.x_min = result_json["getRack"]["uprightVoi"]["xMin"].get<double>();
    out_msg.upright_voi.box.x_max = result_json["getRack"]["uprightVoi"]["xMax"].get<double>();
    out_msg.upright_voi.box.y_min = result_json["getRack"]["uprightVoi"]["yMin"].get<double>();
    out_msg.upright_voi.box.y_max = result_json["getRack"]["uprightVoi"]["yMax"].get<double>();
    out_msg.upright_voi.box.z_min = result_json["getRack"]["uprightVoi"]["zMin"].get<double>();
    out_msg.upright_voi.box.z_max = result_json["getRack"]["uprightVoi"]["zMax"].get<double>();
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_WARN(logger_, "Could not parse RackDetection message from result json.");
    return false;
  }

  return true;
}

bool PdsModule::parse_volume_check_result(const ifm3d::json result_json, ifm3d_ros2::msg::VolumeCheck& out_msg) const
{
  try
  {
    out_msg.header.stamp = rclcpp::Time(std::stoul(result_json["timeStamp"].get<std::string>()));
    out_msg.header.frame_id = frame_id_;

    out_msg.nearest_x = result_json["volCheck"]["nearestX"].get<double>();
    out_msg.number_of_pixels = result_json["volCheck"]["numPixels"].get<uint64_t>();
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_WARN(logger_, "Could not parse VolumeCheck message from result json.");
    return false;
  }

  return true;
}

bool PdsModule::parse_full_result(const ifm3d::json result_json, ifm3d_ros2::msg::PdsFullResult& out_msg) const
{
  bool submessage_parsing_successful = true;

  try
  {
    out_msg.raw_json = result_json.dump();

    out_msg.pds_version_major = result_json["pdsVersion"]["major"].get<int32_t>();
    out_msg.pds_version_major = result_json["pdsVersion"]["minor"].get<int32_t>();
    out_msg.pds_version_major = result_json["pdsVersion"]["patch"].get<int32_t>();

    out_msg.timestamp = result_json["timeStamp"].get<std::string>();

    out_msg.last_command = result_json["lastCommand"].get<std::string>();
    out_msg.last_command_input = result_json["lastCommandInput"].dump();

    submessage_parsing_successful = submessage_parsing_successful &&
                                    parse_get_pallet_result(result_json, out_msg.pallet_detection_array) &&
                                    parse_get_rack_result(result_json, out_msg.rack_detection) &&
                                    parse_volume_check_result(result_json, out_msg.volume_check);
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_WARN(logger_, "Could not parse PdsFullResult message from result json.");
    return false;
  }

  return submessage_parsing_successful;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_configure called");
  (void)previous_state;

  node_ptr_->get_parameter(frame_id_descriptor_.name, frame_id_);
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", frame_id_descriptor_.name.c_str(), frame_id_.c_str());

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_cleanup called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_shutdown called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_activate called");
  (void)previous_state;

  pallet_detection_array_publisher_->on_activate();
  rack_detection_publisher_->on_activate();
  volume_check_publisher_->on_activate();
  full_result_publisher_->on_activate();

  // Retransition to current mode to set application state correctly
  if (!transition_mode(mode_))
  {
    RCLCPP_ERROR(logger_, "Activation failed, couldn't set mode correctly.");
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_deactivate called");
  (void)previous_state;

  if (action_state_ != action_state::NOT_RUNNING)
  {
    RCLCPP_WARN(logger_, "Can't deactivate node while action is being executed.");
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
  }

  pallet_detection_array_publisher_->on_deactivate();
  rack_detection_publisher_->on_deactivate();
  volume_check_publisher_->on_deactivate();
  full_result_publisher_->on_deactivate();

  // Setting Application to config state to stop buffers being send
  set_application_state("CONF");

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn PdsModule::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "PdsModule: on_error called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

size_t PdsModule::get_mode() const
{
  return mode_;
}

bool PdsModule::transition_mode(const uint8_t new_mode, const ifm3d_ros2::msg::GetPalletRequest& get_pallet_request,
                                const ifm3d_ros2::msg::GetRackRequest& get_rack_request,
                                const ifm3d_ros2::msg::BoundingBox& volume_check_request)
{
  // Can't switch from ACTION_ONLY to a different mode if goal is currently being executed
  if ((mode_ == SetPdsModeSrv::Request::ACTION_ONLY) && (new_mode != mode_) &&
      (action_state_ != action_state::NOT_RUNNING))
  {
    RCLCPP_WARN(logger_, "Currently executing an action, can't switch mode.");
    return false;
  }

  if (new_mode == SetPdsModeSrv::Request::ACTION_ONLY)
  {
    set_application_state("IDLE");
  }
  else if (new_mode == SetPdsModeSrv::Request::GET_PALLET_CONTINUOUS)
  {
    ifm3d::json command = create_get_pallet_command(app_instance_, get_pallet_request);
    command["applications"]["instances"][app_instance_]["state"] = "RUN";
    if (!execute_command(command))
    {
      RCLCPP_ERROR(logger_, "Switching mode failed.");
      return false;
    }
  }
  else if (new_mode == SetPdsModeSrv::Request::GET_RACK_CONTINUOUS)
  {
    ifm3d::json command = create_get_rack_command(app_instance_, get_rack_request);
    command["applications"]["instances"][app_instance_]["state"] = "RUN";
    if (!execute_command(command))
    {
      RCLCPP_ERROR(logger_, "Switching mode failed.");
      return false;
    }
  }
  else if (new_mode == SetPdsModeSrv::Request::VOLUME_CHECK_CONTINUOUS)
  {
    ifm3d::json command = create_vol_check_command(app_instance_, volume_check_request);
    command["applications"]["instances"][app_instance_]["state"] = "RUN";
    if (!execute_command(command))
    {
      RCLCPP_ERROR(logger_, "Switching mode failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger_, "Unknown desired mode %d, can't switch modes.", new_mode);
    return false;
  }

  mode_ = new_mode;
  RCLCPP_INFO(logger_, "Switched mode to %d.", mode_);
  return true;
}

void PdsModule::execute_get_pallet_action(const std::shared_ptr<GoalHandleGetPallet> goal_handle)
{
  if (action_state_ == action_state::CANCELLING_GET_PALLET)
  {
    RCLCPP_INFO(logger_, "Canceling get_pallet action before sending request to device");
    auto empty_result = std::make_shared<GetPalletAction::Result>();
    goal_handle->canceled(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }

  if (action_state_ != action_state::ACCEPTED_GET_PALLET)
  {
    RCLCPP_ERROR(logger_, "Trying to execute a goal while not in ACCEPTED state: aborting");
    auto empty_result = std::make_shared<GetPalletAction::Result>();
    goal_handle->abort(empty_result);
  }

  ifm3d::json command = create_get_pallet_command(app_instance_, goal_handle->get_goal()->request);

  current_get_pallet_goal_handle_ = goal_handle;

  action_state_ = action_state::RUNNING_GET_PALLET;

  if (!execute_command(command))
  {
    // Action fails if command is not accepted
    auto empty_result = std::make_shared<GetPalletAction::Result>();
    goal_handle->abort(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }
}

void PdsModule::execute_get_rack_action(const std::shared_ptr<GoalHandleGetRack> goal_handle)
{
  if (action_state_ == action_state::CANCELLING_GET_RACK)
  {
    RCLCPP_INFO(logger_, "Canceling get_rack action before sending request to device");
    auto empty_result = std::make_shared<GetRackAction::Result>();
    goal_handle->canceled(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }

  if (action_state_ != action_state::ACCEPTED_GET_RACK)
  {
    RCLCPP_ERROR(logger_, "Trying to execute a goal while not in ACCEPTED state: aborting");
    auto empty_result = std::make_shared<GetRackAction::Result>();
    goal_handle->abort(empty_result);
  }

  ifm3d::json command = create_get_rack_command(app_instance_, goal_handle->get_goal()->request);

  current_get_rack_goal_handle_ = goal_handle;

  action_state_ = action_state::RUNNING_GET_RACK;

  if (!execute_command(command))
  {
    // Action fails if command is not accepted
    auto empty_result = std::make_shared<GetRackAction::Result>();
    goal_handle->abort(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }
}

void PdsModule::execute_volume_check_action(const std::shared_ptr<GoalHandleVolumeCheck> goal_handle)
{
  if (action_state_ == action_state::CANCELLING_GET_PALLET)
  {
    RCLCPP_INFO(logger_, "Canceling volume_check action before sending request to device");
    auto empty_result = std::make_shared<VolumeCheckAction::Result>();
    goal_handle->canceled(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }

  if (action_state_ != action_state::ACCEPTED_VOLUME_CHECK)
  {
    RCLCPP_ERROR(logger_, "Trying to execute a goal while not in ACCEPTED state: aborting");
    auto empty_result = std::make_shared<VolumeCheckAction::Result>();
    goal_handle->abort(empty_result);
  }

  ifm3d::json command = create_vol_check_command(app_instance_, goal_handle->get_goal()->volume);

  current_volume_check_goal_handle_ = goal_handle;

  action_state_ = action_state::RUNNING_VOLUME_CHECK;

  if (!execute_command(command))
  {
    // Action fails if command is not accepted
    auto empty_result = std::make_shared<VolumeCheckAction::Result>();
    goal_handle->abort(empty_result);
    action_state_ = action_state::NOT_RUNNING;
  }
}

void PdsModule::handle_get_pallet_result(const ifm3d::json result_json,
                                         const std::shared_ptr<GoalHandleGetPallet> goal_handle)
{
  // Parse results and prepare outgoing data types
  auto action_result = std::make_shared<GetPalletAction::Result>();
  const bool switch_mode = goal_handle->get_goal()->switch_to_continuous_mode;
  FullResultMsg full_result_msg;

  if (!parse_full_result(result_json, full_result_msg))
  {
    // Could not parse result json
    RCLCPP_ERROR(logger_, "Action failed: result could not be parsed.%s",
                 (switch_mode) ? " Ignoring 'switch_to_continuous_mode', not switching." : "");
    goal_handle->abort(action_result);
    action_state_ = action_state::NOT_RUNNING;
    return;
  }

  action_result->pallet_detections = full_result_msg.pallet_detection_array;

  // Publish results to topics
  pallet_detection_array_publisher_->publish(full_result_msg.pallet_detection_array);
  full_result_publisher_->publish(full_result_msg);

  // Finalize goal handling
  goal_handle->succeed(action_result);
  RCLCPP_INFO(logger_, "Send get_pallet action result.");
  action_state_ = action_state::NOT_RUNNING;

  // Switching mode if indicated by flag in action request
  if (switch_mode)
  {
    RCLCPP_INFO(logger_, "Switching to mode 'GET_PALLET_CONTINUOUS' as 'switch_to_continuous_mode' was set to 'true'.");
    transition_mode(ifm3d_ros2::srv::SetPdsMode::Request::GET_PALLET_CONTINUOUS, goal_handle->get_goal()->request);
  }
}

void PdsModule::handle_get_rack_result(const ifm3d::json result_json,
                                       const std::shared_ptr<GoalHandleGetRack> goal_handle)
{
  // Parse results and prepare outgoing data types
  auto action_result = std::make_shared<GetRackAction::Result>();
  const bool switch_mode = goal_handle->get_goal()->switch_to_continuous_mode;
  FullResultMsg full_result_msg;

  if (!parse_full_result(result_json, full_result_msg))
  {
    // Could not parse result json
    RCLCPP_ERROR(logger_, "Action failed: result could not be parsed.%s",
                 (switch_mode) ? " Ignoring 'switch_to_continuous_mode', not switching." : "");
    goal_handle->abort(action_result);
    action_state_ = action_state::NOT_RUNNING;
    return;
  }

  action_result->rack_detection = full_result_msg.rack_detection;

  // Publish results to topics
  rack_detection_publisher_->publish(full_result_msg.rack_detection);
  full_result_publisher_->publish(full_result_msg);

  // Finalize goal handling
  goal_handle->succeed(action_result);
  RCLCPP_INFO(logger_, "Send get_rack action result.");
  action_state_ = action_state::NOT_RUNNING;

  // Switching mode if indicated by flag in action request
  if (switch_mode)
  {
    RCLCPP_INFO(logger_, "Switching to mode 'GET_RACK_CONTINUOUS' as 'switch_to_continuous_mode' was set to 'true'.");
    transition_mode(ifm3d_ros2::srv::SetPdsMode::Request::GET_RACK_CONTINUOUS,
                    ifm3d_ros2::msg::GetPalletRequest(),  // default request, will not be evaluated
                    goal_handle->get_goal()->request);
  }
}

void PdsModule::handle_volume_check_result(const ifm3d::json result_json,
                                           const std::shared_ptr<GoalHandleVolumeCheck> goal_handle)
{
  // Parse results and prepare outgoing data types
  auto action_result = std::make_shared<VolumeCheckAction::Result>();
  const bool switch_mode = goal_handle->get_goal()->switch_to_continuous_mode;
  FullResultMsg full_result_msg;

  if (!parse_full_result(result_json, full_result_msg))
  {
    // Could not parse result json
    RCLCPP_ERROR(logger_, "Action failed: result could not be parsed.%s",
                 (switch_mode) ? " Ignoring 'switch_to_continuous_mode', not switching." : "");
    goal_handle->abort(action_result);
    action_state_ = action_state::NOT_RUNNING;
    return;
  }

  action_result->volume_check = full_result_msg.volume_check;

  // Publish results to topics
  volume_check_publisher_->publish(full_result_msg.volume_check);
  full_result_publisher_->publish(full_result_msg);

  // Finalize goal handling
  goal_handle->succeed(action_result);
  RCLCPP_INFO(logger_, "Send volume_check action result.");
  action_state_ = action_state::NOT_RUNNING;

  // Switching mode if indicated by flag in action request
  if (switch_mode)
  {
    RCLCPP_INFO(logger_,
                "Switching to mode 'VOLUME_CHECK_CONTINUOUS' as 'switch_to_continuous_mode' was set to 'true'.");
    transition_mode(ifm3d_ros2::srv::SetPdsMode::Request::VOLUME_CHECK_CONTINUOUS,
                    ifm3d_ros2::msg::GetPalletRequest(),  // default request, will not be evaluated
                    ifm3d_ros2::msg::GetRackRequest(),    // default request, will not be evaluated
                    goal_handle->get_goal()->volume);
  }
}

bool PdsModule::execute_command(const ifm3d::json& command) const
{
  RCLCPP_DEBUG(logger_, "Executing command:\n%s", command.dump(2).c_str());

  try
  {
    o3r_->Set(command);
  }
  catch (const ifm3d::Error& e)
  {
    RCLCPP_ERROR_STREAM(logger_, "Setting command failed: " << e.what());
    return false;
  }

  return true;
}

void PdsModule::set_application_state(const std::string& state)
{
  ifm3d::json command;
  command["applications"]["instances"][app_instance_]["state"] = state;

  // Setting command without catching potential ifm3d::Error
  o3r_->Set(command);
}

}  // namespace ifm3d_ros2