// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>

#include <ifm3d_ros2/diag_module.hpp>
#include <ifm3d_ros2/qos.hpp>

using json = ifm3d::json;
using namespace std::chrono_literals;
namespace ifm3d_ros2
{
DiagModule::DiagModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                       std::shared_ptr<ifm3d::O3R> o3r)
  : FunctionModule(logger), o3r_(o3r), node_ptr_(node_ptr)
{
  RCLCPP_DEBUG(logger_, "DiagModule contructor called.");
  hardware_id_ = std::string(node_ptr_->get_namespace()) + "/" + std::string(get_name());
  RCLCPP_INFO(logger_, "hardware_id: %s", hardware_id_.c_str());
}

DiagModule::DiagnosticStatusMsg DiagModule::create_diagnostic_status(const uint8_t level, ifm3d::json parsed_json)
{
  DiagnosticStatusMsg msg;
  msg.level = level;

  if (parsed_json.empty())
  {
    RCLCPP_WARN(logger_, "Empty JSON received from callback with level %d", level);
    return msg;
  }
  if (parsed_json.contains("source"))
  {
    msg.hardware_id = parsed_json["source"];
  }
  if (parsed_json.contains("name"))
  {
    msg.name = parsed_json["name"];
  }

  if (parsed_json.contains("description"))
  {
    msg.message = parsed_json["description"];
  }
  //   TODO: should the KeyValue object be the diagnostic message id instead?
  for (auto& it : parsed_json.items())
  {
    try
    {
      diagnostic_msgs::msg::KeyValue obj;
      obj.key = it.key();
      obj.value = it.value().dump();
      msg.values.push_back(obj);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(logger_, "Couldn't parse entry of diagnostics status %s: %s", msg.name.c_str(), e.what());
    }
  }
  return msg;
}

DiagModule::DiagnosticArrayMsg DiagModule::create_diagnostic_message(const uint8_t level, const std::string& json_msg)
{
  ifm3d::json parsed_json;
  DiagnosticArrayMsg diag_msg;

  // parse json message
  try
  {
    parsed_json = json::parse(json_msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "Invalid JSON received from callback with level %d: %s", level, e.what());
    // use current time as timestamp because we cant parse the json message.
    diag_msg.header.stamp = node_ptr_->now(); 
    return diag_msg;
  }

  // get timestamp
  ifm3d::json timestamp;
  try
  {
    if (! parsed_json["timestamp"].is_null())
    {
      // The diagnostic message is formatted differently depending on whether
      // we receive it through the asynchronous or synchronous method.
      timestamp = parsed_json["timestamp"];

    }
    else if (! parsed_json["stats"]["lastActivated"]["timestamp"].is_null())
    {
      timestamp = parsed_json["stats"]["lastActivated"]["timestamp"];
    }
    else 
    {
      RCLCPP_ERROR(logger_, "No timestamp received from callback with level %d. Using current timestamp as fallback.", level);
      diag_msg.header.stamp = node_ptr_->now();
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "Invalid timestamp received from callback with level %d: %s. Using current timestamp as fallback.", level, e.what());
    diag_msg.header.stamp = node_ptr_->now();
  }

  // Timestamp my be string but we need int64 for convertion to rclcpp::Time.
  try
  {
    diag_msg.header.stamp = rclcpp::Time(std::stoll(timestamp.get<std::string>()));
  }
  catch (...)
  {
    try
    {
      diag_msg.header.stamp = rclcpp::Time(timestamp.get<int64_t>());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Invalid timestamp received from callback with level %d. Using current time. Error: %s", level, e.what());
      diag_msg.header.stamp = node_ptr_->now();
    }
  }

  diag_msg.status.push_back(create_diagnostic_status(level, parsed_json));
  return diag_msg;
}

void DiagModule::periodic_diag_callback()
{
  // TODO: do we really need this shared mutex here?
  // std::lock_guard<std::mutex> lock(this->gil_);
  try
  {
    ifm3d::json diagnostic_json = this->o3r_->GetDiagnostic();
    RCLCPP_DEBUG(logger_, "Diagnostics: %s", diagnostic_json.dump().c_str());

    auto events = diagnostic_json["events"];
    for (auto event : events)
    {
      diagnostic_publisher_->publish(
          create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::OK, event.dump()));
    }
  }
  catch (const ifm3d::Error& ex)
  {
    RCLCPP_INFO(logger_, "ifm3d error while trying to get the diagnostic: %s", ex.what());
  }
  catch (...)
  {
    RCLCPP_INFO(logger_, "Unknown error while trying to get the diagnostic");
  }
}

void DiagModule::handle_error(int i, const std::string& s)
{
  RCLCPP_ERROR(logger_, "AsyncError received from ifm3d: %d %s", i, s.c_str());
  DiagnosticArrayMsg msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, s);
  diagnostic_publisher_->publish(msg);
  RCLCPP_DEBUG(this->logger_, "Published the async diagnostic message.");
}

void DiagModule::handle_notification(const std::string& s1, const std::string& s2)
{
  RCLCPP_INFO(logger_, "AsyncNotification received from ifm3d: %s  |  %s", s1.c_str(), s2.c_str());
  DiagnosticArrayMsg msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::OK, s2);
  diagnostic_publisher_->publish(msg);
  RCLCPP_DEBUG(this->logger_, "Published the async notification.");
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_configure called");
  (void)previous_state;

  const auto qos = ifm3d_ros2::LowLatencyQoS();

  this->diagnostic_publisher_ = node_ptr_->create_publisher<DiagnosticArrayMsg>("/diagnostics", qos);
  // Timer for periodic publication of the full list of diagnostic messages
  this->diagnostic_timer_ = this->node_ptr_->create_wall_timer(1s, [this]() -> void { periodic_diag_callback(); });
  this->diagnostic_timer_->cancel();  // Deactivate timer, manage activity via lifecycle

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_cleanup called");
  (void)previous_state;

  diagnostic_publisher_.reset();

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_shutdown called");
  (void)previous_state;
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_activate called");
  (void)previous_state;

  this->diagnostic_publisher_->on_activate();
  this->diagnostic_timer_->reset();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_deactivate called");
  (void)previous_state;

  this->diagnostic_publisher_->on_deactivate();
  this->diagnostic_timer_->cancel();
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn DiagModule::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(logger_, "DiagModule: on_error called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}  // namespace ifm3d_ros2
