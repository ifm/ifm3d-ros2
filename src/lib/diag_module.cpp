// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>
#include <vector>
#include <algorithm>

#include <ifm3d_ros2/diag_module.hpp>
#include <ifm3d_ros2/qos.hpp>

using json = ifm3d::json;
using namespace std::chrono_literals;

namespace ifm3d_ros2
{

namespace
{
uint8_t severity_to_diag_level(const std::string& severity)
{
  if (severity == "info")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  if (severity == "minor")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  if (severity == "major" || severity == "critical")
  {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

void add_diag_kv(diagnostic_msgs::msg::DiagnosticStatus& msg, const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  msg.values.push_back(kv);
}

void append_severity_guidance(diagnostic_msgs::msg::DiagnosticStatus& msg, const std::string& severity)
{
  add_diag_kv(msg, "severity_normalized", severity);

  if (severity == "info")
  {
    add_diag_kv(msg, "safety_action", "none");
    add_diag_kv(msg, "recovery_expectation", "normal_operation");
    add_diag_kv(msg, "operator_action", "monitor");
  }
  else if (severity == "minor")
  {
    add_diag_kv(msg, "safety_action", "continue_with_caution");
    add_diag_kv(msg, "recovery_expectation", "self_healing_possible");
    add_diag_kv(msg, "operator_action", "monitor_and_plan_service");
  }
  else if (severity == "major")
  {
    add_diag_kv(msg, "safety_action", "stop_robot");
    add_diag_kv(msg, "recovery_expectation", "self_healing_possible");
    add_diag_kv(msg, "operator_action", "stop_then_monitor_for_auto_recovery");
  }
  else if (severity == "critical")
  {
    add_diag_kv(msg, "safety_action", "stop_robot");
    add_diag_kv(msg, "recovery_expectation", "manual_intervention_required");
    add_diag_kv(msg, "operator_action", "stop_and_request_maintenance");
  }
  else
  {
    add_diag_kv(msg, "safety_action", "review_event");
    add_diag_kv(msg, "recovery_expectation", "unknown");
    add_diag_kv(msg, "operator_action", "inspect_diagnostics");
  }
}
}  // namespace

// Define severity levels in order of increasing severity
const std::vector<std::string> DiagModule::severity_levels_ = {"info", "minor", "major", "critical"};

DiagModule::DiagModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                       std::shared_ptr<ifm3d::O3R> o3r)
  : FunctionModule(logger), o3r_(o3r), node_ptr_(node_ptr), severity_threshold_("minor")
{
  RCLCPP_DEBUG(logger_, "DiagModule contructor called.");
  hardware_id_ = std::string(node_ptr_->get_namespace()) + "/" + std::string(node_ptr_->get_name());
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
  
  if (parsed_json.contains("severity"))
  {
    std::string severity = parsed_json["severity"];
    msg.level = severity_to_diag_level(severity);
    append_severity_guidance(msg, severity);
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

bool DiagModule::is_diagnostic_notification(const std::string& json_msg)
{
  try
  {
    ifm3d::json parsed_json = json::parse(json_msg);
    
    // Check if the notification contains diagnostic-related keywords
    if (parsed_json.contains("location"))
    {
      std::string location = parsed_json["location"];
      
      // Filter out configuration/parameter change notifications
      // These are operational notifications, not diagnostics
      if (location.find("/presets/") != std::string::npos ||
          location.find("/parameters/") != std::string::npos ||
          location.find("/command") != std::string::npos ||
          location.find("/load/") != std::string::npos ||
          location.find("/save/") != std::string::npos ||
          location.find("/configuration/") != std::string::npos)
      {
        return false;  // Not a diagnostic notification
      }
      
      // Allow diagnostic-related location paths
      if (location.find("/diagnostic") != std::string::npos ||
          location.find("/error") != std::string::npos ||
          location.find("/status") != std::string::npos ||
          location.find("/health") != std::string::npos)
      {
        return true;   // This is diagnostic-related
      }
    }
    
    // Check for other diagnostic indicators
    if (parsed_json.contains("severity") || 
        parsed_json.contains("events") ||
        parsed_json.contains("name"))
    {
      return true;  // Likely a diagnostic message
    }
    
    // Default: treat as operational notification
    return false;
  }
  catch (...)
  {
    // If we can't parse it, treat as diagnostic to be safe
    return true;
  }
}

std::vector<DiagModule::DiagnosticStatusMsg> DiagModule::create_group_diagnostics(const ifm3d::json& groups_json)
{
  std::vector<DiagnosticStatusMsg> group_statuses;
  
  for (const auto& group : groups_json.items())
  {
    std::string group_name = group.key();
    std::string status = group.value();
    
    // Only report groups that have issues (skip "no_incident" and "not_available" for cleaner output)
    if (status == "no_incident" || status == "not_available")
    {
      continue;  // Skip these to reduce noise
    }
    
    DiagnosticStatusMsg msg;
    
    // Set the diagnostic name based on group type
    if (group_name.find("port") == 0)
    {
      msg.name = "Port " + group_name.substr(4) + " Status";  // "port4" -> "Port 4 Status"
      msg.hardware_id = hardware_id_ + "/" + group_name;
    }
    else if (group_name.find("app") == 0)
    {
      msg.name = "Application " + group_name.substr(3) + " Status";  // "app0" -> "Application 0 Status"
      msg.hardware_id = hardware_id_ + "/" + group_name;
    }
    else
    {
      msg.name = group_name + " Status";
      msg.hardware_id = hardware_id_ + "/" + group_name;
    }
    
    // Map ifm3d group status to ROS diagnostic levels
    if (status == "info" || status == "minor" || status == "major" || status == "critical")
    {
      msg.level = severity_to_diag_level(status);
      msg.message = status;
      append_severity_guidance(msg, status);
      if (status == "critical")
      {
        RCLCPP_ERROR(logger_, "CRITICAL diagnostic on %s! Stop the robot and follow handling strategy.",
                     group_name.c_str());
      }
    }
    else
    {
      msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg.message = "Unknown status: " + status;
    }
    
    // Add the status as a key-value pair
    diagnostic_msgs::msg::KeyValue status_kv;
    status_kv.key = "status";
    status_kv.value = status;
    msg.values.push_back(status_kv);
    
    // Add group type
    diagnostic_msgs::msg::KeyValue type_kv;
    type_kv.key = "type";
    if (group_name.find("port") == 0)
    {
      type_kv.value = "camera_port";
    }
    else if (group_name.find("app") == 0)
    {
      type_kv.value = "application";
    }
    else
    {
      type_kv.value = "system_component";
    }
    msg.values.push_back(type_kv);
    
    group_statuses.push_back(msg);
  }
  
  return group_statuses;
}

bool DiagModule::should_report_severity(const std::string& severity, const std::string& threshold)
{
  // Find indices of severity and threshold in the ordered severity levels
  auto severity_it = std::find(severity_levels_.begin(), severity_levels_.end(), severity);
  auto threshold_it = std::find(severity_levels_.begin(), severity_levels_.end(), threshold);
  
  // If either is not found, be conservative and report it
  if (severity_it == severity_levels_.end() || threshold_it == severity_levels_.end())
  {
    return true;
  }
  
  // Report if severity level is >= threshold level (higher index = more severe)
  return std::distance(severity_levels_.begin(), severity_it) >= 
         std::distance(severity_levels_.begin(), threshold_it);
}

DiagModule::DiagnosticArrayMsg DiagModule::create_diagnostic_message(const uint8_t level, const std::string& json_msg)
{
  ifm3d::json parsed_json;
  DiagnosticArrayMsg diag_msg;

  try
  {
    parsed_json = json::parse(json_msg);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Invalid JSON received from callback with level %d", level);
  }
  try
  {
    // The diagnostic message is formatted differently depending on whether
    // we receive it through the asynchronous or synchronous method.
    diag_msg.header.stamp = rclcpp::Time(parsed_json["timestamp"]);
  }
  catch (...)
  {
    try
    {
      uint64_t ns_since_epoch = 0;

      // Try top-level timestamp first
      if (parsed_json.contains("timestamp") && parsed_json["timestamp"].is_string())
      {
        ns_since_epoch = std::stoull(parsed_json["timestamp"].get<std::string>());
      }
      else if (parsed_json.contains("timestamp") && parsed_json["timestamp"].is_number())
      {
        ns_since_epoch = parsed_json["timestamp"].get<uint64_t>();
      }

      diag_msg.header.stamp = rclcpp::Time(ns_since_epoch);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Invalid timestamp received from callback with level %d", level);
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

    // Always publish individual events from the events array (standard ROS 2 diagnostic behavior)
    if (diagnostic_json.contains("events"))
    {
      auto events = diagnostic_json["events"];
      for (auto event : events)
      {
        // Always publish events - let ROS 2 diagnostic tools handle filtering
        auto event_msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::OK, event.dump());
        diagnostic_publisher_->publish(event_msg);
        
        // Log critical events for visibility
        if (event.contains("severity") && event["severity"] == "critical")
        {
          RCLCPP_ERROR(logger_, "CRITICAL diagnostic event! ID: %d, Name: %s", 
                      event.value("id", 0), event.value("name", "unknown").c_str());
        }
      }
    }

    // Additionally publish groups as separate diagnostic statuses for system overview
    if (diagnostic_json.contains("groups"))
    {
      auto groups = diagnostic_json["groups"];
      auto group_diagnostics = create_group_diagnostics(groups);
      
      // Only publish group diagnostics if there are issues to report
      if (!group_diagnostics.empty())
      {
        DiagnosticArrayMsg groups_msg;
        
        // Set timestamp from the main diagnostic message
        try
        {
          if (diagnostic_json.contains("timestamp"))
          {
            uint64_t ns_since_epoch = 0;
            if (diagnostic_json["timestamp"].is_string())
            {
              ns_since_epoch = std::stoull(diagnostic_json["timestamp"].get<std::string>());
            }
            else if (diagnostic_json["timestamp"].is_number())
            {
              ns_since_epoch = diagnostic_json["timestamp"].get<uint64_t>();
            }
            groups_msg.header.stamp = rclcpp::Time(ns_since_epoch);
          }
          else
          {
            groups_msg.header.stamp = node_ptr_->get_clock()->now();
          }
        }
        catch (...)
        {
          groups_msg.header.stamp = node_ptr_->get_clock()->now();
        }
        
        // Add all group diagnostics to the message
        groups_msg.status = group_diagnostics;
        
        // Publish the groups diagnostic message
        diagnostic_publisher_->publish(groups_msg);
        RCLCPP_DEBUG(logger_, "Published %zu group diagnostic statuses.", group_diagnostics.size());
      }
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
  
  try
  {
    // Parse the JSON to see if it contains structured diagnostic data
    ifm3d::json parsed_json = json::parse(s);
    
    // Check if this is a full diagnostic JSON with events and groups
    if (parsed_json.contains("events") && parsed_json.contains("groups"))
    {
      RCLCPP_DEBUG(logger_, "Async error contains full diagnostic JSON - skipping to avoid duplicates with periodic polling");
      return; // Skip to avoid duplicates with periodic polling
    }
    
    // For individual error events, publish normally
    DiagnosticArrayMsg msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, s);
    diagnostic_publisher_->publish(msg);
    RCLCPP_DEBUG(this->logger_, "Published async error diagnostic message.");
  }
  catch (...)
  {
    // If parsing fails, publish as-is (might be a simple error message)
    DiagnosticArrayMsg msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, s);
    diagnostic_publisher_->publish(msg);
    RCLCPP_DEBUG(this->logger_, "Published async error diagnostic message (unparseable JSON).");
  }
}

void DiagModule::handle_notification(const std::string& s1, const std::string& s2)
{
  RCLCPP_INFO(logger_, "AsyncNotification received from ifm3d: %s  |  %s", s1.c_str(), s2.c_str());
  
  try
  {
    // Parse the JSON to see if it contains structured diagnostic data
    ifm3d::json parsed_json = json::parse(s2);
    
    // Check if this is a full diagnostic JSON with events and groups
    if (parsed_json.contains("events") && parsed_json.contains("groups"))
    {
      RCLCPP_DEBUG(logger_, "Async notification contains full diagnostic JSON - skipping to avoid duplicates with periodic polling");
      return; // Skip to avoid duplicates with periodic polling
    }
    
    // Forward diagnostic notifications and log the rest for operator awareness
    if (is_diagnostic_notification(s2))
    {
      DiagnosticArrayMsg msg = create_diagnostic_message(diagnostic_msgs::msg::DiagnosticStatus::OK, s2);
      diagnostic_publisher_->publish(msg);
      RCLCPP_DEBUG(this->logger_, "Forwarded diagnostic notification to /diagnostics topic.");
    }
    else
    {
      RCLCPP_DEBUG(this->logger_, "Ignoring operational notification (already visible in logs).");
    }
  }
  catch (...)
  {
    RCLCPP_DEBUG(this->logger_, "Failed to parse notification JSON, ignoring.");
  }
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
  diagnostic_timer_.reset();

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