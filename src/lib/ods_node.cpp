/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/common/logging/logger.h>
#include <ifm3d/fg/frame.h>

#include <ifm3d_ros2/ods_node.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <fstream>
#include <exception>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <ifm3d_ros2/qos.hpp>

using json = ifm3d::json;
using namespace std::chrono_literals;

namespace ifm3d_ros2
{
namespace
{
}  // namespace

OdsNode::OdsNode(const rclcpp::NodeOptions& opts) : OdsNode::OdsNode("ods_node", opts)
{
}

OdsNode::OdsNode(const std::string& node_name, const rclcpp::NodeOptions& opts)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", opts), logger_(this->get_logger()), width_(0), height_(0)
{
  // unbuffered I/O to stdout (so we can see our log messages)
  std::setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  RCLCPP_INFO(this->logger_, "namespace: %s", this->get_namespace());
  RCLCPP_INFO(this->logger_, "node name: %s", this->get_name());
  RCLCPP_INFO(this->logger_, "middleware: %s", rmw_get_implementation_identifier());

  // declare our parameters and default values -- parameters defined in
  // the passed in `opts` (via __params:=/path/to/params.yaml on cmd line)
  // will override our default values specified.
  RCLCPP_INFO(this->logger_, "Declaring parameters...");
  this->init_params();
  RCLCPP_INFO(this->logger_, "After the parameters declaration");

  this->gil_ = std::make_shared<std::mutex>();

  RCLCPP_INFO(this->logger_, "node created, waiting for `configure()`...");
}

OdsNode::~OdsNode()
{
  RCLCPP_INFO(this->logger_, "Dtor called.");
}

TC_RETVAL OdsNode::on_configure(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_configure(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //
  // parse params and initialize instance vars
  //
  RCLCPP_INFO(this->logger_, "Parsing parameters...");
  parse_params();
  RCLCPP_INFO(this->logger_, "Parameters parsed.");

  // Add parameter subscriber, if none is active
  if (!param_subscriber_)
  {
    RCLCPP_INFO(logger_, "Adding callbacks to handle parameter changes at runtime...");
    set_parameter_event_callbacks();
    RCLCPP_INFO(this->logger_, "Callbacks set.");
  }

  //
  // We need a global lock on all the ifm3d core data structures
  //
  std::lock_guard<std::mutex> lock(*this->gil_);

  //
  // Initialize the camera interface
  //
  RCLCPP_INFO(this->logger_, "Initializing Device");
  this->o3r_ = std::make_shared<ifm3d::O3R>(this->ip_);
  RCLCPP_INFO(this->logger_, "Initializing FrameGrabber for data");
  this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->o3r_, this->pcic_port_);
  RCLCPP_DEBUG(this->logger_, "Initializing FrameGrabber for diagnostics");
  this->fg_diag_ = std::make_shared<ifm3d::FrameGrabber>(this->o3r_, this->o3r_->Port("diagnostics").pcic_port);
  RCLCPP_DEBUG(this->logger_, "Find out which data stream we are handling");

  // If a configuration file is provided, configure the device.
  if (this->config_file_!=""){
    std::ifstream file(this->config_file_);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open config file: " + this->config_file_);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    RCLCPP_INFO(this->logger_, "Setting configuration: %s",  buffer.str().c_str());
    ifm3d::json config_json = json::parse(buffer.str()) ;
    this->o3r_->Set(config_json);
  }

  // Get all the necessary info for the port.
  for (auto port : this->o3r_->Ports())
  {
    if (port.pcic_port == this->pcic_port_)
    {
      this->port_info_ = port;
    }
  }

  //
  // Initialize ODS Module
  //
  RCLCPP_INFO(this->logger_, "Creating OdsModule...");
  this->ods_module_ = std::make_shared<OdsModule>(this->get_logger(), shared_from_this());
  RCLCPP_INFO(this->logger_, "OdsModule created.");
  //
  // Initialize diagnostic Module
  //
  RCLCPP_INFO(this->logger_, "Creating DiagModule...");
  this->diag_module_ = std::make_shared<DiagModule>(this->get_logger(), shared_from_this(), this->o3r_);
  RCLCPP_INFO(this->logger_, "DiagModule created.");

  //
  // Create a list of all the modules to reduce duplicate code
  //
  this->modules_.push_back(this->ods_module_);
  this->modules_.push_back(this->diag_module_);

  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_configure(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }

  // Initialize the services using the BaseServices class
  RCLCPP_INFO(this->logger_, "Creating BaseServices...");
  this->base_services_ =
      std::make_shared<BaseServices>(this->get_logger(), shared_from_this(), this->o3r_, this->port_info_, this->gil_);
  RCLCPP_INFO(this->logger_, "BaseServices created.");

  RCLCPP_INFO(this->logger_, "Configuration complete.");
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL OdsNode::on_activate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  // Register Callbacks to handle new frames and print errors
  RCLCPP_DEBUG(this->logger_, "Registering the callbacks");
  this->fg_->OnNewFrame(std::bind(&OdsNode::frame_callback, this, std::placeholders::_1));
  this->fg_->OnError(std::bind(&OdsNode::error_callback, this, std::placeholders::_1));

  this->fg_diag_->OnAsyncError(
      std::bind(&OdsNode::async_error_callback, this, std::placeholders::_1, std::placeholders::_2));
  this->fg_diag_->OnAsyncNotification(
      std::bind(&OdsNode::async_notification_callback, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_DEBUG(this->logger_, "Registered all the callbacks");

  // The Framegrabber, needs a BufferList (a vector of std::variant)
  RCLCPP_INFO(this->logger_, "Starting the Framegrabbers...");
  ifm3d::FrameGrabber::BufferList buffer_list;
  buffer_list.push_back(ifm3d::buffer_id::O3R_ODS_INFO);
  buffer_list.push_back(ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID);

  // Start framegrabbers and wait for the returned future
  this->fg_->Start(buffer_list).wait();
  RCLCPP_DEBUG(this->logger_, "Data FrameGrabber started, frames should be streaming");

  this->fg_diag_->Start({}).wait();
  RCLCPP_INFO(this->logger_, "Diagnostic monitoring active.");

  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_activate(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL OdsNode::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(logger_, "Stopping FrameGrabber...");
  this->fg_->Stop().wait();

  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_deactivate(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL OdsNode::on_cleanup(const rclcpp_lifecycle::State& prev_state)
{
  // clean-up resources -- this will include our cam, fg,
  RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  std::lock_guard<std::mutex> lock(*this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");

  this->fg_.reset();
  this->o3r_.reset();

  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_cleanup(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
    module.reset();
  }
  // this->modules_.reset();

  RCLCPP_INFO(this->logger_, "Node cleanup complete.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL OdsNode::on_shutdown(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  // TODO: figure out how to properly shutdown modules
  this->fg_->Stop();
  this->fg_diag_->Stop();

  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_shutdown(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL OdsNode::on_error(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_error(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  std::lock_guard<std::mutex> lock(*this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");
  // this->im_.reset();
  this->fg_.reset();
  this->o3r_.reset();

  // Transition function modules
  // Transition function modules
  for (auto& module : this->modules_)
  {
    auto retval = module->on_error(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }

  RCLCPP_INFO(this->logger_, "Error processing complete.");

  return TC_RETVAL::SUCCESS;
}

void OdsNode::init_params()  // TODO cleanup params
{
  // Node name as string to set default frame names
  const std::string node_name(this->get_name());

  /*
   * For all parameters in alphabetical order:
   *   - Define Descriptor
   *   - Declare Parameter
   */

  rcl_interfaces::msg::ParameterDescriptor ip_descriptor;
  ip_descriptor.name = "ip";
  ip_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  ip_descriptor.description = "IP address of the camera";
  ip_descriptor.additional_constraints = "Should be an IPv4 address or resolvable name on your network";
  this->declare_parameter("ip", ifm3d::DEFAULT_IP, ip_descriptor);

  rcl_interfaces::msg::ParameterDescriptor pcic_port_descriptor;
  pcic_port_descriptor.name = "pcic_port";
  pcic_port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  pcic_port_descriptor.description =
      " TCP port the on-image processing platform pcic server is listening on. Corresponds to the port the camera head "
      "is connected to.";
  this->declare_parameter("pcic_port", 51010, pcic_port_descriptor);

  rcl_interfaces::msg::ParameterDescriptor xmlrpc_port_descriptor;
  xmlrpc_port_descriptor.name = "xmlrpc_port";
  xmlrpc_port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  xmlrpc_port_descriptor.description = "TCP port the on-camera xmlrpc server is listening on";
  xmlrpc_port_descriptor.additional_constraints = "A valid TCP port 0 - 65535";
  rcl_interfaces::msg::IntegerRange xmlrpc_port_range;
  xmlrpc_port_range.from_value = 0;
  xmlrpc_port_range.to_value = 65535;
  xmlrpc_port_range.step = 1;
  xmlrpc_port_descriptor.integer_range.push_back(xmlrpc_port_range);
  this->declare_parameter("xmlrpc_port", ifm3d::DEFAULT_XMLRPC_PORT, xmlrpc_port_descriptor);
}

void OdsNode::parse_params()
{
  /*
   * For all parameters in alphabetical order:
   *   - Read currently set parameter
   *   - Where applicable, parse read data into more useful data type
   */
  this->get_parameter("config_file", this->config_file_);
  RCLCPP_INFO(this->logger_, "Config file: %s", this->config_file_.c_str());

  this->get_parameter("ip", this->ip_);
  RCLCPP_INFO(this->logger_, "ip: %s", this->ip_.c_str());

  this->get_parameter("pcic_port", this->pcic_port_);
  RCLCPP_INFO(this->logger_, "pcic_port: %u", this->pcic_port_);

  this->get_parameter("xmlrpc_port", this->xmlrpc_port_);
  RCLCPP_INFO(this->logger_, "xmlrpc_port: %u", this->xmlrpc_port_);
}

void OdsNode::set_parameter_event_callbacks()
{
  // Create a parameter subscriber that can be used to monitor parameter changes
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  /*
   * For all parameters in alphabetical order:
   *   - Create a callback as lambda to handle parameter change at runtime
   *   - Add lambda to parameter subscriber
   */
  auto config_file_cb = [this](const rclcpp::Parameter& p) {
    RCLCPP_WARN(logger_, "This new config_file will be used after CONFIGURE transition was called: '%s'", p.as_string().c_str());
  };
  registered_param_callbacks_["config_file"] = param_subscriber_->add_parameter_callback("config_file", config_file_cb);

  auto ip_cb = [this](const rclcpp::Parameter& p) {
    RCLCPP_WARN(logger_, "This new ip will be used after CONFIGURE transition was called: '%s'", p.as_string().c_str());
  };
  registered_param_callbacks_["ip"] = param_subscriber_->add_parameter_callback("ip", ip_cb);

  auto pcic_port_cb = [this](const rclcpp::Parameter& p) {
    RCLCPP_WARN(logger_, "This new pcic_port will be used after CONFIGURE transition was called: %ld", p.as_int());
  };
  registered_param_callbacks_["pcic_port"] = param_subscriber_->add_parameter_callback("pcic_port", pcic_port_cb);

  auto xmlrpc_port_cb = [this](const rclcpp::Parameter& p) {
    this->xmlrpc_port_ = p.as_int();
    RCLCPP_INFO(logger_, "New xmlrpc_port: %d", this->xmlrpc_port_);
  };
  registered_param_callbacks_["xmlrpc_port"] = param_subscriber_->add_parameter_callback("xmlrpc_port", xmlrpc_port_cb);
}

void OdsNode::frame_callback(ifm3d::Frame::Ptr frame)
{
  this->ods_module_->handle_frame(frame);

  RCLCPP_DEBUG(this->logger_, "Frame callback done.");
}

void OdsNode::error_callback(const ifm3d::Error& error)
{
  RCLCPP_ERROR(logger_, "Error received from ifm3d: %s", error.what());
  // TODO send diagnostic message
}

void OdsNode::async_error_callback(int i, const std::string& s)
{
  this->diag_module_->handle_error(i, s);
  RCLCPP_DEBUG(this->logger_, "Async error callback done.");
}

void OdsNode::async_notification_callback(const std::string& s1, const std::string& s2)
{
  this->diag_module_->handle_notification(s1, s2);
  RCLCPP_DEBUG(this->logger_, "Async notification callback done.");
}

}  // namespace ifm3d_ros2

#include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(ifm3d_ros2::OdsNode)
