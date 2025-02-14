/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/device/o3r.h>
#include <ifm3d_ros2/camera_node.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <exception>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <ifm3d_ros2/qos.hpp>

using json = ifm3d::json;
using namespace std::chrono_literals;

namespace ifm3d_ros2
{
CameraNode::CameraNode(const rclcpp::NodeOptions& opts) : CameraNode::CameraNode("camera", opts)
{
}

CameraNode::CameraNode(const std::string& node_name, const rclcpp::NodeOptions& opts)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", opts), logger_(this->get_logger())
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

CameraNode::~CameraNode()
{
  RCLCPP_INFO(this->logger_, "Dtor called.");
}

TC_RETVAL CameraNode::on_configure(const rclcpp_lifecycle::State& prev_state)
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
  RCLCPP_INFO(this->logger_, "Initializing camera...");
  this->o3r_ = std::make_shared<ifm3d::O3R>(this->ip_, this->xmlrpc_port_);
  RCLCPP_INFO(this->logger_, "Initializing FrameGrabber");
  this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->o3r_, this->pcic_port_);
  // We do not use the asynchronous diagnostic right now, so no need to declare
  // the diagnostic framegrabber
  // this->fg_diag_ = std::make_shared<ifm3d::FrameGrabber>(this->o3r_, 50009);

  if (this->config_file_!=""){
    std::ifstream file(this->config_file_);

    if (!std::filesystem::is_regular_file(this->config_file_)) {
      RCLCPP_WARN(this->logger_, "Config file path exists but is not a regular file: %s", this->config_file_.c_str());
    }
    else {
      if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + this->config_file_);
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      RCLCPP_DEBUG(this->logger_, "Setting configuration: %s",  buffer.str().c_str());
      ifm3d::json config_json = json::parse(buffer.str()) ;
      this->o3r_->Set(config_json);
    }
  }

  // Get all the necessary info for the port.
  for (auto port : this->o3r_->Ports())
  {
    if (port.pcic_port == this->pcic_port_)
    {
      this->port_info_ = port;
    }
  }

  // Get resolution from port configuration
  std::string j_string = "/ports/" + this->port_info_.port + "/info/features/resolution";
  ifm3d::json::json_pointer j(j_string);
  auto resolution = this->o3r_->Get({ j_string })[j];
  auto width = static_cast<uint32_t>(resolution["width"]);
  auto height = static_cast<uint32_t>(resolution["height"]);

  // Determine data stream type from port info
  this->data_stream_type_ = stream_type_from_port_info(this->port_info_);

  // Create RGB or 3D modules depending on data type
  if (this->data_stream_type_ == ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d)
  {
    RCLCPP_INFO(logger_, "Data type is 2D");
    this->data_module_ =
        std::make_shared<RgbModule>(this->get_logger(), shared_from_this(), o3r_, this->port_info_.port, width, height);
    this->buffer_list_.insert(this->buffer_list_.end(),
                              std::get<std::shared_ptr<RgbModule>>(this->data_module_)->buffer_id_list_.begin(),
                              std::get<std::shared_ptr<RgbModule>>(this->data_module_)->buffer_id_list_.end());
    this->modules_.push_back(std::get<std::shared_ptr<RgbModule>>(this->data_module_));
    RCLCPP_DEBUG(this->logger_, "RgbModule created.");
  }
  else if (this->data_stream_type_ == ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d)
  {
    RCLCPP_INFO(logger_, "Data type is 3D");
    this->data_module_ =
        std::make_shared<TofModule>(this->get_logger(), shared_from_this(), o3r_, this->port_info_.port, width, height);
    this->buffer_list_.insert(this->buffer_list_.end(),
                              std::get<std::shared_ptr<TofModule>>(this->data_module_)->buffer_id_list_.begin(),
                              std::get<std::shared_ptr<TofModule>>(this->data_module_)->buffer_id_list_.end());
    this->modules_.push_back(std::get<std::shared_ptr<TofModule>>(this->data_module_));
    RCLCPP_DEBUG(this->logger_, "TofModule created.");
  }
  else
  {
    RCLCPP_ERROR(this->logger_, "Unknown data stream type");
    return TC_RETVAL::ERROR;
  }

  //
  // Initialize diagnostic Module
  //
  RCLCPP_DEBUG(this->logger_, "Creating DiagModule...");
  this->diag_module_ = std::make_shared<DiagModule>(this->get_logger(), shared_from_this(), this->o3r_);
  RCLCPP_DEBUG(this->logger_, "DiagModule created.");

  //
  // Create a list of all the modules to reduce duplicate code
  //
  this->modules_.push_back(this->diag_module_);

  // Transition function modules
  for (auto module : this->modules_)
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

TC_RETVAL CameraNode::on_activate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  // Register Callbacks to handle new frames and print errors
  this->fg_->OnNewFrame(std::bind(&CameraNode::frame_callback, this, std::placeholders::_1));
  this->fg_->OnError(std::bind(&CameraNode::error_callback, this, std::placeholders::_1));

  // Registering the error and notification callbacks is currently not necessary,
  // as currently we only use the periodic diagnostic poll.
  // this->fg_diag_->OnAsyncError(
  //     std::bind(&CameraNode::async_error_callback, this, std::placeholders::_1, std::placeholders::_2));
  // this->fg_diag_->OnAsyncNotification(
  //     std::bind(&CameraNode::async_notification_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Start the Framegrabber, needs a BufferList (a vector of std::variant)
  RCLCPP_INFO(this->logger_, "Starting the Framegrabber...");
  this->fg_->Start(this->buffer_list_).wait();
  // Currently, the diagnostic framegrabber is not used as we are only using the
  // periodic diagnostic
  // this->fg_diag_->Start({}).wait();
  RCLCPP_INFO(this->logger_, "Framegrabber started.");

  // Transition function modules
  for (auto module : this->modules_)
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

TC_RETVAL CameraNode::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(logger_, "Stopping FrameGrabber...");
  this->fg_->Stop().wait();

  // Transition function modules
  for (auto module : this->modules_)
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

TC_RETVAL CameraNode::on_cleanup(const rclcpp_lifecycle::State& prev_state)
{
  // clean-up resources -- this will include our cam, fg,
  RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  std::lock_guard<std::mutex> lock(*this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");

  this->fg_.reset();
  this->o3r_.reset();

  // Transition function modules
  for (auto module : this->modules_)
  {
    auto retval = module->on_cleanup(prev_state);
    if (retval != TC_RETVAL::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Module %s transition did not succeed.", module->get_name().c_str());
      // TODO de-transition previous modules
      return retval;
    }
  }

  RCLCPP_INFO(this->logger_, "Node cleanup complete.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_shutdown(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  // TODO: figure out how to properly shutdown modules
  this->fg_->Stop();
  // this->fg_diag_->Stop();

  // Transition function modules
  for (auto module : this->modules_)
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

TC_RETVAL CameraNode::on_error(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_error(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  std::lock_guard<std::mutex> lock(*this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");

  this->fg_.reset();
  this->o3r_.reset();

  // Transition function modules
  for (auto module : this->modules_)
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

void CameraNode::init_params()
{
  // Node name as string to set default frame names
  const std::string node_name(this->get_name());

  /*
   * For all parameters in alphabetical order:
   *   - Define Descriptor
   *   - Declare Parameter
   */
  rcl_interfaces::msg::ParameterDescriptor config_descriptor;
  config_descriptor.name = "config_file";
  config_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  config_descriptor.description = "Configuration file, in JSON format.";
  this->declare_parameter("config_file", "", config_descriptor);

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
  this->declare_parameter("pcic_port", ifm3d::DEFAULT_PCIC_PORT, pcic_port_descriptor);

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

  // TODO: extend parameter description to include required params:
  // password: for lock / unlock of JSON configuration
}

void CameraNode::parse_params()
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

void CameraNode::set_parameter_event_callbacks()
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

void CameraNode::frame_callback(ifm3d::Frame::Ptr frame)
{
  if (std::holds_alternative<std::shared_ptr<RgbModule>>(this->data_module_))
  {
    std::get<std::shared_ptr<RgbModule>>(this->data_module_)->handle_frame(frame);
  }
  else if (std::holds_alternative<std::shared_ptr<TofModule>>(this->data_module_))
  {
    std::get<std::shared_ptr<TofModule>>(this->data_module_)->handle_frame(frame);
  }
  RCLCPP_DEBUG(this->logger_, "Frame callback done.");
}

buffer_id_utils::data_stream_type CameraNode::stream_type_from_port_info(ifm3d::PortInfo port_info)
{
  buffer_id_utils::data_stream_type data_stream_type;
  RCLCPP_INFO(logger_, "Using port %s (pcic_port=%d) with type %s", port_info.port.c_str(), port_info.pcic_port,
              port_info.type.c_str());

  // Derive data_stream_type from PortInfo
  if (port_info.type == "3D")
  {
    data_stream_type = buffer_id_utils::data_stream_type::tof_3d;
    RCLCPP_INFO(logger_, "Data stream type is tof_3d.");
  }
  else if (port_info.type == "2D")
  {
    data_stream_type = buffer_id_utils::data_stream_type::rgb_2d;
    RCLCPP_INFO(logger_, "Data stream type is rgb_2d.");
  }
  else
  {
    data_stream_type = buffer_id_utils::data_stream_type::tof_3d;
    RCLCPP_ERROR(logger_, "Unknown data stream type '%s'. Defaulting to tof_3d.", port_info.type.c_str());
  }

  return data_stream_type;
}

void CameraNode::error_callback(const ifm3d::Error& error)
{
  RCLCPP_ERROR(logger_, "Error received from ifm3d: %s", error.what());
  // TODO send diagnostic message
}

void CameraNode::async_error_callback(int i, const std::string& s)
{
  this->diag_module_->handle_error(i, s);
  RCLCPP_DEBUG(this->logger_, "Async error callback done.");
}

void CameraNode::async_notification_callback(const std::string& s1, const std::string& s2)
{
  this->diag_module_->handle_notification(s1, s2);
  RCLCPP_DEBUG(this->logger_, "Async notification callback done.");
}
}  // namespace ifm3d_ros2

#include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(ifm3d_ros2::CameraNode)
