/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#include <ifm3d_ros2/camera_node.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/parameter.hpp>
#include <rmw/rmw.h>
#include <sensor_msgs/image_encodings.hpp>

#include <ifm3d/camera/camera_base.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d_ros2/qos.hpp>

#include <ifm3d/contrib/nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

namespace ifm3d_ros2
{
CameraNode::CameraNode(const rclcpp::NodeOptions& opts) : CameraNode::CameraNode("camera", opts)
{
}

CameraNode::CameraNode(const std::string& node_name, const rclcpp::NodeOptions& opts)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", opts)
  , logger_(this->get_logger())
  , test_destroy_(false)
  , camera_frame_(this->get_name() + std::string("_link"))
  , optical_frame_(this->get_name() + std::string("_optical_link"))
{
  // unbuffered I/O to stdout (so we can see our log messages)
  std::setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  RCLCPP_INFO(this->logger_, "namespace: %s", this->get_namespace());
  RCLCPP_INFO(this->logger_, "node name: %s", this->get_name());
  RCLCPP_INFO(this->logger_, "middleware: %s", rmw_get_implementation_identifier());
  RCLCPP_INFO(this->logger_, "camera frame: %s", this->camera_frame_.c_str());
  RCLCPP_INFO(this->logger_, "optical frame: %s", this->optical_frame_.c_str());

  // declare our parameters and default values -- parameters defined in
  // the passed in `opts` (via __params:=/path/to/params.yaml on cmd line)
  // will override our default values specified.
  this->init_params();
  this->add_on_set_parameters_callback(std::bind(&ifm3d_ros2::CameraNode::set_params_cb, this, std::placeholders::_1));

  RCLCPP_INFO(this->logger_, "After the parameters declaration");
  //
  // Set up our publishers.
  //
  this->xyz_pub_ = this->create_publisher<ImageMsg>("~/xyz_image", ifm3d_ros2::LowLatencyQoS());
  this->conf_pub_ = this->create_publisher<ImageMsg>("~/confidence", ifm3d_ros2::LowLatencyQoS());
  this->distance_pub_ = this->create_publisher<ImageMsg>("~/distance", ifm3d_ros2::LowLatencyQoS());
  this->amplitude_pub_ = this->create_publisher<ImageMsg>("~/amplitude", ifm3d_ros2::LowLatencyQoS());
  this->raw_amplitude_pub_ = this->create_publisher<ImageMsg>("~/raw_amplitude", ifm3d_ros2::LowLatencyQoS());
  this->cloud_pub_ = this->create_publisher<PCLMsg>("~/cloud", ifm3d_ros2::LowLatencyQoS());
  this->extrinsics_pub_ = this->create_publisher<ExtrinsicsMsg>("~/extrinsics", ifm3d_ros2::LowLatencyQoS());

  this->rgb_pub_ = this->create_publisher<ImageMsg>("~/rgb", ifm3d_ros2::LowLatencyQoS());

  RCLCPP_INFO(this->logger_, "After publishers declaration");

  //
  // Set up our service servers
  //
  this->dump_srv_ =
      this->create_service<DumpService>("~/Dump", std::bind(&ifm3d_ros2::CameraNode::Dump, this, std::placeholders::_1,
                                                            std::placeholders::_2, std::placeholders::_3));

  this->config_srv_ = this->create_service<ConfigService>(
      "~/Config", std::bind(&ifm3d_ros2::CameraNode::Config, this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3));

  this->soft_off_srv_ = this->create_service<SoftoffService>(
      "~/Softoff", std::bind(&ifm3d_ros2::CameraNode::Softoff, this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  this->soft_on_srv_ = this->create_service<SoftonService>(
      "~/Softon", std::bind(&ifm3d_ros2::CameraNode::Softon, this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3));

  RCLCPP_INFO(this->logger_, "node created, waiting for `configure()`...");
}

CameraNode::~CameraNode()
{
  RCLCPP_INFO(this->logger_, "Dtor...");

  try
  {
    this->stop_publish_loop();
  }
  catch (...)
  {
    RCLCPP_WARN(this->logger_, "Exception caught in dtor!");
  }

  RCLCPP_INFO(this->logger_, "Dtor done.");
}

TC_RETVAL CameraNode::on_configure(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_configure(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //
  // parse params and initialize instance vars
  //
  RCLCPP_INFO(this->logger_, "Parsing parameters...");

  this->get_parameter("pcic_port", this->pcic_port_);
  RCLCPP_INFO(this->logger_, "pcic_port: %u", this->pcic_port_);

  this->get_parameter("ip", this->ip_);
  RCLCPP_INFO(this->logger_, "ip: %s", this->ip_.c_str());

  this->get_parameter("xmlrpc_port", this->xmlrpc_port_);
  RCLCPP_INFO(this->logger_, "xmlrpc_port: %u", this->xmlrpc_port_);

  this->get_parameter("password", this->password_);
  RCLCPP_INFO(this->logger_, "password: %s", std::string(this->password_.size(), '*').c_str());
  ;

  this->get_parameter("schema_mask", this->schema_mask_);
  RCLCPP_INFO(this->logger_, "schema_mask: %u", this->schema_mask_);

  this->get_parameter("timeout_millis", this->timeout_millis_);
  RCLCPP_INFO(this->logger_, "timeout_millis: %d", this->timeout_millis_);

  this->get_parameter("timeout_tolerance_secs", this->timeout_tolerance_secs_);
  RCLCPP_INFO(this->logger_, "timeout_tolerance_secs: %f", this->timeout_tolerance_secs_);

  this->get_parameter("frame_latency_thresh", this->frame_latency_thresh_);
  RCLCPP_INFO(this->logger_, "frame_latency_thresh (seconds): %f", this->frame_latency_thresh_);

  this->get_parameter("sync_clocks", this->sync_clocks_);
  RCLCPP_INFO(this->logger_, "sync_clocks: %s", this->sync_clocks_ ? "true" : "false");

  RCLCPP_INFO(this->logger_, "Parameters parsed OK.");

  //
  // We need a global lock on all the ifm3d core data structures
  //
  std::lock_guard<std::mutex> lock(this->gil_);

  //
  // Initialize the camera interface
  //

  RCLCPP_INFO(this->logger_, "Initializing camera...");
  this->cam_ = ifm3d::CameraBase::MakeShared(this->ip_, this->xmlrpc_port_, this->password_);
  RCLCPP_INFO(this->logger_, "Initializing FrameGrabber with mask: %u", this->schema_mask_);
  this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, this->schema_mask_, this->pcic_port_);
  RCLCPP_INFO(this->logger_, "Initializing ImageBuffer...");
  this->im_ = std::make_shared<ifm3d::ImageBuffer>();

  RCLCPP_INFO(this->logger_, "Configuration complete.");
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_activate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //  activate all publishers
  RCLCPP_INFO(this->logger_, "Activating publishers...");
  this->xyz_pub_->on_activate();
  this->conf_pub_->on_activate();
  this->distance_pub_->on_activate();
  this->amplitude_pub_->on_activate();
  this->raw_amplitude_pub_->on_activate();
  this->cloud_pub_->on_activate();
  this->extrinsics_pub_->on_activate();
  this->rgb_pub_->on_activate();
  RCLCPP_INFO(this->logger_, "Publishers activated.");

  // start the publishing loop
  this->test_destroy_ = false;
  this->pub_loop_ = std::thread(std::bind(&ifm3d_ros2::CameraNode::publish_loop, this));

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //
  // stop the publish loop and join on the thread.
  //
  RCLCPP_INFO(this->logger_, "Stopping publishing thread...");
  this->test_destroy_ = true;
  if (this->pub_loop_.joinable())
  {
    this->pub_loop_.join();
    RCLCPP_INFO(this->logger_, "Publishing thread stopped.");
  }
  else
  {
    RCLCPP_WARN(this->logger_, "Publishing thread is not joinable!");
  }

  // explicitly deactive the publishers
  RCLCPP_INFO(this->logger_, "Deactivating publishers...");
  this->extrinsics_pub_->on_deactivate();
  this->cloud_pub_->on_deactivate();
  this->raw_amplitude_pub_->on_deactivate();
  this->amplitude_pub_->on_deactivate();
  this->distance_pub_->on_deactivate();
  this->conf_pub_->on_deactivate();
  this->xyz_pub_->on_deactivate();
  RCLCPP_INFO(this->logger_, "Publishers deactivated.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_cleanup(const rclcpp_lifecycle::State& prev_state)
{
  // clean-up resources -- this will include our cam, fg,
  RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  std::lock_guard<std::mutex> lock(this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");
  this->im_.reset();
  this->fg_.reset();
  this->cam_.reset();

  RCLCPP_INFO(this->logger_, "Node cleanup complete.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_shutdown(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //
  // We only need to make sure the pulishing loop has been stopped.
  //
  // ifm3d and cv::Mat dtors will dealloc the rest of our core data
  // structures.
  //
  this->stop_publish_loop();
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_error(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_error(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  this->stop_publish_loop();

  std::lock_guard<std::mutex> lock(this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");
  this->im_.reset();
  this->fg_.reset();
  this->cam_.reset();

  RCLCPP_INFO(this->logger_, "Error processing complete.");

  return TC_RETVAL::SUCCESS;
}

void CameraNode::init_params()
{
  RCLCPP_INFO(this->logger_, "declaring parameters...");

  rcl_interfaces::msg::ParameterDescriptor pcic_port_descriptor;
  pcic_port_descriptor.name = "pcic_port";
  pcic_port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  pcic_port_descriptor.description =
      " TCP port the on-image processing platform pcic server is listening on. Corresponds to the port the camera head "
      "is connected to.";
  this->declare_parameter("pcic_port", ifm3d::DEFAULT_PCIC_PORT, pcic_port_descriptor);
  // this->declare_parameter("pcic_port", 50012, pcic_port_descriptor);

  rcl_interfaces::msg::ParameterDescriptor ip_descriptor;
  ip_descriptor.name = "ip";
  ip_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  ip_descriptor.description = "IP address of the camera";
  ip_descriptor.additional_constraints = "Should be an IPv4 address or resolvable name on your network";
  this->declare_parameter("ip", ifm3d::DEFAULT_IP, ip_descriptor);

  rcl_interfaces::msg::ParameterDescriptor xmlrpc_port_descriptor;
  xmlrpc_port_descriptor.name = "xmlrpc_port";
  xmlrpc_port_descriptor.type =  // std::uint16_t
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  xmlrpc_port_descriptor.description = "TCP port the on-camera xmlrpc server is listening on";
  xmlrpc_port_descriptor.additional_constraints = "A valid TCP port 0 - 65535";
  //
  // XXX: There seems to be an IDL conversion problem here between
  // the ROS msg and the DDS idl. Need to get back to this when
  // Dashing is officially released.
  //
  // rcl_interfaces::msg::IntegerRange xmlrpc_port_range;
  // xmlrpc_port_range.from_value = 0;
  // xmlrpc_port_range.to_value = 65535;
  // xmlrpc_port_range.step = 1;
  // xmlrpc_port_descriptor.integer_range = xmlrpc_port_range;
  this->declare_parameter("xmlrpc_port", ifm3d::DEFAULT_XMLRPC_PORT, xmlrpc_port_descriptor);

  rcl_interfaces::msg::ParameterDescriptor password_descriptor;
  password_descriptor.name = "password";
  password_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  password_descriptor.description = "Password for camera edit session";
  this->declare_parameter("password", ifm3d::DEFAULT_PASSWORD, password_descriptor);

  rcl_interfaces::msg::ParameterDescriptor schema_mask_descriptor;
  schema_mask_descriptor.name = "schema_mask";
  schema_mask_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  schema_mask_descriptor.description = "Schema bitmask encoding which images should be streamed from the camera";
  schema_mask_descriptor.additional_constraints = "Unsigned 16-bit bitmask";
  //
  // XXX: add an IntegerRange constraint here
  //
  this->declare_parameter("schema_mask", ifm3d::IMG_RDIS | ifm3d::IMG_AMP | ifm3d::IMG_RAMP | ifm3d::IMG_CART,
                          schema_mask_descriptor);

  rcl_interfaces::msg::ParameterDescriptor timeout_millis_descriptor;
  timeout_millis_descriptor.name = "timeout_millis";
  timeout_millis_descriptor.type =  // long (signed)
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  timeout_millis_descriptor.description = "How long to block for a single frame from the camera (millis)";
  timeout_millis_descriptor.additional_constraints = "A timeout <= 0 will block indefinitely";
  this->declare_parameter("timeout_millis", 500, timeout_millis_descriptor);

  rcl_interfaces::msg::ParameterDescriptor timeout_tolerance_secs_descriptor;
  timeout_tolerance_secs_descriptor.name = "timeout_tolerance_secs";
  timeout_tolerance_secs_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  timeout_tolerance_secs_descriptor.description = "Time (seconds) without a frame to consider the camera disconnected";
  this->declare_parameter("timeout_tolerance_secs", 5.0, timeout_tolerance_secs_descriptor);

  rcl_interfaces::msg::ParameterDescriptor frame_latency_thresh_descriptor;
  frame_latency_thresh_descriptor.name = "frame_latency_thresh";
  frame_latency_thresh_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  frame_latency_thresh_descriptor.description = "Threshold (seconds) for determining use of acq time vs rcv time";
  this->declare_parameter("frame_latency_thresh", 1.0, frame_latency_thresh_descriptor);

  rcl_interfaces::msg::ParameterDescriptor sync_clocks_descriptor;
  sync_clocks_descriptor.name = "sync_clocks";
  sync_clocks_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  sync_clocks_descriptor.description = "Attempt to sync host and camera clock";
  this->declare_parameter("sync_clocks", false, sync_clocks_descriptor);
}

rcl_interfaces::msg::SetParametersResult CameraNode::set_params_cb(const std::vector<rclcpp::Parameter>& params)
{
  //
  // Some of our parameters can be changed on the fly, others require
  // us to reconnect to the camera or perhaps connect to a different camera.
  // If we need to reconnect to the/a camera, we force a state transition
  // here.
  //
  bool reconfigure = false;
  for (const auto& param : params)
  {
    std::string name = param.get_name();
    RCLCPP_INFO(this->logger_, "Handling param change for: %s", name.c_str());

    if (name == "timeout_millis")
    {
      this->timeout_millis_ = static_cast<int>(param.as_int());
    }
    else if (name == "timeout_tolerance_secs")
    {
      this->timeout_tolerance_secs_ = static_cast<float>(param.as_double());
    }
    else if (name == "frame_latency_thresh")
    {
      this->frame_latency_thresh_ = static_cast<float>(param.as_double());
    }
    else
    {
      RCLCPP_WARN(this->logger_, "New parameter requires reconfiguration!");
      reconfigure = true;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "OK";

  if (reconfigure)
  {
    std::thread emit_reconfigure_t([this]() {
      auto state_id = this->get_current_state().id();
      switch (state_id)
      {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
          this->cleanup();
          break;

        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
          this->deactivate();
          break;

        default:
          RCLCPP_WARN(this->logger_, "Skipping reconfiguration from state id: %d", state_id);
          break;
      }
    });
    emit_reconfigure_t.detach();
  }

  RCLCPP_INFO(this->logger_, "Set param callback OK.");
  return result;
}

void CameraNode::stop_publish_loop()
{
  if (!this->test_destroy_)
  {
    RCLCPP_INFO(this->logger_, "Stopping publishing thread...");
    this->test_destroy_ = true;
    if (this->pub_loop_.joinable())
    {
      this->pub_loop_.join();
      RCLCPP_INFO(this->logger_, "Publishing thread stopped.");
    }
    else
    {
      RCLCPP_WARN(this->logger_, "Publishing thread is not joinable!");
    }
  }
}

void CameraNode::Config(const std::shared_ptr<rmw_request_id_t>, const ConfigRequest req, const ConfigResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling config request...");

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    // XXX: may want to change this logic. For now, I do it so I know
    // the ifm3d data structures are not null pointers
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->gil_);
    resp->status = 0;
    resp->msg = "OK";

    try
    {
      this->cam_->FromJSON(json::parse(req->json));  // HERE
    }
    catch (const ifm3d::error_t& ex)
    {
      resp->status = ex.code();
      resp->msg = ex.what();
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      resp->msg = std_ex.what();
    }
    catch (...)
    {
      resp->status = -2;
      resp->msg = "Unknown error in `Config'";
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "Config: %d - %s", resp->status, resp->msg.c_str());
    }
  }

  RCLCPP_INFO(this->logger_, "Config request done.");
}

void CameraNode::Dump(const std::shared_ptr<rmw_request_id_t>, const DumpRequest, const DumpResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling dump request...");

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    // XXX: may want to change this logic. For now, I do it so I know
    // the ifm3d data structures are not null pointers
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->gil_);
    resp->status = 0;

    try
    {
      json j = this->cam_->ToJSON();  // HERE
      resp->config = j.dump();
    }
    catch (const ifm3d::error_t& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "Dump: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "Dump request done.");
}

void CameraNode::Softoff(const std::shared_ptr<rmw_request_id_t>, const SoftoffRequest, const SoftoffResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling SoftOff request...");

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->gil_);
    resp->status = 0;
    int port_arg = -1;

    try
    {
      port_arg = static_cast<int>(this->pcic_port_) % 50010;
      this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"IDLE\"}}}");
    }
    catch (const ifm3d::error_t& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "SoftOff: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "SoftOff request done.");
}

void CameraNode::Softon(const std::shared_ptr<rmw_request_id_t>, const SoftonRequest, const SoftonResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling SoftOn request...");

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->gil_);
    resp->status = 0;
    int port_arg = -1;

    try
    {
      port_arg = static_cast<int>(this->pcic_port_) % 50010;
      this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\":{\"state\":\"RUN\"}}}");
    }
    catch (const ifm3d::error_t& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "SoftOn: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "SoftOn request done.");
}

//
// Runs as a separate thread of execution, kicked off in `on_activate()`.
//
void CameraNode::publish_loop()
{
  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

  auto head = std_msgs::msg::Header();
  head.frame_id = this->camera_frame_;
  head.stamp = ros_clock.now();

  auto optical_head = std_msgs::msg::Header();
  optical_head.frame_id = this->optical_frame_;
  optical_head.stamp = head.stamp;

  pcl::PointCloud<ifm3d::PointT>::Ptr cloud(new pcl::PointCloud<ifm3d::PointT>());
  cv::Mat xyz_img;
  cv::Mat confidence_img;
  cv::Mat distance_img;
  cv::Mat amplitude_img;
  cv::Mat raw_amplitude_img;
  std::vector<float> extrinsics(6);
  cv::Mat rgb_img;
  rclcpp::Time last_frame_time = head.stamp;

  RCLCPP_INFO(this->logger_, "Starting publishing loop...");
  while (rclcpp::ok() && (!this->test_destroy_))
  {
    // create a new scope for holding the GIL
    auto processing_begin = std::chrono::system_clock::now();
    {
      std::lock_guard<std::mutex> lock(this->gil_);

      if (!this->fg_->WaitForFrame(this->im_.get(), this->timeout_millis_))
      {
        // XXX: May not want to emit this if the camera is software
        //      triggered.
        RCLCPP_WARN(this->logger_, "Timeout waiting for camera!");

        if (std::fabs((rclcpp::Time(last_frame_time, RCL_SYSTEM_TIME) - ros_clock.now()).nanoseconds() / 1e9) >
            this->timeout_tolerance_secs_)
        {
          RCLCPP_WARN(this->logger_, "Timeouts exceeded tolerance threshold!");

          std::thread deactivate_t([this]() { this->deactivate(); });
          deactivate_t.detach();
          break;
        }

        continue;
      }
      auto now = ros_clock.now();
      processing_begin = std::chrono::system_clock::now();
      auto frame_time = rclcpp::Time(
          std::chrono::duration_cast<std::chrono::nanoseconds>(this->im_->TimeStamp().time_since_epoch()).count(),
          RCL_SYSTEM_TIME);

      if (std::fabs((frame_time - now).nanoseconds() / 1e9) > this->frame_latency_thresh_)
      {
        RCLCPP_WARN_ONCE(this->logger_, "Frame latency thresh exceeded, using reception timestamps!");
        head.stamp = now;
      }
      else
      {
        head.stamp = frame_time;
      }
      optical_head.stamp = head.stamp;
      last_frame_time = head.stamp;

      //
      // pull out all the wrapped images
      //
      cloud = this->im_->Cloud();
      xyz_img = this->im_->XYZImage();
      confidence_img = this->im_->ConfidenceImage();
      distance_img = this->im_->DistanceImage();
      amplitude_img = this->im_->AmplitudeImage();
      raw_amplitude_img = this->im_->RawAmplitudeImage();
      extrinsics = this->im_->Extrinsics();
      rgb_img = this->im_->JPEGImage();

    }  // closes our GIL scope

    //
    // Publish the data
    //

    // Confidence image is invariant - no need to check the mask
    this->conf_pub_->publish(std::move(*(cv_bridge::CvImage(optical_head, "mono16", confidence_img).toImageMsg())));

    if ((this->schema_mask_ & ifm3d::IMG_CART) == ifm3d::IMG_CART)
    {
      cloud->header = pcl_conversions::toPCL(head);
      auto pc_msg = std::make_shared<PCLMsg>();
      pcl::toROSMsg(*cloud, *pc_msg);
      this->cloud_pub_->publish(std::move(*pc_msg));

      this->xyz_pub_->publish(
          std::move(*(cv_bridge::CvImage(head, xyz_img.type() == CV_32FC3 ? enc::TYPE_32FC3 : enc::TYPE_16SC3, xyz_img)
                          .toImageMsg())));
    }

    if ((this->schema_mask_ & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
    {
      this->distance_pub_->publish(std::move(
          *(cv_bridge::CvImage(optical_head, distance_img.type() == CV_32FC1 ? enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               distance_img)
                .toImageMsg())));
    }

    if ((this->schema_mask_ & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
    {
      this->amplitude_pub_->publish(std::move(
          *(cv_bridge::CvImage(optical_head, amplitude_img.type() == CV_32FC1 ? enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               amplitude_img)
                .toImageMsg())));
    }

    if ((this->schema_mask_ & ifm3d::IMG_RAMP) == ifm3d::IMG_RAMP)
    {
      this->raw_amplitude_pub_->publish(std::move(
          *(cv_bridge::CvImage(optical_head, raw_amplitude_img.type() == CV_32FC1 ? enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               raw_amplitude_img)
                .toImageMsg())));
    }

    if (!rgb_img.empty())
    {
      cv::Mat im_decode = cv::imdecode(rgb_img, cv::IMREAD_UNCHANGED);
      this->rgb_pub_->publish(std::move(*(cv_bridge::CvImage(optical_head, "bgr8", im_decode).toImageMsg())));
    }
    //
    // publish extrinsics
    //
    ifm3d_ros2::msg::Extrinsics extrinsics_msg;
    extrinsics_msg.header = optical_head;
    try
    {
      extrinsics_msg.tx = extrinsics.at(0);
      extrinsics_msg.ty = extrinsics.at(1);
      extrinsics_msg.tz = extrinsics.at(2);
      extrinsics_msg.rot_x = extrinsics.at(3);
      extrinsics_msg.rot_y = extrinsics.at(4);
      extrinsics_msg.rot_z = extrinsics.at(5);
    }
    catch (const std::out_of_range& ex)
    {
      RCLCPP_WARN(this->logger_, "Out-of-range error fetching extrinsics");
    }
    this->extrinsics_pub_->publish(std::move(extrinsics_msg));

    auto processing_latency = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - processing_begin);
    RCLCPP_INFO(this->logger_, "Processing latency: %d µs", processing_latency.count());
  }  // end: while (rclcpp::ok() && (! this->test_destroy_))

  RCLCPP_INFO(this->logger_, "Publish loop/thread exiting.");
}

}  // namespace ifm3d_ros2

#include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(ifm3d_ros2::CameraNode)
