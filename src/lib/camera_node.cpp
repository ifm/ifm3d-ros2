/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#include <ifm3d_ros2/camera_node.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <exception>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <ifm3d_ros2/buffer_id_utils.hpp>
#include <ifm3d_ros2/qos.hpp>

#include <ifm3d/contrib/nlohmann/json.hpp>

sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                  // image.end() don't have const overloads.
                                           const std_msgs::msg::Header& header, const rclcpp::Logger& logger)
{
  static constexpr auto max_pixel_format = static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32F3);
  static constexpr auto image_format_info = [] {
    auto image_format_info = std::array<const char*, max_pixel_format + 1>{};

    {
      using namespace ifm3d;
      using namespace sensor_msgs::image_encodings;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8U)] = TYPE_8UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8S)] = TYPE_8SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U)] = TYPE_16UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16S)] = TYPE_16SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32U)] = "32UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32S)] = TYPE_32SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F)] = TYPE_32FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64U)] = "64UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64F)] = TYPE_64FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U2)] = TYPE_16UC2;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F3)] = TYPE_32FC3;
    }

    return image_format_info;
  }();

  const auto format = static_cast<std::size_t>(image.dataFormat());

  sensor_msgs::msg::Image result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = 0;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (format >= max_pixel_format)
  {
    RCLCPP_ERROR(logger, "Pixel format out of range (%ld >= %ld)", format, max_pixel_format);
    return result;
  }

  result.encoding = image_format_info.at(format);
  result.step = result.width * sensor_msgs::image_encodings::bitDepth(image_format_info.at(format)) / 8;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.step * result.height));

  if (result.encoding.empty())
  {
    RCLCPP_WARN(logger, "Can't handle encoding %ld (32U == %ld, 64U == %ld)", format,
                static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32U),
                static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_64U));
    result.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  }

  return result;
}

sensor_msgs::msg::Image ifm3d_to_ros_image(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                           const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_image(image, header, logger);
}

sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer& image,  // Need non-const image because
                                                                                       // image.begin(), image.end()
                                                                                       // don't have const overloads.
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format,  // "jpeg" or "png"
                                                                const rclcpp::Logger& logger)
{
  sensor_msgs::msg::CompressedImage result{};
  result.header = header;
  result.format = format;

  if (const auto dataFormat = image.dataFormat();
      dataFormat != ifm3d::pixel_format::FORMAT_8S && dataFormat != ifm3d::pixel_format::FORMAT_8U)
  {
    RCLCPP_ERROR(logger, "Invalid data format for %s data (%ld)", format.c_str(), static_cast<std::size_t>(dataFormat));
    return result;
  }

  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), image.width() * image.height()));
  return result;
}

sensor_msgs::msg::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer&& image,
                                                                const std_msgs::msg::Header& header,
                                                                const std::string& format, const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_compressed_image(image, header, format, logger);
}

sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                        // image.end() don't have const overloads.
                                                 const std_msgs::msg::Header& header, const rclcpp::Logger& logger)
{
  sensor_msgs::msg::PointCloud2 result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = false;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (image.dataFormat() != ifm3d::pixel_format::FORMAT_32F3 && image.dataFormat() != ifm3d::pixel_format::FORMAT_32F)
  {
    RCLCPP_ERROR(logger, "Unsupported pixel format %ld for point cloud", static_cast<std::size_t>(image.dataFormat()));
    return result;
  }

  sensor_msgs::msg::PointField x_field{};
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::msg::PointField y_field{};
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  sensor_msgs::msg::PointField z_field{};
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  result.fields = {
    x_field,
    y_field,
    z_field,
  };

  result.point_step = result.fields.size() * sizeof(float);
  result.row_step = result.point_step * result.width;
  result.is_dense = true;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.row_step * result.height));

  return result;
}

sensor_msgs::msg::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer&& image, const std_msgs::msg::Header& header,
                                                 const rclcpp::Logger& logger)
{
  return ifm3d_to_ros_cloud(image, header, logger);
}

ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)
{
  ifm3d_ros2::msg::Extrinsics extrinsics_msg;
  extrinsics_msg.header = header;
  try
  {
    extrinsics_msg.tx = buffer.at<double>(0);
    extrinsics_msg.ty = buffer.at<double>(1);
    extrinsics_msg.tz = buffer.at<double>(2);
    extrinsics_msg.rot_x = buffer.at<double>(3);
    extrinsics_msg.rot_y = buffer.at<double>(4);
    extrinsics_msg.rot_z = buffer.at<double>(5);
  }
  catch (const std::out_of_range& ex)
  {
    RCLCPP_WARN(logger, "Out-of-range error fetching extrinsics");
  }
  return extrinsics_msg;
}

ifm3d_ros2::msg::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer&& buffer, const std_msgs::msg::Header& header,
                                                const rclcpp::Logger& logger)
{
  return ifm3d_to_extrinsics(buffer, header, logger);
}

using json = ifm3d::json;
using namespace std::chrono_literals;

namespace ifm3d_ros2
{
namespace
{
constexpr auto xmlrpc_base_port = 50010;

}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions& opts) : CameraNode::CameraNode("camera", opts)
{
}

CameraNode::CameraNode(const std::string& node_name, const rclcpp::NodeOptions& opts)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", opts)
  , logger_(this->get_logger())
  , is_active_(false)
  , camera_frame_(this->get_name() + std::string("_link"))
  , optical_frame_(this->get_name() + std::string("_optical_link"))
{
  // unbuffered I/O to stdout (so we can see our log messages)
  std::setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  RCLCPP_INFO(this->logger_, "namespace: %s", this->get_namespace());
  RCLCPP_INFO(this->logger_, "node name: %s", this->get_name());
  RCLCPP_INFO(this->logger_, "middleware: %s", rmw_get_implementation_identifier());
  RCLCPP_INFO(this->logger_, "camera frame: %s", this->camera_frame_.c_str());
  RCLCPP_INFO(this->logger_, "optical frame: %s", this->optical_frame_.c_str());

  // declare our parameters and default values -- parameters defined in
  // the passed in `opts` (via __params:=/path/to/params.yaml on cmd line)
  // will override our default values specified.
  this->init_params();
  set_params_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ifm3d_ros2::CameraNode::set_params_cb, this, std::placeholders::_1));

  RCLCPP_INFO(this->logger_, "After the parameters declaration");

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

  this->get_parameter("pcic_port", this->pcic_port_);
  RCLCPP_INFO(this->logger_, "pcic_port: %u", this->pcic_port_);

  this->get_parameter("ip", this->ip_);
  RCLCPP_INFO(this->logger_, "ip: %s", this->ip_.c_str());

  this->get_parameter("xmlrpc_port", this->xmlrpc_port_);
  RCLCPP_INFO(this->logger_, "xmlrpc_port: %u", this->xmlrpc_port_);

  this->get_parameter("password", this->password_);
  RCLCPP_INFO(this->logger_, "password: %s", std::string(this->password_.size(), '*').c_str());

  std::vector<std::string> buffer_id_strings;
  this->get_parameter("buffer_id_list", buffer_id_strings);
  RCLCPP_INFO(this->logger_, "Reading %ld buffer_ids: [%s]", buffer_id_strings.size(),
              buffer_id_utils::vector_to_string(buffer_id_strings).c_str());
  // Populate buffer_id_list_ from read strings
  this->buffer_id_list_.clear();
  for (const std::string& string : buffer_id_strings)
  {
    ifm3d::buffer_id found_id;
    if (buffer_id_utils::convert(string, found_id))
    {
      this->buffer_id_list_.push_back(found_id);
    }
    else
    {
      RCLCPP_WARN(this->logger_, "Ignoring unknown buffer_id %s", string.c_str());
    }
  }
  RCLCPP_INFO(this->logger_, "Parsed %ld buffer_ids: %s", this->buffer_id_list_.size(),
              buffer_id_utils::vector_to_string(this->buffer_id_list_).c_str());

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
  // Set up our publishers.
  //
  this->initialize_publishers();
  RCLCPP_INFO(this->logger_, "After publishers declaration");

  //
  // We need a global lock on all the ifm3d core data structures
  //
  std::lock_guard<std::mutex> lock(this->gil_);

  //
  // Initialize the camera interface
  //

  RCLCPP_INFO(this->logger_, "Initializing camera...");
  this->cam_ = ifm3d::Device::MakeShared(this->ip_, this->xmlrpc_port_, this->password_);
  RCLCPP_INFO(this->logger_, "Initializing FrameGrabber");
  this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, this->pcic_port_);

  RCLCPP_INFO(this->logger_, "Configuration complete.");
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_activate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  //  activate all publishers
  RCLCPP_INFO(this->logger_, "Activating publishers...");
  this->activate_publishers();
  RCLCPP_INFO(this->logger_, "Publishers activated.");

  // Start the Framegrabber, needs a BufferList (a vector of std::variant)
  RCLCPP_INFO(this->logger_, "Starting the Framegrabber...");
  std::vector<std::variant<long unsigned int, int, ifm3d::buffer_id>> buffer_list{};
  buffer_list.insert(buffer_list.end(), buffer_id_list_.begin(), buffer_id_list_.end());
  // Start framegrabber and wait for the returned future
  this->fg_->Start(buffer_list).wait();
  // Register Callbacks to handle new frames and print errors
  this->fg_->OnNewFrame(std::bind(&CameraNode::frame_callback, this, std::placeholders::_1));
  this->fg_->OnError(std::bind(&CameraNode::error_callback, this, std::placeholders::_1));
  this->is_active_ = true;
  RCLCPP_INFO(this->logger_, "Framegrabber started.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(this->logger_, "Stopping publishing...");
  this->is_active_ = false;

  // explicitly deactive the publishers
  RCLCPP_INFO(this->logger_, "Deactivating publishers...");
  this->deactivate_publishers();
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
  // this->im_.reset();
  this->fg_.reset();
  this->cam_.reset();

  RCLCPP_INFO(this->logger_, "Node cleanup complete.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_shutdown(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  this->is_active_ = false;

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL CameraNode::on_error(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_error(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  this->is_active_ = false;

  std::lock_guard<std::mutex> lock(this->gil_);
  RCLCPP_INFO(this->logger_, "Resetting core ifm3d data structures...");
  // this->im_.reset();
  this->fg_.reset();
  this->cam_.reset();

  RCLCPP_INFO(this->logger_, "Error processing complete.");

  return TC_RETVAL::SUCCESS;
}

void CameraNode::init_params()
{
  static constexpr auto default_timeout_millis{ 500 };
  static constexpr auto default_timeout_tolerance_secs{ 5.0 };
  static constexpr auto default_frame_latency_threshold{ 1.0 };
  static constexpr auto default_sync_clocks{ false };
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

  rcl_interfaces::msg::ParameterDescriptor buffer_id_list_descriptor;
  const std::vector<std::string> default_buffer_id_list{
    //
    "AMPLITUDE_IMAGE",        //
    "NORM_AMPLITUDE_IMAGE",   //
    "CONFIDENCE_IMAGE",       //
    "JPEG_IMAGE",             //
    "RADIAL_DISTANCE_IMAGE",  //
    "XYZ",                    //
    "EXTRINSIC_CALIB",        //
  };
  buffer_id_list_descriptor.name = "buffer_id_list";
  buffer_id_list_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  buffer_id_list_descriptor.description = "List of buffer_id strings denoting the wanted buffers.";

  //
  // XXX: add an IntegerRange constraint here
  //
  this->declare_parameter("buffer_id_list", default_buffer_id_list, buffer_id_list_descriptor);

  rcl_interfaces::msg::ParameterDescriptor timeout_millis_descriptor;
  timeout_millis_descriptor.name = "timeout_millis";
  timeout_millis_descriptor.type =  // long (signed)
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  timeout_millis_descriptor.description = "How long to block for a single frame from the camera (millis)";
  timeout_millis_descriptor.additional_constraints = "A timeout <= 0 will block indefinitely";
  this->declare_parameter("timeout_millis", default_timeout_millis, timeout_millis_descriptor);

  rcl_interfaces::msg::ParameterDescriptor timeout_tolerance_secs_descriptor;
  timeout_tolerance_secs_descriptor.name = "timeout_tolerance_secs";
  timeout_tolerance_secs_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  timeout_tolerance_secs_descriptor.description = "Time (seconds) without a frame to consider the camera disconnected";
  this->declare_parameter("timeout_tolerance_secs", default_timeout_tolerance_secs, timeout_tolerance_secs_descriptor);

  rcl_interfaces::msg::ParameterDescriptor frame_latency_thresh_descriptor;
  frame_latency_thresh_descriptor.name = "frame_latency_thresh";
  frame_latency_thresh_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  frame_latency_thresh_descriptor.description = "Threshold (seconds) for determining use of acq time vs rcv time";
  this->declare_parameter("frame_latency_thresh", default_frame_latency_threshold, frame_latency_thresh_descriptor);

  rcl_interfaces::msg::ParameterDescriptor sync_clocks_descriptor;
  sync_clocks_descriptor.name = "sync_clocks";
  sync_clocks_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  sync_clocks_descriptor.description = "Attempt to sync host and camera clock";
  this->declare_parameter("sync_clocks", default_sync_clocks, sync_clocks_descriptor);
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
    const std::string& name = param.get_name();
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

void CameraNode::initialize_publishers()
{
  using namespace buffer_id_utils;

  image_publishers_.clear();
  compressed_image_publishers_.clear();
  pcl_publishers_.clear();
  extrinsics_publishers_.clear();

  std::vector<ifm3d::buffer_id> ids_to_remove{};

  for (const ifm3d::buffer_id& id : this->buffer_id_list_)
  {
    // Create Publishers in node namespace to make multi-camera setups easier
    const std::string topic_name = "~/" + buffer_id_utils::topic_name_map[id];
    const auto qos = ifm3d_ros2::LowLatencyQoS();

    if (std::find(image_ids.begin(), image_ids.end(), id) != image_ids.end())
    {
      // buffer of Type ImageMsg
      image_publishers_[id] = this->create_publisher<ImageMsg>(topic_name, qos);
    }

    else if (std::find(compressed_image_ids.begin(), compressed_image_ids.end(), id) != compressed_image_ids.end())
    {
      // buffer of Type CompressedImageMsg
      compressed_image_publishers_[id] = this->create_publisher<CompressedImageMsg>(topic_name, qos);
    }

    else if (std::find(plc_ids.begin(), plc_ids.end(), id) != plc_ids.end())
    {
      // buffer of Type PCLMsg
      pcl_publishers_[id] = this->create_publisher<PCLMsg>(topic_name, qos);
    }
    else if (std::find(extrinsics_ids.begin(), extrinsics_ids.end(), id) != extrinsics_ids.end())
    {
      // buffer of Type ExtrinsicsMsg
      extrinsics_publishers_[id] = this->create_publisher<ExtrinsicsMsg>(topic_name, qos);
    }

    else
    {
      std::string id_string;
      convert(id, id_string);
      RCLCPP_ERROR(logger_, "Unknown message type for buffer_id %s. Will be removed from list...", id_string.c_str());
      ids_to_remove.push_back(id);
    }
  }

  while (ids_to_remove.size() > 0)
  {
    ifm3d::buffer_id id_to_remove = ids_to_remove.back();
    ids_to_remove.pop_back();
    std::vector<ifm3d::buffer_id>::iterator itr =
        std::find(buffer_id_list_.begin(), buffer_id_list_.end(), id_to_remove);
    auto index = std::distance(buffer_id_list_.begin(), itr);
    buffer_id_list_.erase(buffer_id_list_.begin() + index);

    std::string id_string;
    convert(id_to_remove, id_string);
    RCLCPP_INFO(logger_, "Removed buffer_id %s from list at position %ld", id_string.c_str(), index);
  }
}

void CameraNode::activate_publishers()
{
  for (auto& [id, publisher] : image_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : compressed_image_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : pcl_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : extrinsics_publishers_)
  {
    publisher->on_activate();
  }
};

void CameraNode::deactivate_publishers()
{
  for (auto& [id, publisher] : image_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : compressed_image_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : pcl_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : extrinsics_publishers_)
  {
    publisher->on_deactivate();
  }
};

void CameraNode::Config(const std::shared_ptr<rmw_request_id_t> /*unused*/, ConfigRequest req, ConfigResponse resp)
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
    catch (const ifm3d::Error& ex)
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

void CameraNode::Dump(const std::shared_ptr<rmw_request_id_t> /*unused*/, DumpRequest /*unused*/, DumpResponse resp)
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
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "%s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "%s", std_ex.what());
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

void CameraNode::Softoff(const std::shared_ptr<rmw_request_id_t> /*unused*/, SoftoffRequest /*unused*/,
                         SoftoffResponse resp)
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
      port_arg = static_cast<int>(this->pcic_port_) % xmlrpc_base_port;
      this->cam_->FromJSONStr(R"({"ports":{"port)" + std::to_string(port_arg) + R"(": {"state": "IDLE"}}})");
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "%s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "%s", std_ex.what());
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

void CameraNode::Softon(const std::shared_ptr<rmw_request_id_t> /*unused*/, SoftonRequest /*unused*/,
                        SoftonResponse resp)
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
      port_arg = static_cast<int>(this->pcic_port_) % xmlrpc_base_port;
      this->cam_->FromJSONStr(R"({"ports":{"port)" + std::to_string(port_arg) + R"(":{"state":"RUN"}}})");
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "%s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "%s", std_ex.what());
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

void CameraNode::frame_callback(ifm3d::Frame::Ptr frame)
{
  using namespace buffer_id_utils;

  RCLCPP_INFO_ONCE(logger_, "Receiving Frames. Processing buffer for [%s]...",
                   buffer_id_utils::vector_to_string(this->buffer_id_list_).c_str());
  RCLCPP_DEBUG(logger_, "Received new Frame.");

  // Ignore new frames if node not in ACTIVE state
  if (!this->is_active_)
  {
    RCLCPP_INFO(logger_, "Node inactive, ignoring new Frame.");
    return;
  }

  rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

  auto head = std_msgs::msg::Header();
  head.frame_id = this->camera_frame_;
  head.stamp = ros_clock.now();

  auto optical_head = std_msgs::msg::Header();
  optical_head.frame_id = this->optical_frame_;
  optical_head.stamp = head.stamp;

  rclcpp::Time last_frame_time = head.stamp;

  // TODO(CE) mutex still needed?
  std::lock_guard<std::mutex> lock(this->gil_);

  for (const ifm3d::buffer_id& id : this->buffer_id_list_)
  {
    // Helper for logging
    auto& clk = *this->get_clock();
    std::string id_string;
    convert(id, id_string);

    if (std::find(image_ids.begin(), image_ids.end(), id) != image_ids.end())
    {
      if (frame->HasBuffer(id))
      {
        auto buffer = frame->GetBuffer(id);
        ImageMsg msg = ifm3d_to_ros_image(frame->GetBuffer(id), optical_head, logger_);
        image_publishers_[id]->publish(msg);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(logger_, clk, 5000,
                             "Frame does not contain buffer %s. Is the correct camera head connected?",
                             id_string.c_str());
      }
    }

    else if (std::find(compressed_image_ids.begin(), compressed_image_ids.end(), id) != compressed_image_ids.end())
    {
      if (frame->HasBuffer(id))
      {
        auto buffer = frame->GetBuffer(id);
        CompressedImageMsg msg = ifm3d_to_ros_compressed_image(frame->GetBuffer(id), optical_head, "jpeg", logger_);
        compressed_image_publishers_[id]->publish(msg);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(logger_, clk, 5000,
                             "Frame does not contain buffer %s. Is the correct camera head connected?",
                             id_string.c_str());
      }
    }

    else if (std::find(plc_ids.begin(), plc_ids.end(), id) != plc_ids.end())
    {
      if (frame->HasBuffer(id))
      {
        auto buffer = frame->GetBuffer(id);
        PCLMsg msg = ifm3d_to_ros_cloud(frame->GetBuffer(id), optical_head, logger_);
        pcl_publishers_[id]->publish(msg);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(logger_, clk, 5000,
                             "Frame does not contain buffer %s. Is the correct camera head connected?",
                             id_string.c_str());
      }
    }
    else if (std::find(extrinsics_ids.begin(), extrinsics_ids.end(), id) != extrinsics_ids.end())
    {
      if (frame->HasBuffer(id))
      {
        auto buffer = frame->GetBuffer(id);
        ExtrinsicsMsg msg = ifm3d_to_extrinsics(buffer, optical_head, logger_);
        extrinsics_publishers_[id]->publish(msg);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(logger_, clk, 5000,
                             "Frame does not contain buffer %s. Is the correct camera head connected?",
                             id_string.c_str());
      }
    }

    else
    {
      RCLCPP_ERROR_THROTTLE(logger_, clk, 5000, "Unknown message type for buffer_id %s. Can not publish.",
                            id_string.c_str());
    }
  }

  RCLCPP_DEBUG(this->logger_, "Publish loop exiting.");
}

void CameraNode::error_callback(const ifm3d::Error& error)
{
  RCLCPP_ERROR(logger_, "Error received from ifm3d: %s", error.what());
}

void CameraNode::async_error_callback(int i, const std::string& s)
{
  RCLCPP_ERROR(logger_, "AsyncError received from ifm3d: %d %s", i, s.c_str());
}

void CameraNode::async_notification_callback(const std::string& s1, const std::string& s2)
{
  RCLCPP_INFO(logger_, "AsyncNotification received from ifm3d: %s  |  %s", s1.c_str(), s2.c_str());
}

}  // namespace ifm3d_ros2

#include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(ifm3d_ros2::CameraNode)
