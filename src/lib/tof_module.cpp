// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d_ros2/buffer_conversions.hpp>
#include <ifm3d_ros2/buffer_id_utils.hpp>
#include <ifm3d_ros2/tof_module.hpp>
#include <ifm3d_ros2/qos.hpp>

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/deserialize/struct_tof_info_v4.hpp>

namespace ifm3d_ros2
{
TofModule::TofModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                     ifm3d::O3R::Ptr o3r_ptr, std::string port, uint32_t width, uint32_t height)
  : FunctionModule(logger), node_ptr_(node_ptr), tf_publisher_(node_ptr, o3r_ptr, port), width_(width), height_(height)
{
  RCLCPP_INFO(logger_, "TofModule contructor called.");

  RCLCPP_INFO(this->logger_, "Declaring parameters...");
  const std::string node_name(this->node_ptr_->get_name());
  const std::vector<std::string> default_buffer_id_list{
    "CONFIDENCE_IMAGE",       //
    "EXTRINSIC_CALIB",        //
    "NORM_AMPLITUDE_IMAGE",   //
    "RADIAL_DISTANCE_IMAGE",  //
    "TOF_INFO",
    "XYZ",  //
  };

  buffer_id_list_descriptor_.name = "buffer_id_list";
  buffer_id_list_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  buffer_id_list_descriptor_.description = "List of buffer_id strings denoting the wanted buffers.";
  this->node_ptr_->declare_parameter(buffer_id_list_descriptor_.name, default_buffer_id_list,
                                     buffer_id_list_descriptor_);

  tf_base_frame_name_descriptor_.name = "tf.base_frame_name";
  tf_base_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_base_frame_name_descriptor_.description = "Name for the ifm base frame, defaults to ifm_base_link.";
  this->node_ptr_->declare_parameter(tf_base_frame_name_descriptor_.name, "ifm_base_link",
                                     tf_base_frame_name_descriptor_);

  tf_mounting_frame_name_descriptor_.name = "tf.mounting_frame_name";
  tf_mounting_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_mounting_frame_name_descriptor_.description =
      "Name for the mounting point frame, defaults to <node_name>_mounting_link.";
  this->node_ptr_->declare_parameter(tf_mounting_frame_name_descriptor_.name, node_name + "_mounting_link",
                                     tf_mounting_frame_name_descriptor_);

  tf_optical_frame_name_descriptor_.name = "tf.optical_frame_name";
  tf_optical_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_optical_frame_name_descriptor_.description =
      "Name for the point optical frame, defaults to <node_name>_optical_link.";
  this->node_ptr_->declare_parameter(tf_optical_frame_name_descriptor_.name, node_name + "_optical_link",
                                     tf_optical_frame_name_descriptor_);

  tf_publish_base_to_mounting_descriptor_.name = "tf.publish_base_to_mounting";
  tf_publish_base_to_mounting_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  tf_publish_base_to_mounting_descriptor_.description =
      "Whether the transform from the ifm base link to the mounting point should be published.";
  this->node_ptr_->declare_parameter(tf_publish_base_to_mounting_descriptor_.name, true,
                                     tf_publish_base_to_mounting_descriptor_);

  tf_publish_mounting_to_optical_descriptor_.name = "tf.publish_mounting_to_optical";
  tf_publish_mounting_to_optical_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  tf_publish_mounting_to_optical_descriptor_.description =
      "Whether the transform from the mounting point to the optical frame should be published.";
  this->node_ptr_->declare_parameter(tf_publish_mounting_to_optical_descriptor_.name, true,
                                     tf_publish_mounting_to_optical_descriptor_);
  RCLCPP_INFO(this->logger_, "After the parameters declaration");

  this->first_ = true;
}

void TofModule::handle_frame(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handle TOF frame");

  using namespace buffer_id_utils;

  RCLCPP_INFO_ONCE(logger_, "Receiving Frames. Processing buffer for [%s]...",
                   buffer_id_utils::vector_to_string(this->buffer_id_list_).c_str());
  RCLCPP_DEBUG(logger_, "Received new Frame.");

  rclcpp::Time frame_ts = ifm3d_ros2::ifm3d_to_ros_time(frame->TimeStamps()[0]);
  RCLCPP_DEBUG(logger_, "Frame timestamp: %f", frame_ts.seconds());

  auto cloud_header = std_msgs::msg::Header();
  cloud_header.frame_id = tf_publisher_.tf_base_link_frame_name_;
  cloud_header.stamp = frame_ts;

  auto optical_header = std_msgs::msg::Header();
  optical_header.frame_id = tf_publisher_.tf_optical_link_frame_name_;
  optical_header.stamp = frame_ts;

  for (const ifm3d::buffer_id& id : this->buffer_id_list_)
  {
    // Helper for logging
    auto& clk = *this->node_ptr_->get_clock();
    std::string id_string;
    if (!buffer_id_utils::convert(id, id_string)){
      RCLCPP_ERROR(logger_, "Cannot convert the buffer id to a string");
    }

    const buffer_id_utils::message_type message_type = buffer_id_utils::message_type_map[id];

    if (!frame->HasBuffer(id))
    {
      RCLCPP_WARN_THROTTLE(logger_, clk, 5000,
                           "Frame does not contain buffer %s. Is the correct camera head connected?",
                           id_string.c_str());
    }

    switch (message_type)
    {
      case buffer_id_utils::message_type::raw_image: {
        auto buffer = frame->GetBuffer(id);
        ImageMsg raw_image_msg = ifm3d_ros2::ifm3d_to_ros_image(frame->GetBuffer(id), optical_header, logger_);
        image_publishers_[id]->publish(raw_image_msg);
      }
      break;

      case buffer_id_utils::message_type::pointcloud: {
        auto buffer = frame->GetBuffer(id);
        PCLMsg pointcloud_msg = ifm3d_ros2::ifm3d_to_ros_cloud(frame->GetBuffer(id), cloud_header, logger_);
        pcl_publishers_[id]->publish(pointcloud_msg);
      }
      break;
      case buffer_id_utils::message_type::extrinsics: {
        auto buffer = frame->GetBuffer(id);
        ExtrinsicsMsg extrinsics_msg = ifm3d_ros2::ifm3d_to_extrinsics(buffer, optical_header, logger_);
        extrinsics_publishers_[id]->publish(extrinsics_msg);
      }
      break;
      case buffer_id_utils::intrinsics: {
        auto buffer = frame->GetBuffer(id);
        IntrinsicsMsg intrinsics_msg = ifm3d_ros2::ifm3d_to_intrinsics(buffer, optical_header, logger_);
        intrinsics_publishers_[id]->publish(intrinsics_msg);
      }
      break;
      case buffer_id_utils::message_type::inverse_intrinsics: {
        auto buffer = frame->GetBuffer(id);
        InverseIntrinsicsMsg inverse_intrinsics_msg =
            ifm3d_ros2::ifm3d_to_inverse_intrinsics(buffer, optical_header, logger_);
        inverse_intrinsics_publishers_[id]->publish(inverse_intrinsics_msg);
      }
      break;
      case buffer_id_utils::message_type::tof_info: {
        auto buffer = frame->GetBuffer(id);
        TOFInfoMsg tof_info_msg = ifm3d_ros2::ifm3d_to_tof_info(buffer, optical_header, logger_);
        tof_info_publishers_[id]->publish(tof_info_msg);

        if (tf_publisher_.tf_publish_mounting_to_optical_ || tf_publisher_.tf_publish_base_to_mounting_)
        {
          geometry_msgs::msg::TransformStamped tf_base_to_optical;
          if (ifm3d_ros2::ifm3d_tof_info_to_optical_mount_link(buffer, tf_publisher_.tf_base_link_frame_name_,
                                                               tf_publisher_.tf_optical_link_frame_name_, logger_,
                                                               tf_base_to_optical))
          {
            tf_publisher_.update_and_publish_tf_if_changed(tf_base_to_optical);
          }
          else
          {
            RCLCPP_ERROR(logger_, "Failed to derive transform from tof_info, retrying...");
          }
        }

        CameraInfoMsg camera_info_msg;
        if (ifm3d_ros2::ifm3d_tof_info_to_camera_info(buffer, optical_header, this->height_, this->width_, logger_,
                                                      camera_info_msg))
        {
          RCLCPP_INFO_ONCE(logger_, "Parsing CameraInfo successfull.");
          camera_info_publishers_[id]->publish(camera_info_msg);
        }
        else
        {
          RCLCPP_ERROR(logger_, "Failed to retrieve camera_info from rgb_info buffer.");
        };
      }
      break;
      default:
        RCLCPP_ERROR_THROTTLE(logger_, clk, 5000, "Unknown message type for buffer_id %s. Can not publish.",
                              id_string.c_str());
        break;
    }
  }
  RCLCPP_DEBUG(logger_, "Finished publishing tof messages.");
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_configure called");
  (void)previous_state;

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

  using namespace buffer_id_utils;

  this->image_publishers_.clear();
  this->pcl_publishers_.clear();
  this->extrinsics_publishers_.clear();
  this->camera_info_publishers_.clear();
  this->tof_info_publishers_.clear();
  this->intrinsics_publishers_.clear();
  this->inverse_intrinsics_publishers_.clear();

  // Remove buffer_ids unfit for the given data type
  this->buffer_id_list_ =
      buffer_id_utils::buffer_ids_for_data_stream_type(this->buffer_id_list_, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d);
  RCLCPP_INFO(logger_, "After removing buffer_ids unfit for the given data stream type, the final list is: [%s].",
              buffer_id_utils::vector_to_string(this->buffer_id_list_).c_str());

  std::vector<ifm3d::buffer_id> ids_to_remove{};
  // Create correctly typed publishers for all given buffer_ids
  for (const ifm3d::buffer_id& id : this->buffer_id_list_)
  {
    // Create Publishers in node namespace to make multi-camera setups easier
    const std::string topic_name = "~/" + buffer_id_utils::topic_name_map[id];
    const auto qos = ifm3d_ros2::LowLatencyQoS();
    const buffer_id_utils::message_type message_type = buffer_id_utils::message_type_map[id];

    switch (message_type)
    {
      case buffer_id_utils::message_type::raw_image:
        image_publishers_[id] = node_ptr_->create_publisher<ImageMsg>(topic_name, qos);
        break;
      case buffer_id_utils::message_type::pointcloud:
        pcl_publishers_[id] = node_ptr_->create_publisher<PCLMsg>(topic_name, qos);
        break;
      case buffer_id_utils::message_type::extrinsics:
        extrinsics_publishers_[id] = node_ptr_->create_publisher<ExtrinsicsMsg>(topic_name, qos);
        break;
      case buffer_id_utils::message_type::intrinsics:
        intrinsics_publishers_[id] = node_ptr_->create_publisher<IntrinsicsMsg>(topic_name, qos);
        break;
      case buffer_id_utils::message_type::tof_info:
        tof_info_publishers_[id] = node_ptr_->create_publisher<TOFInfoMsg>(topic_name, qos);
        camera_info_publishers_[id] = node_ptr_->create_publisher<CameraInfoMsg>("~/camera_info", qos);
        break;
      case buffer_id_utils::message_type::inverse_intrinsics:
        inverse_intrinsics_publishers_[id] = node_ptr_->create_publisher<InverseIntrinsicsMsg>(topic_name, qos);
        break;
      default:
        std::string id_string;
        buffer_id_utils::convert(id, id_string);
        RCLCPP_ERROR(logger_, "Unknown message type for buffer_id %s. Will be removed from list...", id_string.c_str());
        ids_to_remove.push_back(id);
        break;
    }
  }
  RCLCPP_INFO(this->logger_, "Created publishers for all buffer_ids.");

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_cleanup called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_shutdown called");
  (void)previous_state;
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_activate called");
  (void)previous_state;

  for (auto& [id, publisher] : this->image_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->pcl_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->extrinsics_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->camera_info_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->tof_info_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->intrinsics_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->inverse_intrinsics_publishers_)
  {
    publisher->on_activate();
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_deactivate called");
  (void)previous_state;

  for (auto& [id, publisher] : this->image_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : this->pcl_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : this->extrinsics_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : this->camera_info_publishers_)
  {
    publisher->on_activate();
  }
  for (auto& [id, publisher] : this->tof_info_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : this->intrinsics_publishers_)
  {
    publisher->on_deactivate();
  }
  for (auto& [id, publisher] : this->inverse_intrinsics_publishers_)
  {
    publisher->on_deactivate();
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn TofModule::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "TofModule: on_error called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void TofModule::parse_params()
{
  std::vector<std::string> buffer_id_strings;
  this->node_ptr_->get_parameter(buffer_id_list_descriptor_.name, buffer_id_strings);
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

  this->node_ptr_->get_parameter(tf_base_frame_name_descriptor_.name, tf_publisher_.tf_base_link_frame_name_);
  RCLCPP_INFO(this->logger_, "tf.base_link.frame_name: %s", tf_publisher_.tf_base_link_frame_name_.c_str());

  this->node_ptr_->get_parameter(tf_mounting_frame_name_descriptor_.name, tf_publisher_.tf_mounting_link_frame_name_);
  RCLCPP_INFO(this->logger_, "tf.mounting_link.frame_name: %s", tf_publisher_.tf_mounting_link_frame_name_.c_str());

  this->node_ptr_->get_parameter(tf_optical_frame_name_descriptor_.name, tf_publisher_.tf_optical_link_frame_name_);
  RCLCPP_INFO(this->logger_, "tf.optical_link.frame_name: %s", tf_publisher_.tf_optical_link_frame_name_.c_str());

  this->node_ptr_->get_parameter(tf_publish_base_to_mounting_descriptor_.name,
                                 tf_publisher_.tf_publish_base_to_mounting_);

  this->node_ptr_->get_parameter(tf_publish_mounting_to_optical_descriptor_.name,
                                 tf_publisher_.tf_publish_mounting_to_optical_);

  RCLCPP_INFO(this->logger_, "tf.optical_link.publish_transform: %s",
              tf_publisher_.tf_publish_mounting_to_optical_ ? "true" : "false");
}

void TofModule::set_parameter_event_callbacks()
{
  // Create a parameter subscriber that can be used to monitor parameter changes
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this->node_ptr_);

  auto buffer_id_list_cb = [this](const rclcpp::Parameter& p) {
    RCLCPP_WARN(logger_, "This new buffer_id_list will be used after CONFIGURE transition was called: %s",
                buffer_id_utils::vector_to_string(p.as_string_array()).c_str());
  };
  registered_param_callbacks_[buffer_id_list_descriptor_.name] =
      param_subscriber_->add_parameter_callback(buffer_id_list_descriptor_.name, buffer_id_list_cb);

  auto tf_mounting_link_frame_name_cb = [this](const rclcpp::Parameter& p) {
    tf_publisher_.tf_mounting_link_frame_name_ = p.as_string();
    RCLCPP_INFO(logger_, "New tf.mounting_link.frame_name: '%s'", tf_publisher_.tf_mounting_link_frame_name_.c_str());
  };
  registered_param_callbacks_[tf_mounting_frame_name_descriptor_.name] = param_subscriber_->add_parameter_callback(
      tf_mounting_frame_name_descriptor_.name, tf_mounting_link_frame_name_cb);

  auto tf_optical_link_frame_name_cb = [this](const rclcpp::Parameter& p) {
    tf_publisher_.tf_optical_link_frame_name_ = p.as_string();
    RCLCPP_INFO(logger_, "New tf.optical_link.frame_name: '%s'", tf_publisher_.tf_optical_link_frame_name_.c_str());
  };
  registered_param_callbacks_[tf_optical_frame_name_descriptor_.name] =
      param_subscriber_->add_parameter_callback(tf_optical_frame_name_descriptor_.name, tf_optical_link_frame_name_cb);

  auto tf_optical_link_publish_transform_cb = [this](const rclcpp::Parameter& p) {
    tf_publisher_.tf_publish_mounting_to_optical_ = p.as_bool();
    RCLCPP_INFO(logger_, "New tf.optical_link.publish_transform: %s",
                tf_publisher_.tf_publish_mounting_to_optical_ ? "true" : "false");
  };
  registered_param_callbacks_[tf_publish_mounting_to_optical_descriptor_.name] =
      param_subscriber_->add_parameter_callback(tf_publish_mounting_to_optical_descriptor_.name,
                                                tf_optical_link_publish_transform_cb);
}

}  // namespace ifm3d_ros2