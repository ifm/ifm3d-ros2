// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>

#include <ifm3d_ros2/imu_module.hpp>
#include <ifm3d_ros2/buffer_conversions.hpp>
#include <ifm3d_ros2/buffer_id_utils.hpp>
#include <ifm3d_ros2/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ifm3d_ros2
{
ImuModule::ImuModule(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr)
  : FunctionModule(logger), node_ptr_(node_ptr), tf_publisher_(node_ptr)
{
  RCLCPP_INFO(logger_, "ImuModule contructor called.");

  const std::string node_name(this->node_ptr_->get_name());

  RCLCPP_DEBUG(logger_, "Declaring parameter...");

  tf_publisher_.declare_parameters("imu");

  publish_averaged_data_descriptor_.name = "imu.publish_averaged_data";
  publish_averaged_data_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_averaged_data_descriptor_.description =
      "Per default (publish_averaged_data=false) an average of the chunked imu data is not published.";
  if (!node_ptr_->has_parameter(publish_averaged_data_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_averaged_data_descriptor_.name, false, publish_averaged_data_descriptor_);
  }

  publish_bulk_data_descriptor_.name = "imu.publish_bulk_data";
  publish_bulk_data_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  publish_bulk_data_descriptor_.description =
      "Per default (publish_bulk_data=true) the intermediate imu data is published as bulk message.";
  if (!node_ptr_->has_parameter(publish_bulk_data_descriptor_.name))
  {
    node_ptr_->declare_parameter(publish_bulk_data_descriptor_.name, true, publish_bulk_data_descriptor_);
  }
}

bool ImuModule::extract_imu_data(ifm3d::Frame::Ptr frame, std::vector<ImuMsg>& imu_msgs_output,
                                 TfMsg& mounting_tf_output, TfMsg& base_tf_output)
{
  imu_msgs_output.clear();

  RCLCPP_DEBUG(logger_, "Handling imu data");
  if (!frame->HasBuffer(ifm3d::buffer_id::O3R_RESULT_IMU))
  {
    RCLCPP_WARN(logger_, "ImuModule: No imu data in frame");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Deserializing imu data");

  auto imu_buffer = frame->GetBuffer(ifm3d::buffer_id::O3R_RESULT_IMU);

  // Check buffer validity
  if (imu_buffer.width() == 0 || imu_buffer.height() == 0)
  {
    RCLCPP_WARN(logger_, "Empty IMU buffer received, skipping...");
    return false;
  }

  // Convert buffer to vector<uint8_t> for deserializer
  auto* buffer_data = imu_buffer.ptr<uint8_t>(0);
  size_t buffer_size = imu_buffer.width() * imu_buffer.height();

  std::vector<uint8_t> data(buffer_data, buffer_data + buffer_size);

  const size_t data_size = data.size();
  const size_t non_sample_data_size = 4 +      // IMU verion
                                      4 +      // num saples
                                      6 * 4 +  // IMU to user
                                      6 * 4 +  // IMU to VPU
                                      8;       // imu_fifo_rcv_timestamp
  const size_t sample_size = 2 + 8 + 4 * 7;
  const size_t number_of_samples = (data_size - non_sample_data_size) / sample_size;

  RCLCPP_DEBUG(logger_, "data_size=%ld, non_sample_data_size=%ld, sample_size=%ld, number_of_samples=%ld", data_size,
               non_sample_data_size, sample_size, number_of_samples);

  if (data_size != (non_sample_data_size + sample_size * number_of_samples))
  {
    RCLCPP_WARN(logger_, "IMU buffer of unexpected layout, data might be truncated. Skipping...");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Filling the ROS message with imu data");

  // 4 byte at the beginning denote the IMU version, followed by samples
  size_t offset = 4;

  /* ***********************
   * IMU sample extraction *
   *************************/
  for (uint32_t i = 0; i < number_of_samples; i++)
  {
    uint64_t timestamp;
    float temperature;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    // sample starts with hw_timestamp entry, which is not needed and therefore skipped
    offset += 2;
    std::memcpy(&timestamp, &data[offset], 8);
    offset += 8;
    std::memcpy(&temperature, &data[offset], 4);
    offset += 4;
    std::memcpy(&accel_x, &data[offset], 4);
    offset += 4;
    std::memcpy(&accel_y, &data[offset], 4);
    offset += 4;
    std::memcpy(&accel_z, &data[offset], 4);
    offset += 4;
    std::memcpy(&gyro_x, &data[offset], 4);
    offset += 4;
    std::memcpy(&gyro_y, &data[offset], 4);
    offset += 4;
    std::memcpy(&gyro_z, &data[offset], 4);
    offset += 4;

    // Valid sample is marked by non-zero timestamp
    if (timestamp > 0)
    {
      sensor_msgs::msg::Imu imu_msg;

      imu_msg.header.frame_id = tf_publisher_.tf_sensor_link_frame_name_;
      imu_msg.header.stamp = rclcpp::Time(timestamp);
      imu_msg.orientation_covariance[0] = -1.0;  // indicating no data
      imu_msg.linear_acceleration.x = accel_x;
      imu_msg.linear_acceleration.y = accel_y;
      imu_msg.linear_acceleration.z = accel_z;
      imu_msg.angular_velocity.x = gyro_x;
      imu_msg.angular_velocity.y = gyro_y;
      imu_msg.angular_velocity.z = gyro_z;

      imu_msgs_output.push_back(imu_msg);
    }
  }

  uint32_t read_num_samples;
  std::memcpy(&read_num_samples, &data[offset], 4);
  offset += 4;
  if (read_num_samples != imu_msgs_output.size())
  {
    RCLCPP_WARN(logger_, "Mismatch between counted number of samples (%ld) and expected (%d).", imu_msgs_output.size(),
                read_num_samples);
  }

  /* **********************************
   * extrinsic_imu_to_user extraction *
   ************************************/
  float trans_x, trans_y, trans_z, rot_x, rot_y, rot_z;
  std::memcpy(&trans_x, &data[offset], 4);
  offset += 4;
  std::memcpy(&trans_y, &data[offset], 4);
  offset += 4;
  std::memcpy(&trans_z, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_x, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_y, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_z, &data[offset], 4);
  offset += 4;

  base_tf_output.translation.x = trans_x;
  base_tf_output.translation.y = trans_y;
  base_tf_output.translation.z = trans_z;
  base_tf_output.rotation = tf2::toMsg(tf_publisher_.quaternion_from_ifm_rpy(rot_x, rot_y, rot_z));

  /* *********************************
   * extrinsic_imu_to_vpu extraction *
   ***********************************/
  std::memcpy(&trans_x, &data[offset], 4);
  offset += 4;
  std::memcpy(&trans_y, &data[offset], 4);
  offset += 4;
  std::memcpy(&trans_z, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_x, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_y, &data[offset], 4);
  offset += 4;
  std::memcpy(&rot_z, &data[offset], 4);
  offset += 4;

  mounting_tf_output.translation.x = trans_x;
  mounting_tf_output.translation.y = trans_y;
  mounting_tf_output.translation.z = trans_z;
  base_tf_output.rotation = tf2::toMsg(tf_publisher_.quaternion_from_ifm_rpy(rot_x, rot_y, rot_z));

  // Parse receive timestamp

  // last 8 bytes containing imu_fifo_rcv_timestamp are skipped

  RCLCPP_DEBUG(logger_, "Imu messages ready");
  return true;
}

void ImuModule::handle_frame(ifm3d::Frame::Ptr frame)
{
  RCLCPP_DEBUG(logger_, "Handle frame");

  if (!frame)
  {
    RCLCPP_WARN(logger_, "Received invalid frame, skipping...");
    return;
  }

  std::vector<ImuMsg> imu_msgs;
  geometry_msgs::msg::Transform mounting_tf, base_tf;
  bool extraction_success = this->extract_imu_data(frame, imu_msgs, mounting_tf, base_tf);

  if (!extraction_success)
  {
    RCLCPP_WARN(logger_, "Data extracion from frame failed, skipping...");
    return;
  }

  RCLCPP_DEBUG(logger_, "Frame contains %ld imu samples.", imu_msgs.size());

  if (publish_bulk_data_)
  {
    ImuBurstMsg burst_msg;
    burst_msg.samples = imu_msgs;
    imu_burst_publisher_->publish(burst_msg);
  }

  if (publish_averaged_data_)
  {
    // Send IMU message with average sensor readings and current timestamp
    auto avg_msg = average(imu_msgs);
    average_imu_publisher_->publish(avg_msg);
  }

  // TODO tf publishing

  if (tf_publisher_.tf_publish_base_to_mounting_ || tf_publisher_.tf_publish_mounting_to_sensor_)
  {
    rclcpp::Time stamp = imu_msgs.back().header.stamp;  // Last imu message is the most current one
    tf_publisher_.update_and_publish_tf_if_changed(base_tf, mounting_tf, stamp);
  }

  RCLCPP_DEBUG(logger_, "Finished publishing imu message.");
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_configure called");
  (void)previous_state;

  tf_publisher_.parse_parameters();

  node_ptr_->get_parameter(publish_averaged_data_descriptor_.name, this->publish_averaged_data_);
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_averaged_data_descriptor_.name.c_str(),
              this->publish_averaged_data_ ? "true" : "false");

  node_ptr_->get_parameter(publish_bulk_data_descriptor_.name, this->publish_bulk_data_);
  RCLCPP_INFO(this->logger_, "Parameter %s set to '%s'", publish_bulk_data_descriptor_.name.c_str(),
              this->publish_bulk_data_ ? "true" : "false");

  const auto qos = ifm3d_ros2::LowLatencyQoS();

  if (publish_averaged_data_)
  {
    average_imu_publisher_ = node_ptr_->create_publisher<ImuMsg>("~/average_imu", qos);
  }

  if (publish_bulk_data_)
  {
    imu_burst_publisher_ = node_ptr_->create_publisher<ImuBurstMsg>("~/imu_burst", qos);
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_cleanup called");
  (void)previous_state;

  average_imu_publisher_.reset();

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_shutdown called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_activate called");
  (void)previous_state;

  if (publish_averaged_data_)
  {
    this->average_imu_publisher_->on_activate();
  }
  if (publish_bulk_data_)
  {
    this->imu_burst_publisher_->on_activate();
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_deactivate called");
  (void)previous_state;

  if (publish_averaged_data_)
  {
    this->average_imu_publisher_->on_deactivate();
  }
  if (publish_bulk_data_)
  {
    this->imu_burst_publisher_->on_deactivate();
  }

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn ImuModule::on_error(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger_, "ImuModule: on_error called");
  (void)previous_state;

  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

ImuModule::ImuMsg ImuModule::average(std::vector<ImuMsg>& messages)
{
  double lin_acc_x_sum{ 0.0 }, lin_acc_y_sum{ 0.0 }, lin_acc_z_sum{ 0.0 },  //
      ang_vel_x_sum{ 0.0 }, ang_vel_y_sum{ 0.0 }, ang_vel_z_sum{ 0.0 };
  const size_t number_of_msgs = messages.size();

  for (auto imu_msg : messages)
  {
    lin_acc_x_sum += imu_msg.linear_acceleration.x;
    lin_acc_y_sum += imu_msg.linear_acceleration.y;
    lin_acc_z_sum += imu_msg.linear_acceleration.z;
    ang_vel_x_sum += imu_msg.angular_velocity.x;
    ang_vel_y_sum += imu_msg.angular_velocity.y;
    ang_vel_z_sum += imu_msg.angular_velocity.z;
  }

  sensor_msgs::msg::Imu avg_msg;
  avg_msg.header.frame_id = tf_publisher_.tf_sensor_link_frame_name_;
  avg_msg.header.stamp = messages.back().header.stamp;  // Last message is the most current one
  avg_msg.orientation_covariance[0] = -1;
  avg_msg.angular_velocity.x = ang_vel_x_sum / number_of_msgs;
  avg_msg.angular_velocity.y = ang_vel_y_sum / number_of_msgs;
  avg_msg.angular_velocity.z = ang_vel_z_sum / number_of_msgs;
  avg_msg.linear_acceleration.x = lin_acc_x_sum / number_of_msgs;
  avg_msg.linear_acceleration.y = lin_acc_y_sum / number_of_msgs;
  avg_msg.linear_acceleration.z = lin_acc_z_sum / number_of_msgs;

  return avg_msg;
}

}  // namespace ifm3d_ros2