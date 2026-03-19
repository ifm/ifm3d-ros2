#include <ifm3d_ros2/sensor_tf_publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace ifm3d_ros2
{

SensorTfPublisher::SensorTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, std::string base_frame_name,
                                     std::string mounting_frame_name, std::string optical_frame_name)
  : tf_base_link_frame_name_(base_frame_name)
  , tf_mounting_link_frame_name_(mounting_frame_name)
  , tf_sensor_link_frame_name_(optical_frame_name)
  , tf_publish_mounting_to_sensor_(true)
  , tf_publish_base_to_mounting_(true)
  , node_ptr_(node_ptr)
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->node_ptr_);
}

SensorTfPublisher::SensorTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr)
  : SensorTfPublisher(node_ptr, "", "", "")
{
}

geometry_msgs::msg::TransformStamped SensorTfPublisher::calculate_tf_mounting_to_sensor(
    builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_optical,
    geometry_msgs::msg::TransformStamped tf_base_to_mounting) const
{
  const tf2::Vector3 vector_mounting_to_optical_in_base(
      tf_base_to_optical.transform.translation.x - tf_base_to_mounting.transform.translation.x,
      tf_base_to_optical.transform.translation.y - tf_base_to_mounting.transform.translation.y,
      tf_base_to_optical.transform.translation.z - tf_base_to_mounting.transform.translation.z);

  const tf2::Quaternion quad_base_to_mounting(
      tf_base_to_mounting.transform.rotation.x, tf_base_to_mounting.transform.rotation.y,
      tf_base_to_mounting.transform.rotation.z, tf_base_to_mounting.transform.rotation.w);

  const tf2::Quaternion quad_base_to_optical(
      tf_base_to_optical.transform.rotation.x, tf_base_to_optical.transform.rotation.y,
      tf_base_to_optical.transform.rotation.z, tf_base_to_optical.transform.rotation.w);

  tf2::Vector3 vector_mounting_to_optical_in_mounting =
      tf2::quatRotate(quad_base_to_mounting.inverse(), vector_mounting_to_optical_in_base);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_mounting_link_frame_name_;
  tf.child_frame_id = tf_sensor_link_frame_name_;
  tf.transform.translation.x = vector_mounting_to_optical_in_mounting.x();
  tf.transform.translation.y = vector_mounting_to_optical_in_mounting.y();
  tf.transform.translation.z = vector_mounting_to_optical_in_mounting.z();

  const tf2::Quaternion quad_mounting_to_optical(quad_base_to_optical * quad_base_to_mounting.inverse());
  tf.transform.rotation.x = quad_mounting_to_optical.x();
  tf.transform.rotation.y = quad_mounting_to_optical.y();
  tf.transform.rotation.z = quad_mounting_to_optical.z();
  tf.transform.rotation.w = quad_mounting_to_optical.w();

  return tf;
}

geometry_msgs::msg::TransformStamped SensorTfPublisher ::calculate_tf_base_to_sensor(
    builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_mounting,
    geometry_msgs::msg::TransformStamped tf_mounting_to_optical) const
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_base_link_frame_name_;
  tf.child_frame_id = tf_sensor_link_frame_name_;

  const tf2::Vector3 vector_base_to_mounting_in_base(tf_base_to_mounting.transform.translation.x,
                                                     tf_base_to_mounting.transform.translation.y,
                                                     tf_base_to_mounting.transform.translation.z);
  const tf2::Vector3 vector_mounting_to_optical_in_mounting(tf_mounting_to_optical.transform.translation.x,
                                                            tf_mounting_to_optical.transform.translation.y,
                                                            tf_mounting_to_optical.transform.translation.z);

  const tf2::Quaternion quad_base_to_mounting(
      tf_base_to_mounting.transform.rotation.x, tf_base_to_mounting.transform.rotation.y,
      tf_base_to_mounting.transform.rotation.z, tf_base_to_mounting.transform.rotation.w);

  const tf2::Quaternion quad_mounting_to_optical(
      tf_mounting_to_optical.transform.rotation.x, tf_mounting_to_optical.transform.rotation.y,
      tf_mounting_to_optical.transform.rotation.z, tf_mounting_to_optical.transform.rotation.w);

  const tf2::Vector3 vector_mounting_to_optical_in_base =
      tf2::quatRotate(quad_base_to_mounting.inverse(), vector_mounting_to_optical_in_mounting);

  const tf2::Vector3 vector_base_to_optical_in_base =
      vector_base_to_mounting_in_base + vector_mounting_to_optical_in_base;

  const tf2::Quaternion quad_base_to_optical = quad_base_to_mounting * quad_mounting_to_optical;

  tf.transform.translation.x = vector_base_to_optical_in_base.x();
  tf.transform.translation.y = vector_base_to_optical_in_base.y();
  tf.transform.translation.z = vector_base_to_optical_in_base.z();
  tf.transform.rotation.x = quad_base_to_optical.x();
  tf.transform.rotation.y = quad_base_to_optical.y();
  tf.transform.rotation.z = quad_base_to_optical.z();
  tf.transform.rotation.w = quad_base_to_optical.w();

  return tf;
}

bool SensorTfPublisher::transform_identical(geometry_msgs::msg::TransformStamped tf1,
                                            geometry_msgs::msg::TransformStamped tf2) const
{
  return (tf1.child_frame_id == tf2.child_frame_id) && (tf1.header.frame_id == tf2.header.frame_id) &&
         (tf1.transform == tf2.transform);
}

std::string SensorTfPublisher::tf_to_string(geometry_msgs::msg::TransformStamped tf) const
{
  std::stringstream ss("");

  ss << "frame_id=" << tf.header.frame_id << "\n";
  ss << "child_id=" << tf.child_frame_id << "\n";
  ss << "trans_x= " << tf.transform.translation.x << "\n";
  ss << "trans_y= " << tf.transform.translation.y << "\n";
  ss << "trans_z= " << tf.transform.translation.z << "\n";
  ss << "rot_x=   " << tf.transform.rotation.x << "\n";
  ss << "rot_y=   " << tf.transform.rotation.y << "\n";
  ss << "rot_z=   " << tf.transform.rotation.z << "\n";
  ss << "rot_w=   " << tf.transform.rotation.w << "\n";

  tf2::Quaternion quad(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                       tf.transform.rotation.w);

  ss << "angle=   " << quad.getAngle() << "\n";

  return ss.str();
}

void SensorTfPublisher::declare_parameters(std::string sensor_name)
{
  const std::string node_name(this->node_ptr_->get_name());

  tf_base_frame_name_descriptor_.name = "tf.base_frame_name";
  tf_base_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_base_frame_name_descriptor_.description = "Name for the ifm base frame, defaults to ifm_base_link.";
  if (!this->node_ptr_->has_parameter(tf_base_frame_name_descriptor_.name))
  {
    this->node_ptr_->declare_parameter(tf_base_frame_name_descriptor_.name, "ifm_base_link",
                                       tf_base_frame_name_descriptor_);
  }

  tf_mounting_frame_name_descriptor_.name = "tf.mounting_frame_name";
  tf_mounting_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_mounting_frame_name_descriptor_.description =
      "Name for the mounting point frame, defaults to <node_name>_mounting_link.";
  if (!this->node_ptr_->has_parameter(tf_mounting_frame_name_descriptor_.name))
  {
    this->node_ptr_->declare_parameter(tf_mounting_frame_name_descriptor_.name, node_name + "_mounting_link",
                                       tf_mounting_frame_name_descriptor_);
  }

  tf_sensor_frame_name_descriptor_.name = "tf." + sensor_name + "_frame_name";
  tf_sensor_frame_name_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  tf_sensor_frame_name_descriptor_.description =
      "Name for the point " + sensor_name + " frame, defaults to <node_name>_" + sensor_name + "_link.";
  if (!this->node_ptr_->has_parameter(tf_sensor_frame_name_descriptor_.name))
  {
    this->node_ptr_->declare_parameter(tf_sensor_frame_name_descriptor_.name, node_name + "_" + sensor_name + "_link",
                                       tf_sensor_frame_name_descriptor_);
  }

  tf_publish_base_to_mounting_descriptor_.name = "tf.publish_base_to_mounting";
  tf_publish_base_to_mounting_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  tf_publish_base_to_mounting_descriptor_.description =
      "Whether the transform from the ifm base link to the mounting point should be published.";
  if (!this->node_ptr_->has_parameter(tf_publish_base_to_mounting_descriptor_.name))
  {
    this->node_ptr_->declare_parameter(tf_publish_base_to_mounting_descriptor_.name, true,
                                       tf_publish_base_to_mounting_descriptor_);
  }

  tf_publish_mounting_to_sensor_descriptor_.name = "tf.publish_mounting_to_" + sensor_name;
  tf_publish_mounting_to_sensor_descriptor_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  tf_publish_mounting_to_sensor_descriptor_.description =
      "Whether the transform from the mounting point to the " + sensor_name + " frame should be published.";
  if (!this->node_ptr_->has_parameter(tf_publish_mounting_to_sensor_descriptor_.name))
  {
    this->node_ptr_->declare_parameter(tf_publish_mounting_to_sensor_descriptor_.name, true,
                                       tf_publish_mounting_to_sensor_descriptor_);
  }
}

void SensorTfPublisher::parse_parameters()
{
  node_ptr_->get_parameter(tf_base_frame_name_descriptor_.name, tf_base_link_frame_name_);
  RCLCPP_INFO(node_ptr_->get_logger(), "%s: %s", tf_base_frame_name_descriptor_.name.c_str(),
              tf_base_link_frame_name_.c_str());

  node_ptr_->get_parameter(tf_mounting_frame_name_descriptor_.name, tf_mounting_link_frame_name_);
  RCLCPP_INFO(node_ptr_->get_logger(), "%s: %s", tf_mounting_frame_name_descriptor_.name.c_str(),
              tf_mounting_link_frame_name_.c_str());

  node_ptr_->get_parameter(tf_sensor_frame_name_descriptor_.name, tf_sensor_link_frame_name_);
  RCLCPP_INFO(node_ptr_->get_logger(), "%s: %s", tf_sensor_frame_name_descriptor_.name.c_str(),
              tf_sensor_link_frame_name_.c_str());

  this->node_ptr_->get_parameter(tf_publish_base_to_mounting_descriptor_.name, tf_publish_base_to_mounting_);
  RCLCPP_INFO(node_ptr_->get_logger(), "%s: %s", tf_publish_base_to_mounting_descriptor_.name.c_str(),
              tf_publish_base_to_mounting_ ? "true" : "false");

  this->node_ptr_->get_parameter(tf_publish_mounting_to_sensor_descriptor_.name, tf_publish_mounting_to_sensor_);
  RCLCPP_INFO(node_ptr_->get_logger(), "%s: %s", tf_publish_mounting_to_sensor_descriptor_.name.c_str(),
              tf_publish_mounting_to_sensor_ ? "true" : "false");
}

tf2::Quaternion SensorTfPublisher::quaternion_from_ifm_rpy(const double roll, const double pitch,
                                                           const double yaw) const
{
  tf2::Quaternion q_roll, q_pitch, q_yaw, q_combined;

  q_roll.setRPY(roll, 0.0, 0.0);
  q_pitch.setRPY(0.0, pitch, 0.0);
  q_yaw.setRPY(0.0, 0.0, yaw);
  q_combined = q_roll * q_pitch * q_yaw;

  return q_combined;
}

}  // namespace ifm3d_ros2