#include <ifm3d_ros2/camera_tf_publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace ifm3d_ros2
{

CameraTfPublisher::CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr,
                                     std::string port, std::string base_frame_name, std::string mounting_frame_name,
                                     std::string optical_frame_name)
  : SensorTfPublisher(node_ptr, base_frame_name, mounting_frame_name, optical_frame_name)
  , o3r_ptr_(o3r_ptr)
  , port_(port)
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->node_ptr_);
}

CameraTfPublisher::CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr,
                                     std::string port)
  : CameraTfPublisher(node_ptr, o3r_ptr, port, "", "", "")
{
}

bool CameraTfPublisher::update_and_publish_tf_if_changed(
    const geometry_msgs::msg::TransformStamped& new_tf_base_to_optical)
{
  bool published_tf = false;

  if (transform_identical(new_tf_base_to_optical, tf_base_to_sensor_))
  {
    // No change in the TFs
    return published_tf;
  }

  // Update the TF2 on first run (tf_base_to_sensor_ is not initialized) and on changes
  tf_base_to_sensor_ = new_tf_base_to_optical;

  const geometry_msgs::msg::TransformStamped new_tf_base_to_mounting =
      read_tf_base_to_mounting_from_config(new_tf_base_to_optical.header.stamp);

  const geometry_msgs::msg::TransformStamped new_tf_mounting_to_optical =
      calculate_tf_mounting_to_sensor(new_tf_base_to_optical.header.stamp, new_tf_base_to_optical, new_tf_base_to_mounting);

  if (!transform_identical(new_tf_base_to_mounting, tf_base_to_mounting_))
  {
    tf_base_to_mounting_ = new_tf_base_to_mounting;

    if (tf_publish_base_to_mounting_)
    {
      tf_static_broadcaster_->sendTransform(tf_base_to_mounting_);
      published_tf = true;
      RCLCPP_DEBUG(node_ptr_->get_logger(), "New Transform published:\n%s",
                   tf_to_string(new_tf_base_to_mounting).c_str());
    }
  }

  if (!transform_identical(new_tf_mounting_to_optical, tf_mounting_to_sensor_))
  {
    tf_mounting_to_sensor_ = new_tf_mounting_to_optical;

    if (tf_publish_mounting_to_sensor_)
    {
      tf_static_broadcaster_->sendTransform(tf_mounting_to_sensor_);
      published_tf = true;
      RCLCPP_DEBUG(node_ptr_->get_logger(), "New Transform published:\n%s",
                   tf_to_string(tf_mounting_to_sensor_).c_str());
    }
  }

  return published_tf;
}

geometry_msgs::msg::TransformStamped
CameraTfPublisher::read_tf_base_to_mounting_from_config(builtin_interfaces::msg::Time stamp)
{
  // TODO use try-catch in case of json errors

  std::string json_string = "/ports/" + port_ + "/processing/extrinsicHeadToUser";
  ifm3d::json::json_pointer j_pointer(json_string);
  auto config = o3r_ptr_->Get({ json_string })[j_pointer];

  const tf2::Quaternion q_combined = quaternion_from_ifm_rpy(config["rotX"], config["rotY"], config["rotZ"]);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_base_link_frame_name_;
  tf.child_frame_id = tf_mounting_link_frame_name_;

  tf.transform.translation.x = config["transX"];
  tf.transform.translation.y = config["transY"];
  tf.transform.translation.z = config["transZ"];
  tf.transform.rotation.x = q_combined.x();
  tf.transform.rotation.y = q_combined.y();
  tf.transform.rotation.z = q_combined.z();
  tf.transform.rotation.w = q_combined.w();

  return tf;
}


}