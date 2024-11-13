#include <ifm3d_ros2/camera_tf_publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace ifm3d_ros2
{

CameraTfPublisher::CameraTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, ifm3d::O3R::Ptr o3r_ptr,
                                     std::string port, std::string base_frame_name, std::string mounting_frame_name,
                                     std::string optical_frame_name)
  : tf_base_link_frame_name_(base_frame_name)
  , tf_mounting_link_frame_name_(mounting_frame_name)
  , tf_optical_link_frame_name_(optical_frame_name)
  , tf_publish_mounting_to_optical_(true)
  , tf_publish_base_to_mounting_(true)
  , node_ptr_(node_ptr)
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

  if (transform_identical(new_tf_base_to_optical, tf_base_to_optical_))
  {
    // No change in the TFs
    return published_tf;
  }

  // Update the TF2 on first run (tf_base_to_optical_ is not initialized) and on changes
  tf_base_to_optical_ = new_tf_base_to_optical;

  const geometry_msgs::msg::TransformStamped new_tf_base_to_mounting =
      read_tf_base_to_mounting_from_device_config(new_tf_base_to_optical.header.stamp);

  const geometry_msgs::msg::TransformStamped new_tf_mounting_to_optical =
      get_tf_mounting_to_optical(new_tf_base_to_optical.header.stamp, new_tf_base_to_optical, new_tf_base_to_mounting);

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

  if (!transform_identical(new_tf_mounting_to_optical, tf_mounting_to_optical_))
  {
    tf_mounting_to_optical_ = new_tf_mounting_to_optical;

    if (tf_publish_mounting_to_optical_)
    {
      tf_static_broadcaster_->sendTransform(tf_mounting_to_optical_);
      published_tf = true;
      RCLCPP_DEBUG(node_ptr_->get_logger(), "New Transform published:\n%s",
                   tf_to_string(tf_mounting_to_optical_).c_str());
    }
  }

  return published_tf;
}

geometry_msgs::msg::TransformStamped
CameraTfPublisher::read_tf_base_to_mounting_from_device_config(builtin_interfaces::msg::Time stamp)
{
  // TODO use try-catch in case of json errors

  std::string json_string = "/ports/" + port_ + "/processing/extrinsicHeadToUser";
  ifm3d::json::json_pointer j_pointer(json_string);
  auto config = o3r_ptr_->Get({ json_string })[j_pointer];

  // Euler angles:
  // ifm3d provides roll, pitch, yaw angles for intrisic rotations
  // while tf2 expects them for extrinsic rotation.
  // Therefore, a tf2 quaternion needs to be created from 3 separate rotations
  tf2::Quaternion q_roll, q_pitch, q_yaw, q_combined;
  q_roll.setRPY(config["rotX"], 0.0, 0.0);
  q_pitch.setRPY(0.0, config["rotY"], 0.0);
  q_yaw.setRPY(0.0, 0.0, config["rotZ"]);
  q_combined = q_roll * q_pitch * q_yaw;

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

geometry_msgs::msg::TransformStamped CameraTfPublisher::get_tf_mounting_to_optical(
    builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_optical,
    geometry_msgs::msg::TransformStamped tf_base_to_mounting)
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

  // TODO check rotation here!
  tf2::Vector3 vector_mounting_to_optical_in_mounting =
      tf2::quatRotate(quad_base_to_mounting.inverse(), vector_mounting_to_optical_in_base);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_mounting_link_frame_name_;
  tf.child_frame_id = tf_optical_link_frame_name_;
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

geometry_msgs::msg::TransformStamped CameraTfPublisher ::calculate_tf_base_to_optical(
    builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped tf_base_to_mounting,
    geometry_msgs::msg::TransformStamped tf_mounting_to_optical)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = tf_base_link_frame_name_;
  tf.child_frame_id = tf_optical_link_frame_name_;

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

bool CameraTfPublisher::transform_identical(geometry_msgs::msg::TransformStamped tf1,
                                            geometry_msgs::msg::TransformStamped tf2)
{
  return (tf1.child_frame_id == tf2.child_frame_id) && (tf1.header.frame_id == tf2.header.frame_id) &&
         (tf1.transform == tf2.transform);
}

std::string CameraTfPublisher::tf_to_string(geometry_msgs::msg::TransformStamped tf)
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

}  // namespace ifm3d_ros2