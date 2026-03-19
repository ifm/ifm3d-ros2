#include <ifm3d_ros2/imu_tf_publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ifm3d_ros2
{

ImuTfPublisher::ImuTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr, std::string base_frame_name,
                               std::string mounting_frame_name, std::string optical_frame_name)
  : SensorTfPublisher(node_ptr, base_frame_name, mounting_frame_name, optical_frame_name)
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->node_ptr_);
}

ImuTfPublisher::ImuTfPublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr)
  : ImuTfPublisher(node_ptr, "", "", "")
{
}

bool ImuTfPublisher::update_and_publish_tf_if_changed(const geometry_msgs::msg::Transform& new_tf_base_to_sensor,
                                                      const geometry_msgs::msg::Transform& new_tf_mounting_to_sensor,
                                                      const rclcpp::Time& stamp)
{
  bool published_tf = false;

  if (tf_publish_mounting_to_sensor_)
  {
    if (new_tf_mounting_to_sensor != tf_mounting_to_sensor_.transform)
    {
      // TF from base to mounting has changed
      published_tf = true;

      tf_mounting_to_sensor_.transform = new_tf_mounting_to_sensor;
      tf_mounting_to_sensor_.header.stamp = stamp;
      tf_mounting_to_sensor_.header.frame_id = tf_mounting_link_frame_name_;
      tf_mounting_to_sensor_.child_frame_id = tf_sensor_link_frame_name_;

      tf_static_broadcaster_->sendTransform(tf_mounting_to_sensor_);

      RCLCPP_DEBUG(node_ptr_->get_logger(), "New Transform published:\n%s",
                   tf_to_string(tf_mounting_to_sensor_).c_str());
    }
  }

  if (tf_publish_base_to_mounting_)
  {
    // base_to_sensor - mounting_to_sensor

    tf2::Quaternion q_base_to_sensor, q_base_to_mounting, q_mounting_to_sensor;
    tf2::fromMsg(new_tf_base_to_sensor.rotation, q_base_to_sensor);
    tf2::fromMsg(new_tf_mounting_to_sensor.rotation, q_mounting_to_sensor);
    q_base_to_mounting = q_base_to_sensor * q_mounting_to_sensor.inverse();

    tf2::Vector3 t_base_to_sensor, t_base_to_mounting, t_mounting_to_sensor;
    tf2::fromMsg(new_tf_base_to_sensor.translation, t_base_to_sensor);
    tf2::fromMsg(new_tf_mounting_to_sensor.translation, t_mounting_to_sensor);
    t_base_to_mounting =
        t_base_to_sensor - t_mounting_to_sensor.rotate(q_base_to_sensor.getAxis(), q_base_to_sensor.getAngle());

    geometry_msgs::msg::Transform new_tf_base_to_mounting;
    new_tf_base_to_mounting.translation = tf2::toMsg(t_base_to_mounting);
    new_tf_base_to_mounting.rotation = tf2::toMsg(q_base_to_mounting);

    if (new_tf_base_to_mounting != tf_base_to_mounting_.transform)
    {
      // TF from base to mounting has changed
      published_tf = true;

      tf_base_to_mounting_.transform = new_tf_base_to_mounting;
      tf_base_to_mounting_.header.stamp = stamp;
      tf_base_to_mounting_.header.frame_id = tf_base_link_frame_name_;
      tf_base_to_mounting_.child_frame_id = tf_mounting_link_frame_name_;

      tf_static_broadcaster_->sendTransform(tf_base_to_mounting_);

      RCLCPP_DEBUG(node_ptr_->get_logger(), "New Transform published:\n%s", tf_to_string(tf_base_to_mounting_).c_str());
    }
  }

  // // Directly publish base to sensor as provided by the buffer for debugging the base -> mounting -> sensor chain
  // geometry_msgs::msg::TransformStamped tf_base_to_sensor;
  // tf_base_to_sensor.transform = new_tf_base_to_sensor;
  // tf_base_to_sensor.header.stamp = stamp;
  // tf_base_to_sensor.header.frame_id = tf_base_link_frame_name_;
  // tf_base_to_sensor.child_frame_id = tf_sensor_link_frame_name_ + "_debug";
  // tf_static_broadcaster_->sendTransform(tf_base_to_sensor);

  return published_tf;
}

}  // namespace ifm3d_ros2