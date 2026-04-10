/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2026 ifm electronic, gmbh
 */

#include <fstream>
#include <cmath>

#include <ifm3d/common/logging/logger.h>
#include <ifm3d_ros2/pds_vis_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <ifm3d_ros2/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std::chrono_literals;

namespace ifm3d_ros2
{

PdsVisNode::PdsVisNode(const rclcpp::NodeOptions& opts) : PdsVisNode::PdsVisNode("pds_vis_node", opts)
{
}

PdsVisNode::PdsVisNode(const std::string& node_name, const rclcpp::NodeOptions& opts)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", opts), logger_(this->get_logger()), currently_active_markers_{}
{
  // unbuffered I/O to stdout (so we can see our log messages)
  std::setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  RCLCPP_INFO(this->logger_, "namespace: %s", this->get_namespace());
  RCLCPP_INFO(this->logger_, "node name: %s", this->get_name());
  RCLCPP_INFO(this->logger_, "middleware: %s", rmw_get_implementation_identifier());

  pds_result_subcriber_ = create_subscription<ifm3d_ros2::msg::PdsFullResult>(
      "~/pds_full_result", BestEffortLowLatencyQoS(),
      std::bind(&PdsVisNode::pds_result_callback, this, std::placeholders::_1));

  marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/pds_vis_marker", ReliableLowLatencyQoS());

  RCLCPP_INFO(this->logger_, "node created, waiting for `configure()`...");
}

PdsVisNode::~PdsVisNode()
{
  RCLCPP_INFO(this->logger_, "Dtor called.");
}

TC_RETVAL PdsVisNode::on_configure(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_configure(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(this->logger_, "Configuration complete.");
  return TC_RETVAL::SUCCESS;
}

TC_RETVAL PdsVisNode::on_activate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_activate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  marker_publisher_->on_activate();

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL PdsVisNode::on_deactivate(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_deactivate(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  marker_publisher_->on_deactivate();

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL PdsVisNode::on_cleanup(const rclcpp_lifecycle::State& prev_state)
{
  // clean-up resources -- this will include our cam, fg,
  RCLCPP_INFO(this->logger_, "on_cleanup(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(this->logger_, "Node cleanup complete.");

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL PdsVisNode::on_shutdown(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_shutdown(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  return TC_RETVAL::SUCCESS;
}

TC_RETVAL PdsVisNode::on_error(const rclcpp_lifecycle::State& prev_state)
{
  RCLCPP_INFO(this->logger_, "on_error(): %s -> %s", prev_state.label().c_str(),
              this->get_current_state().label().c_str());

  RCLCPP_INFO(this->logger_, "Error processing complete.");

  return TC_RETVAL::SUCCESS;
}

visualization_msgs::msg::MarkerArray
PdsVisNode::pallet_visualization_markers(const ifm3d_ros2::msg::PalletDetectionArray& pallet_array) const
{
  visualization_msgs::msg::MarkerArray out_msg;

  for (auto pallet : pallet_array.detections)
  {
    const auto pallet_markers_msg = pallet_markers(pallet.left_position, pallet.center_position, pallet.right_position);
    out_msg.markers.insert(out_msg.markers.end(), pallet_markers_msg.markers.begin(), pallet_markers_msg.markers.end());
  }

  const auto depth_estimation_voi_markers = bounding_box_marker(pallet_array.depth_estimation_voi, BLUE);
  const auto projection_voi_markers = bounding_box_marker(pallet_array.projection_voi, GREEN);

  out_msg.markers.insert(out_msg.markers.end(), depth_estimation_voi_markers.markers.begin(),
                         depth_estimation_voi_markers.markers.end());
  out_msg.markers.insert(out_msg.markers.end(), projection_voi_markers.markers.begin(),
                         projection_voi_markers.markers.end());

  return out_msg;
}

visualization_msgs::msg::MarkerArray PdsVisNode::rack_visualization_markers(
    const ifm3d_ros2::msg::RackDetection& rack_detection, const std::string& last_command_input) const
{
  visualization_msgs::msg::MarkerArray out_msg;

  // Two bars are used to indicate Rack position
  visualization_msgs::msg::Marker horizontal_bar_origin, vertical_bar_origin;
  horizontal_bar_origin.header = rack_detection.header;
  horizontal_bar_origin.type = visualization_msgs::msg::Marker::CUBE;
  horizontal_bar_origin.action = visualization_msgs::msg::Marker::ADD;
  horizontal_bar_origin.scale.x = 0.03;
  horizontal_bar_origin.scale.y = 1.00;
  horizontal_bar_origin.scale.z = 0.03;
  horizontal_bar_origin.pose = rack_detection.position.pose;
  horizontal_bar_origin.color = GREEN;

  vertical_bar_origin = horizontal_bar_origin;
  vertical_bar_origin.scale.x = 0.03;
  vertical_bar_origin.scale.y = 0.03;
  vertical_bar_origin.scale.z = 1.00;
  vertical_bar_origin.color = BLUE;

  // The bars are ofset in Z and Y direction. Y offset is dependent on side.
  const double horizontal_offset = 0.3;
  const double vertical_offset = (rack_detection.side == "left") ? -horizontal_offset : horizontal_offset;

  // We need to calculate the center of both bars, to take not only the translation, but also the rotation into account.
  // The origin of a CUBE marker is in it's middle.
  tf2::Quaternion rack_detection_origin_q;  // Rotation of rack detection (crosspoint of both bars)
  tf2::fromMsg(rack_detection.position.pose.orientation, rack_detection_origin_q);
  tf2::Vector3 rack_detection_origin_vec;  // Position of rack detection (crosspoint of both bars)
  tf2::fromMsg(rack_detection.position.pose.position, rack_detection_origin_vec);
  tf2::Vector3 vertical_offset_vec{ 0.0, 0.0, horizontal_offset };      // Static offset in Z direction
  tf2::Vector3 horizontal_bar_offset_vec{ 0.0, vertical_offset, 0.0 };  // Static offset in Y direction

  tf2::Vector3 horizontal_bar_origin_vec =
      rack_detection_origin_vec + tf2::quatRotate(rack_detection_origin_q, horizontal_bar_offset_vec);

  tf2::Vector3 vertical_bar_origin_vec =
      rack_detection_origin_vec + tf2::quatRotate(rack_detection_origin_q, vertical_offset_vec);

  // Set calculated poses for markers, orientation is already set
  horizontal_bar_origin.pose.position.x = horizontal_bar_origin_vec.getX();
  horizontal_bar_origin.pose.position.y = horizontal_bar_origin_vec.getY();
  horizontal_bar_origin.pose.position.z = horizontal_bar_origin_vec.getZ();
  vertical_bar_origin.pose.position.x = vertical_bar_origin_vec.getX();
  vertical_bar_origin.pose.position.y = vertical_bar_origin_vec.getY();
  vertical_bar_origin.pose.position.z = vertical_bar_origin_vec.getZ();

  out_msg.markers.push_back(horizontal_bar_origin);
  out_msg.markers.push_back(vertical_bar_origin);

  // Visualize the horizontal beam and upright voi markers, as provided in the detection
  auto horizontal_beam_voi_markers = bounding_box_marker(rack_detection.horizontal_beam_voi, GREEN);
  auto upright_voi_markers = bounding_box_marker(rack_detection.upright_voi, BLUE);

  out_msg.markers.insert(out_msg.markers.end(), horizontal_beam_voi_markers.markers.begin(),
                         horizontal_beam_voi_markers.markers.end());
  out_msg.markers.insert(out_msg.markers.end(), upright_voi_markers.markers.begin(), upright_voi_markers.markers.end());

  // Parse the clearing volume from the last command JSON
  try
  {
    auto last_command_json = ifm3d::json::parse(last_command_input);
    double x_min, x_max, y_min, y_max, z_min, z_max;

    x_min = last_command_json["clearingVolume"]["xMin"].get<double>();
    x_max = last_command_json["clearingVolume"]["xMax"].get<double>();
    y_min = last_command_json["clearingVolume"]["yMin"].get<double>();
    y_max = last_command_json["clearingVolume"]["yMax"].get<double>();
    z_min = last_command_json["clearingVolume"]["zMin"].get<double>();
    z_max = last_command_json["clearingVolume"]["zMax"].get<double>();

    visualization_msgs::msg::Marker clearing_vol_marker;

    clearing_vol_marker.header = rack_detection.header;
    clearing_vol_marker.type = visualization_msgs::msg::Marker::CUBE;
    clearing_vol_marker.action = visualization_msgs::msg::Marker::ADD;
    clearing_vol_marker.scale.x = x_max - x_min;
    clearing_vol_marker.scale.y = y_max - y_min;
    clearing_vol_marker.scale.z = z_max - z_min;

    // The clearing volume is specified in the rack coordinate system.
    // Mirror it across local Y based on detected side so it follows the same side behavior
    // as rack bar visualization (left detection -> right side volume, right detection -> left side volume).
    const double side_y_sign = (rack_detection.side == "left") ? -1.0 : 1.0;
    const tf2::Vector3 clearing_center_local{ (x_max + x_min) / 2.0, side_y_sign * (y_max + y_min) / 2.0,
                          (z_max + z_min) / 2.0 };
    const tf2::Vector3 clearing_center_world =
      rack_detection_origin_vec + tf2::quatRotate(rack_detection_origin_q, clearing_center_local);

    clearing_vol_marker.pose.position.x = clearing_center_world.getX();
    clearing_vol_marker.pose.position.y = clearing_center_world.getY();
    clearing_vol_marker.pose.position.z = clearing_center_world.getZ();
    clearing_vol_marker.pose.orientation = rack_detection.position.pose.orientation;
    clearing_vol_marker.color = YELLOW;
    clearing_vol_marker.color.a = 0.3;

    out_msg.markers.push_back(clearing_vol_marker);
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_ERROR(logger_, "Could not parse clearing volume bounding box.");
  }

  return out_msg;
}

visualization_msgs::msg::MarkerArray PdsVisNode::volume_visualization_markers(
    const ifm3d_ros2::msg::VolumeCheck& volume_check, const std::string& last_command_input) const
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Visualizing volume check with last_command=" << last_command_input);
  ifm3d_ros2::msg::BoundingBoxStamped bb_msg;
  ifm3d::json last_command_json;

  try
  {
    last_command_json = ifm3d::json::parse(last_command_input);
    bb_msg.header = volume_check.header;
    bb_msg.box.x_min = last_command_json["xMin"].get<double>();
    bb_msg.box.x_max = last_command_json["xMax"].get<double>();
    bb_msg.box.y_min = last_command_json["yMin"].get<double>();
    bb_msg.box.y_max = last_command_json["yMax"].get<double>();
    bb_msg.box.z_min = last_command_json["zMin"].get<double>();
    bb_msg.box.z_max = last_command_json["zMax"].get<double>();
  }
  catch (ifm3d::json::parse_error& ex)
  {
    RCLCPP_ERROR(logger_, "Could not parse volume check bounding box.");
    return visualization_msgs::msg::MarkerArray();  // return empty marker array
  }

  auto out_msg = bounding_box_marker(bb_msg, YELLOW);

  // If we have a bounding box and there where pixels detected, mark nearest pixel by adding plane
  if (out_msg.markers.size() > 0 && volume_check.number_of_pixels > 0)
  {
    auto nearest_pixel_marker = out_msg.markers[0];
    nearest_pixel_marker.pose.position.x = volume_check.nearest_x;
    nearest_pixel_marker.scale.x = 0.001;
    nearest_pixel_marker.color.a = 0.8;
    out_msg.markers.push_back(nearest_pixel_marker);
  }

  return out_msg;
}

visualization_msgs::msg::MarkerArray PdsVisNode::bounding_box_marker(const ifm3d_ros2::msg::BoundingBoxStamped bb,
                                                                     std_msgs::msg::ColorRGBA color) const
{
  RCLCPP_DEBUG(get_logger(), "Creating marker representing a bounding box...");

  visualization_msgs::msg::MarkerArray out_msg;
  visualization_msgs::msg::Marker box_marker;

  box_marker.header = bb.header;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.action = visualization_msgs::msg::Marker::ADD;
  box_marker.scale.x = bb.box.x_max - bb.box.x_min;
  box_marker.scale.y = bb.box.y_max - bb.box.y_min;
  box_marker.scale.z = bb.box.z_max - bb.box.z_min;
  box_marker.pose.position.x = (bb.box.x_max + bb.box.x_min) / 2.0;
  box_marker.pose.position.y = (bb.box.y_max + bb.box.y_min) / 2.0;
  box_marker.pose.position.z = (bb.box.z_max + bb.box.z_min) / 2.0;
  box_marker.color = color;
  box_marker.color.a = 0.3;

  out_msg.markers.push_back(box_marker);

  RCLCPP_DEBUG(get_logger(), "Created marker representing a bounding box.");

  return out_msg;
}

visualization_msgs::msg::MarkerArray PdsVisNode::pallet_markers(const geometry_msgs::msg::PoseStamped& left_pose,
                                                                const geometry_msgs::msg::PoseStamped& center_pose,
                                                                const geometry_msgs::msg::PoseStamped& right_pose) const
{
  visualization_msgs::msg::MarkerArray out_msg;
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  // Left Arrow
  marker.header = left_pose.header;
  marker.pose = left_pose.pose;
  marker.color = GREEN;
  out_msg.markers.push_back(marker);

  // Center Arrow
  marker.header = center_pose.header;
  marker.pose = center_pose.pose;
  marker.color = YELLOW;
  out_msg.markers.push_back(marker);

  // Right Arrow
  marker.header = right_pose.header;
  marker.pose = right_pose.pose;
  marker.color = RED;
  out_msg.markers.push_back(marker);

  return out_msg;
}

void PdsVisNode::pds_result_callback(ifm3d_ros2::msg::PdsFullResult::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Received message.");

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    // Do nothing if not in final state
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  if (msg->last_command == "getPallet")
  {
    RCLCPP_INFO(get_logger(), "Parsing pallet detections.");
    markers = pallet_visualization_markers(msg->pallet_detection_array);
  }

  if (msg->last_command == "getRack")
  {
    RCLCPP_INFO(get_logger(), "Parsing rack detections.");
    markers = rack_visualization_markers(msg->rack_detection, msg->last_command_input);
  }

  if (msg->last_command == "volCheck")
  {
    RCLCPP_INFO(get_logger(), "Parsing volume check.");
    markers = volume_visualization_markers(msg->volume_check, msg->last_command_input);
  }

  // Set marker id and action for all markers; add DELETE actions if there are less markers now than in the last message
  const size_t number_of_markers_to_publish = markers.markers.size();
  for (size_t marker_id = 0; marker_id < std::max(currently_active_markers_, number_of_markers_to_publish); marker_id++)
  {
    if (marker_id < number_of_markers_to_publish)
    {
      // Add a new marker or update an existing one
      markers.markers[marker_id].action = visualization_msgs::msg::Marker::ADD;
      markers.markers[marker_id].id = marker_id;
    }
    else
    {
      visualization_msgs::msg::Marker deletion_marker;
      deletion_marker.action = visualization_msgs::msg::Marker::DELETE;
      deletion_marker.id = marker_id;
      markers.markers.push_back(deletion_marker);
    }
  }

  // Store number of added/modified markers for next execution
  currently_active_markers_ = number_of_markers_to_publish;

  // Set marker namespace and lifetime for all markers
  const size_t total_marker_count = markers.markers.size();
  for (size_t marker_id = 0; marker_id < total_marker_count; marker_id++)
  {
    markers.markers[marker_id].ns = "pds";
    markers.markers[marker_id].lifetime = rclcpp::Duration::from_nanoseconds(0);
  }

  marker_publisher_->publish(markers);
}

}  // namespace ifm3d_ros2

#include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(ifm3d_ros2::PdsVisNode)
