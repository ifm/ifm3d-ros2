// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_CAMERA_NODE_HPP_
#define IFM3D_ROS2_CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ifm3d_ros2/visibility_control.h>

#include <ifm3d_ros2/msg/extrinsics.hpp>
#include <ifm3d_ros2/srv/dump.hpp>
#include <ifm3d_ros2/srv/config.hpp>
#include <ifm3d_ros2/srv/softon.hpp>
#include <ifm3d_ros2/srv/softoff.hpp>

#include <ifm3d/device.h>
#include <ifm3d/fg.h>

using TC_RETVAL = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using ImageMsg = sensor_msgs::msg::Image;
using ImagePublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ImageMsg>>;

using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using CompressedImagePublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<CompressedImageMsg>>;

using PCLMsg = sensor_msgs::msg::PointCloud2;
using PCLPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<PCLMsg>>;

using ExtrinsicsMsg = ifm3d_ros2::msg::Extrinsics;
using ExtrinsicsPublisher = std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ExtrinsicsMsg>>;

using DumpRequest = std::shared_ptr<ifm3d_ros2::srv::Dump::Request>;
using DumpResponse = std::shared_ptr<ifm3d_ros2::srv::Dump::Response>;
using DumpService = ifm3d_ros2::srv::Dump;
using DumpServer = rclcpp::Service<ifm3d_ros2::srv::Dump>::SharedPtr;

using ConfigRequest = std::shared_ptr<ifm3d_ros2::srv::Config::Request>;
using ConfigResponse = std::shared_ptr<ifm3d_ros2::srv::Config::Response>;
using ConfigService = ifm3d_ros2::srv::Config;
using ConfigServer = rclcpp::Service<ifm3d_ros2::srv::Config>::SharedPtr;

using SoftoffRequest = std::shared_ptr<ifm3d_ros2::srv::Softoff::Request>;
using SoftoffResponse = std::shared_ptr<ifm3d_ros2::srv::Softoff::Response>;
using SoftoffService = ifm3d_ros2::srv::Softoff;
using SoftoffServer = rclcpp::Service<ifm3d_ros2::srv::Softoff>::SharedPtr;

using SoftonRequest = std::shared_ptr<ifm3d_ros2::srv::Softon::Request>;
using SoftonResponse = std::shared_ptr<ifm3d_ros2::srv::Softon::Response>;
using SoftonService = ifm3d_ros2::srv::Softon;
using SoftonServer = rclcpp::Service<ifm3d_ros2::srv::Softon>::SharedPtr;

namespace ifm3d_ros2
{
/**
 * Managed node that implements an ifm3d camera driver for ROS 2 software
 * systems.
 *
 * This camera node is implemented as a lifecycle node allowing for
 * management by an external process or tool. State transitions (edges in the
 * managed node FSM graph) are handled by the `on_XXX()` callback functions
 * implemented herein.
 */
class IFM3D_ROS2_PUBLIC CameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Instantiates the LifecycleNode. At the completion of the ctor, the
   * following initialization (beyond calling the parent ctor) has been done:
   *
   * - A named logger for this node has been initialized
   * - tf frame names have been initialzed based on the node name
   * - All parameters have been declared and a `set` callback has been
   *   registered
   * - All publishers have been created.
   */
  explicit CameraNode(const std::string& node_name, const rclcpp::NodeOptions& opts);

  /**
   * Delegates construction to the above ctor.
   */
  explicit CameraNode(const rclcpp::NodeOptions& opts);

  /**
   * RAII deallocations. As of this writing, given that all structures are
   * handled by various types of smart pointers no "real work" needs to be
   * done here. However, for debugging purposes we emit a log message so we
   * know when the dtor has actually been called and hence when all
   * deallocations actually occur.
   */
  ~CameraNode() override;

  /**
   * Implements the "configuring" transition state
   *
   * The following operations are performed:
   *
   * - Parameters are parsed and held locally in instance variables.
   * - If requested, the camera clock is synchronized to the system clock
   * - The core ifm3d data structures (camera, framegrabber, stlimage buffer)
   *   are initialized and ready to stream data based upon the requested
   *   schema mask.
   */
  TC_RETVAL on_configure(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "activating" transition state
   *
   * The following operations are performed:
   *
   * - The `on_activate()` method is called on all publishers
   * - A new thread is started that will continuous publish image data from
   *   the camera.
   */
  TC_RETVAL on_activate(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "deactivating" transition state
   *
   * The following operations are performed:
   *
   * - The thread that implements the "publish loop" is stopped
   * - All publishers can their `on_deactivate()` method called
   */
  TC_RETVAL on_deactivate(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "cleaningup" transition state
   *
   * The following operations are performed:
   *
   * - The ifm3d core data structures (camera, framegrabber, stlimage buffer)
   *   have their dtors called
   */
  TC_RETVAL on_cleanup(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "shuttingdown" transition state
   *
   * The following operations are performed:
   *
   * - It is ensured that the publishing loop thread is stopped
   */
  TC_RETVAL on_shutdown(const rclcpp_lifecycle::State& prev_state) override;

  /**
   * Implements the "errorprocessing" transition state
   *
   * The following operations are performed:
   *
   * - The publish_loop thread is stopped (if running)
   * - The ifm3d core data structures (camera, framegrabber, stlimage buffer)
   *   have their dtors called
   */
  TC_RETVAL on_error(const rclcpp_lifecycle::State& prev_state) override;

protected:
  /**
   * Implementation of the Dump service.
   */
  void Dump(std::shared_ptr<rmw_request_id_t> request_header, DumpRequest req, DumpResponse resp);

  /**
   * Implementation of the Config service.
   */
  void Config(std::shared_ptr<rmw_request_id_t> request_header, ConfigRequest req, ConfigResponse resp);

  /**
   * Implementation of the SoftOff service.
   */
  void Softoff(std::shared_ptr<rmw_request_id_t> request_header, SoftoffRequest req, SoftoffResponse resp);

  /**
   * Implementation of the SoftOn service.
   */
  void Softon(std::shared_ptr<rmw_request_id_t> request_header, SoftonRequest req, SoftonResponse resp);

  /**
   * Callback that gets called when a parameter(s) is attempted to be set
   *
   * Some parameters can be changed on the fly while others, if changed,
   * require the node to reconfigure itself (e.g., because it needs to
   * switch the operating mode of the camera or connect to a different camera
   * etc.). In general, we take the new parameter values and set them into
   * the instance variables of this node. However, if a reconfiguration is
   * required, after looking at all the parameters, a state change that would
   * ultimately have the camera reinitialize is affected.
   */
  rcl_interfaces::msg::SetParametersResult set_params_cb(const std::vector<rclcpp::Parameter>& params);

  /**
   * Declares parameters and default values
   */
  void init_params();

  /**
   * Callback which receives new Frames from ifm3d
   */
  void frame_callback(ifm3d::Frame::Ptr frame);

  /**
   * Callback which receives Errors from ifm3d
   */
  void error_callback(const ifm3d::Error& error);

  /**
   * Callback which receives AsyncErrors from ifm3d
   */
  void async_error_callback(int i, const std::string& s);

  /**
   * Callback which receives AsyncNotifications from ifm3d
   */
  void async_notification_callback(const std::string& s1, const std::string& s2);

  /**
   * @brief Create publishers according to buffer_id_list_.
   *
   * First, this clears internal publisher lists.
   * Populates the internal publisher lists with new Publishers.
   * Uses buffer_id_utils to determine message types.
   */
  void initialize_publishers();

  /**
   * Activates all publishers on the internal publisher lists.
   */
  void activate_publishers();

  /**
   * Deactivates all publishers on the internal publisher lists.
   */
  void deactivate_publishers();

private:
  rclcpp::Logger logger_;
  // global mutex on ifm3d core data structures `cam_`, `fg_`, `im_`
  std::mutex gil_{};

  std::string ip_{};
  std::uint16_t xmlrpc_port_{};
  std::string password_{};
  std::vector<ifm3d::buffer_id> buffer_id_list_{};
  int timeout_millis_{};
  float timeout_tolerance_secs_{};
  float frame_latency_thresh_{};  // seconds
  bool sync_clocks_{};
  std::uint16_t pcic_port_{};

  DumpServer dump_srv_{};
  ConfigServer config_srv_{};
  SoftoffServer soft_off_srv_{};
  SoftonServer soft_on_srv_{};

  ifm3d::Device::Ptr cam_{};
  ifm3d::FrameGrabber::Ptr fg_{};
  // ifm3d::StlImageBuffer::Ptr im_{};

  std::map<ifm3d::buffer_id, ImagePublisher> image_publishers_;
  std::map<ifm3d::buffer_id, CompressedImagePublisher> compressed_image_publishers_;
  std::map<ifm3d::buffer_id, PCLPublisher> pcl_publishers_;
  std::map<ifm3d::buffer_id, ExtrinsicsPublisher> extrinsics_publishers_;

  std::atomic_bool is_active_{};

  std::string camera_frame_{};
  std::string optical_frame_{};

  rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr set_params_cb_handle_{};
};  // end: class CameraNode

}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_CAMERA_NODE_HPP_
