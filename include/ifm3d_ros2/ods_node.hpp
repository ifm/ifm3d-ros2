// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_ODS_NODE_HPP_
#define IFM3D_ROS2_ODS_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>

#include <ifm3d_ros2/diag_module.hpp>
#include <ifm3d_ros2/ods_module.hpp>
#include <ifm3d_ros2/services.hpp>
#include <ifm3d/device/o3r.h>
#include <ifm3d_ros2/visibility_control.h>

#include <ifm3d/device.h>
#include <ifm3d/fg.h>

using TC_RETVAL = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ifm3d_ros2
{
/**
 * Managed node that implements the ODS application of ifm3d for ROS 2.
 */
class IFM3D_ROS2_PUBLIC OdsNode : public rclcpp_lifecycle::LifecycleNode
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
  explicit OdsNode(const std::string& node_name, const rclcpp::NodeOptions& opts);

  /**
   * Delegates construction to the above ctor.
   */
  explicit OdsNode(const rclcpp::NodeOptions& opts);

  /**
   * RAII deallocations. As of this writing, given that all structures are
   * handled by various types of smart pointers no "real work" needs to be
   * done here. However, for debugging purposes we emit a log message so we
   * know when the dtor has actually been called and hence when all
   * deallocations actually occur.
   */
  ~OdsNode() override;

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
   * Declares parameters and default values
   */
  void init_params();

  /**
   * Reads parameters, needs init_params() to be called beforehand
   */
  void parse_params();

  /**
   * Sets the callbacks handling parameter changes at runtime
   */
  void set_parameter_event_callbacks();

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

private:
  rclcpp::Logger logger_;

  ifm3d::O3R::Ptr o3r_{};  // TODO probably need some other device
  ifm3d::FrameGrabber::Ptr fg_{};
  ifm3d::FrameGrabber::Ptr fg_diag_{};

  std::shared_ptr<OdsModule> ods_module_;
  std::shared_ptr<DiagModule> diag_module_;
  std::vector<std::shared_ptr<FunctionModule>> modules_;

  /// global mutex on ifm3d core data structures `fg_`
  std::shared_ptr<std::mutex> gil_{};

  // Values read from parameters
  std::string config_file_{};
  std::string ip_{};
  std::uint16_t pcic_port_{};
  std::uint16_t xmlrpc_port_{};

  ifm3d::PortInfo port_info_{};

  // Values read from incoming image buffers
  uint32_t width_;
  uint32_t height_;

  /// Subscription to parameter changes
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  /// Callbacks need to be stored to work properly; using a map with parameter name as key
  std::map<std::string, rclcpp::ParameterCallbackHandle::SharedPtr> registered_param_callbacks_;

  // Service Servers
  std::shared_ptr<BaseServices> base_services_{};

};  // end: class OdsNode

}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_ODS_NODE_HPP_
