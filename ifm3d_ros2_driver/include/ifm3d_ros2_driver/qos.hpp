// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_QOS_HPP_
#define IFM3D_ROS2_QOS_HPP_

#include <rmw/rmw.h>
#include <rclcpp/qos.hpp>
#include <ifm3d_ros2_driver/visibility_control.h>

namespace ifm3d_ros2
{
/**
 * A quality of service that implements (as closely as possible) ROS1
 * latching-like behavior. Due to how publishers and subscribers need to
 * having matching/compatible QoS profiles, late subscribers must match this
 * QoS profile, exactly. There are some viable incompatibilities between the
 * publisher and subscriber, however, you should be aware of how that affects
 * the actual connection. For more information, please see:
 *
 * https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
 */
class IFM3D_ROS2_PUBLIC LatchedQoS : public rclcpp::QoS
{
public:
  static constexpr rmw_qos_profile_t qos_latched = { RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                     1,
                                                     RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                     RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                                     RMW_QOS_DEADLINE_DEFAULT,
                                                     RMW_QOS_LIFESPAN_DEFAULT,
                                                     RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                     RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                     false };

  explicit LatchedQoS(
      const rclcpp::QoSInitialization& qos_initialization = (rclcpp::QoSInitialization::from_rmw(qos_latched)))
    : rclcpp::QoS(qos_initialization, qos_latched)
  {
  }
};

/**
 * Specializes the `SensorDataQoS` in an attempt to minimize latency.
 */
class IFM3D_ROS2_PUBLIC LowLatencyQoS : public rclcpp::SensorDataQoS
{
public:
  explicit LowLatencyQoS() : rclcpp::SensorDataQoS()
  {
    // XXX: may ultimately want to make this configurable via parameter
    this->keep_last(2);
  }
};

}  // namespace ifm3d_ros2

#endif  // IFM3D_ROS2_QOS_HPP_
