// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <ifm3d_ros2/function_module.hpp>

namespace ifm3d_ros2
{
FunctionModule::FunctionModule(const rclcpp::Logger& logger) : logger_(logger)
{
  RCLCPP_INFO(logger_, "FunctionModule contructor called.");
}

void FunctionModule::handle_frame(ifm3d::Frame::Ptr frame)
{
  (void)frame;
  RCLCPP_INFO(logger_, "FunctionModule: handle_frame called, implementation missing in derived class.");
}

}  // namespace ifm3d_ros2