/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */
#include <ifm3d/device/o3r.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <string>

#include "ifm3d_ros2/services.hpp"

using json = ifm3d::json;

namespace ifm3d_ros2
{
BaseServices::BaseServices(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                           ifm3d::O3R::Ptr cam, ifm3d::PortInfo port_info, std::shared_ptr<std::mutex> ifm3d_mutex)
  : logger_(logger), node_ptr_(node_ptr), ifm3d_mutex_(ifm3d_mutex), cam_(cam), port_info_(port_info)
{
  RCLCPP_INFO(logger_, "BaseServices constructor called.");

  //
  // Set up our service servers
  //
  this->dump_srv_ = this->node_ptr_->create_service<DumpService>(
      "~/Dump", std::bind(&ifm3d_ros2::BaseServices::Dump, this, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3));

  this->config_srv_ = this->node_ptr_->create_service<ConfigService>(
      "~/Config", std::bind(&ifm3d_ros2::BaseServices::Config, this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3));

  this->soft_off_srv_ = this->node_ptr_->create_service<SoftoffService>(
      "~/Softoff", std::bind(&ifm3d_ros2::BaseServices::Softoff, this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  this->soft_on_srv_ = this->node_ptr_->create_service<SoftonService>(
      "~/Softon", std::bind(&ifm3d_ros2::BaseServices::Softon, this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3));

  this->get_diag_srv_ = this->node_ptr_->create_service<GetDiagService>(
      "~/GetDiag", std::bind(&ifm3d_ros2::BaseServices::GetDiag, this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  RCLCPP_INFO(logger_, "Services created;");
}

void BaseServices::Dump(std::shared_ptr<rmw_request_id_t> request_header, DumpRequest req, DumpResponse resp)
{
  (void)request_header;
  (void)req;
  RCLCPP_INFO(this->logger_, "Handling dump request...");

  if (this->node_ptr_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    // XXX: may want to change this logic. For now, I do it so I know
    // the ifm3d data structures are not null pointers
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(*this->ifm3d_mutex_);
    resp->status = 0;

    try
    {
      json j = this->cam_->ToJSON();
      resp->config = j.dump();
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "%s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "%s", std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "Dump: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "Dump request done.");
}

void BaseServices::Config(const std::shared_ptr<rmw_request_id_t> /*unused*/, ConfigRequest req, ConfigResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling config request...");

  if (this->node_ptr_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    // XXX: may want to change this logic. For now, I do it so I know
    // the ifm3d data structures are not null pointers
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(*this->ifm3d_mutex_);
    resp->status = 0;
    resp->msg = "OK";

    try
    {
      this->cam_->FromJSON(json::parse(req->json));  // HERE
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      resp->msg = ex.what();
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      resp->msg = std_ex.what();
    }
    catch (...)
    {
      resp->status = -2;
      resp->msg = "Unknown error in `Config'";
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "Config: %d - %s", resp->status, resp->msg.c_str());
    }
  }

  RCLCPP_INFO(this->logger_, "Config request done.");
}

void BaseServices::Softoff(const std::shared_ptr<rmw_request_id_t>, SoftoffRequest, SoftoffResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling SoftOff request...");

  if (this->node_ptr_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(*this->ifm3d_mutex_);
    resp->status = 0;
    auto port_type = this->port_info_.type;
    try
    {
      if ((port_type == "2D") | (port_type == "3D"))
      {
        this->cam_->FromJSONStr(R"({"ports":{")" + this->port_info_.port + R"(":{"state":"CONF"}}})");
        RCLCPP_INFO(this->logger_, "SoftOff request successful.");
      }
      else if (port_type == "app")
      {
        this->cam_->FromJSONStr(R"({"applications":{"instances":{")" + this->port_info_.port +
                                R"(":{"state":"CONF"}}}})");
        RCLCPP_INFO(this->logger_, "SoftOff request successful.");
      }
      else
      {
        RCLCPP_WARN(this->logger_, "Unknown port type: %s", port_type.c_str());
      }
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "Caught ifm3d exception: %s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "Caught standard exception: %s", std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
      RCLCPP_WARN(this->logger_, "Caught unknown exception");
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "SoftOff: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "SoftOff request done.");
}

void BaseServices::Softon(const std::shared_ptr<rmw_request_id_t> /*unused*/, SoftonRequest /*unused*/,
                          SoftonResponse resp)
{
  RCLCPP_INFO(this->logger_, "Handling SoftOn request...");

  if (this->node_ptr_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(*this->ifm3d_mutex_);
    resp->status = 0;
    auto port_type = this->port_info_.type;
    try
    {
      if ((port_type == "2D") | (port_type == "3D"))
      {
        this->cam_->FromJSONStr(R"({"ports":{")" + this->port_info_.port + R"(":{"state":"RUN"}}})");
        RCLCPP_INFO(this->logger_, "SoftOff request successful.");
      }
      else if (port_type == "app")
      {
        this->cam_->FromJSONStr(R"({"applications":{"instances":{")" + this->port_info_.port +
                                R"(":{"state":"RUN"}}}})");
        RCLCPP_INFO(this->logger_, "SoftOff request successful.");
      }
      else
      {
        RCLCPP_WARN(this->logger_, "Unknown port type: %s", port_type.c_str());
      }
    }
    catch (const ifm3d::Error& ex)
    {
      resp->status = ex.code();
      RCLCPP_WARN(this->logger_, "Caught ifm3d exception: %s", ex.what());
    }
    catch (const std::exception& std_ex)
    {
      resp->status = -1;
      RCLCPP_WARN(this->logger_, "Caught standard exception: %s", std_ex.what());
    }
    catch (...)
    {
      resp->status = -2;
      RCLCPP_WARN(this->logger_, "Caught unknown exception");
    }

    if (resp->status != 0)
    {
      RCLCPP_WARN(this->logger_, "SoftOff: %d", resp->status);
    }
  }

  RCLCPP_INFO(this->logger_, "SoftOff request done.");
}

void BaseServices::GetDiag(std::shared_ptr<rmw_request_id_t> request_header, GetDiagRequest req, GetDiagResponse resp)
{
  (void)request_header;
  RCLCPP_INFO(this->logger_, "Handling GetDiag request...");
  // TODO: shouldn't this be handled by ROS natively?
  if (this->node_ptr_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    resp->status = -1;
    RCLCPP_WARN(this->logger_, "Can only make a service request when node is ACTIVE");
    return;
  }
  try
  {
    json filter = json::parse(req->filter);
    RCLCPP_INFO(this->logger_, "Filter: %s", filter.dump().c_str());
    json diagnostics = this->cam_->GetDiagnosticFiltered(filter);
    RCLCPP_INFO(this->logger_, "Filtered diagnostics: %s", diagnostics.dump().c_str());
    resp->msg = diagnostics.dump();
    resp->status = 0;
  }
  catch (const ifm3d::Error& ex)
  {
    resp->status = ex.code();
    RCLCPP_INFO(this->logger_, "ifm3d error while trying to get the diagnostic: %s", ex.what());
  }
  catch (...)
  {
    resp->status = -2;
    RCLCPP_INFO(this->logger_, "Unknown error while trying to get the diagnostic");
  }
  RCLCPP_INFO(this->logger_, "GetDiagFiltered request done.");
}

}  // namespace ifm3d_ros2
