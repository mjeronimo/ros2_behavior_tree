// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__ROS2_SERVICE_CLIENT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ROS2_SERVICE_CLIENT_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/bt_conversions.hpp"

namespace ros2_behavior_tree
{

template<class ServiceT>
class ROS2ServiceClientNode : public BT::SyncActionNode
{
public:
  ROS2ServiceClientNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    request_ = std::make_shared<typename ServiceT::Request>();
    response_ = std::make_shared<typename ServiceT::Response>();
  }

  ROS2ServiceClientNode() = delete;

  // Define the ports required by the ROS2ServiceClient node
  static BT::PortsList augment_basic_ports(BT::PortsList additional_ports)
  {
    BT::PortsList basic_ports = {
      BT::InputPort<std::string>("service_name", "The name of the service to call"),
      BT::InputPort<std::chrono::milliseconds>("wait_timeout",
        "The timeout value, in milliseconds, to use when waiting for the service"),
      BT::InputPort<std::chrono::milliseconds>("call_timeout",
        "The timeout value, in milliseconds, to use when calling the service"),
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("client_node",
        "The (non-spinning) client node to use when making service calls")
    };

    basic_ports.insert(additional_ports.begin(), additional_ports.end());
    return basic_ports;
  }

  // Any subclass of ROS2ServiceClient that defines additional ports must then define its
  // own providedPorts method and call augment_basic_ports to add the subclass's ports to
  // the required basic ports
  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({});
  }

  // A derived class the defines input and/or output ports can override these methods
  // to get/set the ports
  virtual void get_input_ports() {}
  virtual void set_output_ports() {}

  // The main override required by a BT service
  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    if (!getInput<std::chrono::milliseconds>("wait_timeout", wait_timeout_)) {
      throw BT::RuntimeError("Missing parameter [wait_timeout] in ROS2ServiceClientNode");
    }

    if (!getInput<std::chrono::milliseconds>("call_timeout", call_timeout_)) {
      throw BT::RuntimeError("Missing parameter [call_timeout] in ROS2ServiceClientNode");
    }

    if (!getInput<std::shared_ptr<rclcpp::Node>>("client_node", client_node_)) {
      throw BT::RuntimeError("Missing parameter [client_node] in ROS2ServiceClientNode");
    }

    get_input_ports();

    if (service_client_ == nullptr) {
      service_client_ = client_node_->create_client<ServiceT>(service_name_);
    }

    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(wait_timeout_))) {
      RCLCPP_ERROR(client_node_->get_logger(),
        "Node timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Send the request to the server
    auto future_result = service_client_->async_send_request(request_);

    // Wait for the response
    auto rc = rclcpp::spin_until_future_complete(client_node_, future_result, call_timeout_);

    if (rc == rclcpp::executor::FutureReturnCode::SUCCESS) {
      response_ = future_result.get();
      set_output_ports();
      return BT::NodeStatus::SUCCESS;
    }

    if (rc == rclcpp::executor::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(client_node_->get_logger(), "Call to \"%s\" service timed out",
        service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(client_node_->get_logger(), "Call to \"%s\" server failed", service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

protected:
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  // The (non-spinning) node to use when calling the service
  rclcpp::Node::SharedPtr client_node_;

  std::string service_name_;

  std::chrono::milliseconds wait_timeout_;
  std::chrono::milliseconds call_timeout_;

  std::shared_ptr<typename ServiceT::Request> request_;
  std::shared_ptr<typename ServiceT::Response> response_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ROS2_SERVICE_CLIENT_NODE_HPP_
