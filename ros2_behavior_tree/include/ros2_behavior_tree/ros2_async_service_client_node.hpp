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

#ifndef ROS2_BEHAVIOR_TREE__ROS2_ASYNC_SERVICE_CLIENT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ROS2_ASYNC_SERVICE_CLIENT_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/bt_conversions.hpp"

namespace ros2_behavior_tree
{

template<class ServiceT>
class ROS2AsyncServiceClientNode : public BT::AsyncActionNode
{
public:
  ROS2AsyncServiceClientNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config)
  {
    request_ = std::make_shared<typename ServiceT::Request>();
    response_ = std::make_shared<typename ServiceT::Response>();
  }

  ROS2AsyncServiceClientNode() = delete;

  // Define the ports required by the ROS2AsyncServiceClient node
  static BT::PortsList augment_basic_ports(BT::PortsList additional_ports)
  {
    BT::PortsList basic_ports = {
      BT::InputPort<std::string>("service_name", "The name of the service to call"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout",
        "The timeout value, in milliseconds, to use when waiting for service responses"),
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("ros2_node",
        "The ROS2 node to use when when creating the service")
    };

    basic_ports.insert(additional_ports.begin(), additional_ports.end());
    return basic_ports;
  }

  // Any subclass of ROS2AsyncServiceClientNode that defines additional ports must then define
  // its own providedPorts method and call augment_basic_ports to add the subclass's ports
  // to the required basic ports
  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({});
  }

  // A derived class the defines input and/or output ports can override these methods
  // to get/set the ports
  virtual void read_input_ports(std::shared_ptr<typename ServiceT::Request> request) {}
  virtual void write_output_ports(std::shared_ptr<typename ServiceT::Response> response) {}

  // The main override required by a BT service
  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2AsyncServiceClientNode");
    }

    if (!getInput<std::chrono::milliseconds>("server_timeout", server_timeout_)) {
      throw BT::RuntimeError("Missing parameter [server_timeout] in ROS2AsyncServiceClientNode");
    }

    if (!getInput<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_)) {
      throw BT::RuntimeError("Missing parameter [ros2_node] in ROS2AsyncServiceClientNode");
    }

    read_input_ports(request_);

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<ServiceT>(service_name_);
    }

    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto future_result = service_client_->async_send_request(request_);
    for (;; ) {
      switch (future_result.wait_for(server_timeout_)) {
        case std::future_status::ready:
          response_ = future_result.get();
          write_output_ports(response_);
          return BT::NodeStatus::SUCCESS;

        case std::future_status::timeout:
          break;

        default:
          return BT::NodeStatus::FAILURE;
      }
    }
  }

protected:
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  // The (non-spinning) node to use when calling the service
  rclcpp::Node::SharedPtr ros2_node_;

  std::string service_name_;

  std::chrono::milliseconds server_timeout_;

  std::shared_ptr<typename ServiceT::Request> request_;
  std::shared_ptr<typename ServiceT::Response> response_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ROS2_ASYNC_SERVICE_CLIENT_NODE_HPP_
