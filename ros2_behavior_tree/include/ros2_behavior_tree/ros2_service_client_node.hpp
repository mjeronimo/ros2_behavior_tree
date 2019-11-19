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
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    if (!getInput<std::chrono::milliseconds>("server_timeout", server_timeout_)) {
      throw BT::RuntimeError("Missing parameter [server_timeout] in ROS2ServiceClientNode");
    }

    node_ = std::make_shared<rclcpp::Node>(name + "_service_client");
    service_client_ = node_->create_client<ServiceT>(service_name_);

    request_ = std::make_shared<typename ServiceT::Request>();
    response_ = std::make_shared<typename ServiceT::Response>();
  }

  ROS2ServiceClientNode() = delete;

  // Define this node's ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout", "BT loop timeout"),
      BT::InputPort<long>("a", "The augend"),
      BT::InputPort<long>("b", "The addend"),
      BT::OutputPort<long>("sum", "The sum of the addition")
    };
  }

  // The main override required by a BT service
  BT::NodeStatus tick() override
  {
    int a;
    if (!getInput("a", a)) {
      throw BT::RuntimeError("Missing parameter [a] in ROS2ServiceClientNode");
    }

    int b;
    if (!getInput("b", b)) {
      throw BT::RuntimeError("Missing parameter [b] in ROS2ServiceClientNode");
    }

    request_->a = a;
    request_->b = b;

    // Make sure the server is actually there before continuing
    // TODO(mjeronimo): make this a parameter
    const int service_wait_timeout = 100;
    // service_client_->wait_for_service();
    if (!service_client_->wait_for_service(std::chrono::milliseconds(service_wait_timeout))) {
      return BT::NodeStatus::FAILURE;
    }

    // Send the request to the server
    auto future_result = service_client_->async_send_request(request_);

    // Wait for the response
    auto rc = rclcpp::spin_until_future_complete(node_, future_result, server_timeout_);

    if (rc == rclcpp::executor::FutureReturnCode::SUCCESS) {
      response_ = future_result.get();
      setOutput("sum", response_->sum);
      return BT::NodeStatus::SUCCESS;
    }

    if (rc == rclcpp::executor::FutureReturnCode::TIMEOUT) {
      RCLCPP_WARN(node_->get_logger(),
        "Node timed out while executing service call to %s.", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
  }

protected:
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  std::string service_name_;

  // The timeout value while to use in the tick loop while waiting for
  // a response from the server
  std::chrono::milliseconds server_timeout_;

  std::shared_ptr<typename ServiceT::Request> request_;
  std::shared_ptr<typename ServiceT::Response> response_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ROS2_SERVICE_CLIENT_NODE_HPP_
