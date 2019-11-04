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

#ifndef EXAMPLES__ACTION_SERVER_LIFECYCLE_NODE__BT_EXECUTOR_HPP_
#define EXAMPLES__ACTION_SERVER_LIFECYCLE_NODE__BT_EXECUTOR_HPP_

#include <memory>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_behavior_tree/behavior_tree_engine.hpp"
#include "ros2_behavior_tree/action/execute_behavior_tree.hpp"

namespace ros2_behavior_tree
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BtExecutor : public rclcpp_lifecycle::LifecycleNode
{
public:
  BtExecutor();
  virtual ~BtExecutor();

protected:
  // The lifecycle node interface
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = ros2_behavior_tree::action::ExecuteBehaviorTree;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<ActionServer>;

  rclcpp_action::Server<ActionServer>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionServer::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    // using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&BtExecutor::executeBehaviorTree, this, _1), goal_handle}.detach();
    std::thread{std::bind(&BtExecutor::executeBehaviorTree, this), goal_handle}.detach();
  }

  // The action server callback
  // TODO(mjeronimo: optional string. If receive string, set xml string (on parameter?) and execute.
  // If no string, get parameter and execute
  void executeBehaviorTree();

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<BehaviorTreeEngine> bt_;

  // The complete behavior tree that results from parsing the incoming XML
  std::unique_ptr<BT::Tree> tree_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;
};

}  // namespace ros2_behavior_tree

#endif  // EXAMPLES__ACTION_SERVER_LIFECYCLE_NODE__BT_EXECUTOR_HPP_
