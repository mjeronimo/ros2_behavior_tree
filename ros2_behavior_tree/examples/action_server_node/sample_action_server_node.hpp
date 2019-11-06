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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__ACTION_SERVER_NODE__SAMPLE_ACTION_SERVER_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__ACTION_SERVER_NODE__SAMPLE_ACTION_SERVER_NODE_HPP_

#include <memory>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"

namespace ros2_behavior_tree
{

class SampleActionServerNode : public rclcpp::Node
{
public:
  SampleActionServerNode();
  ~SampleActionServerNode();

protected:
#if 0
  using ActionServer = ros2_behavior_tree::action::ExecuteBehaviorTree;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionServer>;

  rclcpp_action::Server<ActionServer>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionServer::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
#endif
  // The node executes a Behavior Tree
  BehaviorTree bt_;

  // The XML string that defines the Behavior Tree to create and execute
  static const char bt_xml_[];

  // The routine to run on the separate thread
  BtStatus executeBehaviorTree();
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__ACTION_SERVER_NODE__SAMPLE_ACTION_SERVER_NODE_HPP_
