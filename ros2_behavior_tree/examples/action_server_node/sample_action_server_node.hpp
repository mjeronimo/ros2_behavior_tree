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
#include "ros2_behavior_tree_msgs/action/print_message.hpp"

namespace ros2_behavior_tree
{

class SampleActionServerNode : public rclcpp::Node
{
public:
  SampleActionServerNode();
  ~SampleActionServerNode();

protected:
  // A couple declarations for convenince
  using ActionServer = ros2_behavior_tree_msgs::action::PrintMessage;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionServer>;

  // The action server presented by this node
  rclcpp_action::Server<ActionServer>::SharedPtr action_server_;

  // The three action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionServer::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle);

  // The routine to run on the separate thread
  void print_message(const std::shared_ptr<GoalHandle> goal_handle);

  // The XML string that defines the Behavior Tree used to implement the print_message action
  static const char bt_xml_[];
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__ACTION_SERVER_NODE__SAMPLE_ACTION_SERVER_NODE_HPP_
