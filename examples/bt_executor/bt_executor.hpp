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

#ifndef ROS2_BEHAVIOR_TREE__BT_NAVIGATOR_HPP_
#define ROS2_BEHAVIOR_TREE__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_behavior_treee/behavior_tree_engine.hpp"

namespace ros2_behavior_treee
{

class BtExecutor : public rclcpp_lifecycle::LifecycleNode
{
public:
  BtExecutor();
  virtual ~BtExecutor();

protected:
  // The lifecycle node interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPose>;

  // Our action server implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void navigateToPose();

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The goal (on the blackboard) to be passed to ComputePath
  std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_;

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<NavigateToPoseBehaviorTree> bt_;

  // The complete behavior tree that results from parsing the incoming XML
  std::unique_ptr<BT::Tree> tree_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;
};

}  // namespace ros2_behavior_treee

#endif  // ROS2_BEHAVIOR_TREE__BT_NAVIGATOR_HPP_
