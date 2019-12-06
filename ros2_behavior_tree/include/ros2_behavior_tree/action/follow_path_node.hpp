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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_NODE_HPP_

#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace ros2_behavior_tree
{

using FollowPath = nav2_msgs::action::FollowPath;

class FollowPathNode : public ros2_behavior_tree::ROS2ActionClientNode<FollowPath>
{
public:
  explicit FollowPathNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<FollowPath>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
      });
  }

  void read_input_ports(FollowPath::Goal & goal) override
  {
    if (!getInput<nav_msgs::msg::Path>("path", goal.path)) {
      throw BT::RuntimeError("Missing parameter [path] in FollowPathNode node");
    }

    if (!getInput<std::string>("controller_id", goal.controller_id)) {
      throw BT::RuntimeError("Missing parameter [controller_id] in FollowPathNode node");
    }
  }

  bool read_new_goal(FollowPath::Goal & goal) override
  {
    nav_msgs::msg::Path path;
    if (!getInput<nav_msgs::msg::Path>("path", path)) {
      throw BT::RuntimeError("Missing parameter [path] in FollowPathNode node");
    }

    // If it's not the same as the goal we're currently working on, update the goal
    // and return true
    return (path != goal_.path) ? goal_.path = path, true : false;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_NODE_HPP_
