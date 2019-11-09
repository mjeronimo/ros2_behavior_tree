// Copyright (c) 2018 Intel Corporation
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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__NAVIGATE_TO_POSE_ACTION_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__NAVIGATE_TO_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_conversions.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "ros2_behavior_tree/ros2_action_node.hpp"
#include "ros2_behavior_tree_msgs/action/navigate_to_pose.hpp"

namespace ros2_behavior_tree
{

class NavigateToPoseAction : public ROS2ActionNode<ros2_behavior_tree_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : ROS2ActionNode<ros2_behavior_tree_msgs::action::NavigateToPose>(action_name, conf)
  {
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::msg::Point>("position", "Position"),
        BT::InputPort<geometry_msgs::msg::Quaternion>("orientation", "Orientation")
      });
  }

  void on_tick() override
  {
    // Use the position and orientation fields from the XML attributes to initialize the goal
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;

    auto have_position = getInput("position", position);
    auto have_orientation = getInput("orientation", orientation);

    if (!have_position || !have_orientation) {
      RCLCPP_ERROR(node_->get_logger(),
        "NavigateToPoseAction: position or orientation not provided");
    }

    goal_.pose.pose.position = position;
    goal_.pose.pose.orientation = orientation;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__NAVIGATE_TO_POSE_ACTION_HPP_
