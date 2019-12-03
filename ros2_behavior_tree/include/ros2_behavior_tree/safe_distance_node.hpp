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

#ifndef ROS2_BEHAVIOR_TREE__SAFE_DISTANCE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__SAFE_DISTANCE_NODE_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ros2_behavior_tree
{

class SafeDistanceNode : public BT::ConditionNode
{
public:
  SafeDistanceNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", "The distance threshold"),
      BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose_1", "The first pose"),
      BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose_2", "The second pose")
    };
  }

  BT::NodeStatus tick() override
  {
    double threshold;
    if (!getInput<double>("distance", threshold)) {
      throw BT::RuntimeError("Missing parameter [distance] in SafeDistance node");
    }

    std::shared_ptr<geometry_msgs::msg::PoseStamped> pose1;
    if (!getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose_1", pose1)) {
      throw BT::RuntimeError("Missing parameter [pose_1] in SafeDistance node");
    }

    std::shared_ptr<geometry_msgs::msg::PoseStamped> pose2;
    if (!getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose_2", pose2)) {
      throw BT::RuntimeError("Missing parameter [pose_2] in SafeDistance node");
    }

    double distance = sqrt(
	  pow(pose1->pose.position.x - pose2->pose.position.x, 2) - 
	  pow(pose1->pose.position.y - pose2->pose.position.y, 2)
	);

    printf("threshold: : %lf\n", threshold);
    printf("distance: %lf\n\n", distance);

    return (distance > threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__SAFE_DISTANCE_NODE_HPP_
