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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__GET_POSE_NEAR_ROBOT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__GET_POSE_NEAR_ROBOT_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ros2_behavior_tree
{

class GetPoseNearRobotNode : public BT::SyncActionNode
{
public:
  GetPoseNearRobotNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    nearby_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose",
        "The current pose of the robot"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("nearby_pose", "A pose near the robot")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<geometry_msgs::msg::PoseStamped> robot_pose;
    if (!getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose", robot_pose)) {
      throw BT::RuntimeError("Missing parameter [robot_pose] in GetPoseNearRobot node");
    }

    // Hard-code this as a placeholder until we have this operation
    nearby_pose_->pose.position.x = robot_pose->pose.position.x - 1.2;
    nearby_pose_->pose.position.y = robot_pose->pose.position.y;
    nearby_pose_->pose.position.z = robot_pose->pose.position.z;
    nearby_pose_->pose.orientation.x = robot_pose->pose.orientation.x;
    nearby_pose_->pose.orientation.y = robot_pose->pose.orientation.y;
    nearby_pose_->pose.orientation.z = robot_pose->pose.orientation.z;
    nearby_pose_->pose.orientation.w = robot_pose->pose.orientation.w;

    if (!setOutput<geometry_msgs::msg::PoseStamped>("nearby_pose", *nearby_pose_)) {
      throw BT::RuntimeError("Failed to set output port value [nearby_pose] for GetPoseNearRobot");
    }

    return BT::NodeStatus::SUCCESS;
  }

protected:
  std::shared_ptr<geometry_msgs::msg::PoseStamped> nearby_pose_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__GET_POSE_NEAR_ROBOT_NODE_HPP_
