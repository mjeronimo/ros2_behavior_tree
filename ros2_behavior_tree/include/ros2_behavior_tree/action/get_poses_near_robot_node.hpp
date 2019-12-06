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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ros2_behavior_tree
{

class GetPosesNearRobotNode : public BT::SyncActionNode
{
public:
  GetPosesNearRobotNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose",
        "The current pose of the robot"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("nearby_poses",
        "Poses near the robot")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<geometry_msgs::msg::PoseStamped> robot_pose;
    if (!getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose", robot_pose)) {
      throw BT::RuntimeError("Missing parameter [robot_pose] in GetPosesNearRobot node");
    }

    std::vector<geometry_msgs::msg::PoseStamped> nearby_poses;

    // TODO(mjeronimo): Hard-coded values until we implement this operation

    geometry_msgs::msg::PoseStamped pose1;
    pose1.pose.position.x = robot_pose->pose.position.x - 1.2;
    pose1.pose.position.y = robot_pose->pose.position.y;
    pose1.pose.position.z = robot_pose->pose.position.z;
    pose1.pose.orientation.x = robot_pose->pose.orientation.x;
    pose1.pose.orientation.y = robot_pose->pose.orientation.y;
    pose1.pose.orientation.z = robot_pose->pose.orientation.z;
    pose1.pose.orientation.w = robot_pose->pose.orientation.w;

    nearby_poses.push_back(pose1);

    geometry_msgs::msg::PoseStamped pose2;
    pose2.pose.position.x = robot_pose->pose.position.x;
    pose2.pose.position.y = robot_pose->pose.position.y - 1.2;
    pose2.pose.position.z = robot_pose->pose.position.z;
    pose2.pose.orientation.x = robot_pose->pose.orientation.x;
    pose2.pose.orientation.y = robot_pose->pose.orientation.y;
    pose2.pose.orientation.z = robot_pose->pose.orientation.z;
    pose2.pose.orientation.w = robot_pose->pose.orientation.w;

    nearby_poses.push_back(pose2);

    if (!setOutput<std::vector<geometry_msgs::msg::PoseStamped>>("nearby_poses", nearby_poses)) {
      throw BT::RuntimeError(
              "Failed to set output port value [nearby_poses] for GetPosesNearRobot");
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_
