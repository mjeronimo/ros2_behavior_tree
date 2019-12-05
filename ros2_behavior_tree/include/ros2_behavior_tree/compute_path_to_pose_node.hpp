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

#ifndef ROS2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

namespace ros2_behavior_tree
{

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

class ComputePathToPoseClient : public ros2_behavior_tree::ROS2ActionClientNode<ComputePathToPose>
{
public:
  explicit ComputePathToPoseClient(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<ComputePathToPose>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "The destination to plan to"),
        BT::InputPort<std::string>("planner_id", "The ID of the planner to use"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "The path to the destination")
      });
  }

  void read_input_ports(ComputePathToPose::Goal & goal) override
  {
    if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal.pose)) {
      throw BT::RuntimeError("Missing parameter [goal] in ComputePathToPoseClient node");
    }

    if (!getInput<std::string>("planner_id", goal.planner_id)) {
      throw BT::RuntimeError("Missing parameter [planner_id] in ComputePathToPoseClient node");
    }
  }

  void write_output_ports(
    rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult & result) override
  {
    setOutput<nav_msgs::msg::Path>("path", result.result->path);
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__COMPUTE_PATH_TO_POSE_NODE_HPP_
