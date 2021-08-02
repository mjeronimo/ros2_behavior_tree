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

#ifndef ROS2_BEHAVIOR_TREE__DECORATOR__FOR_EACH_POSE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__DECORATOR__FOR_EACH_POSE_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_behavior_tree
{

class ForEachPoseNode : public BT::DecoratorNode
{
public:
  ForEachPoseNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "Poses to iterate over"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "The next pose in the sequence")
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("poses", poses)) {
      throw BT::RuntimeError("Missing parameter [poses] in GetNextPose node");
    }

    if (poses.size() == 0) {
      return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);

    if (!setOutput<geometry_msgs::msg::PoseStamped>("pose", poses[0])) {
      throw BT::RuntimeError("Failed to set output port value [next_pose] for GetNextPose");
    }

    for (int i = 0; i < poses.size(); ) {
      if (!setOutput<geometry_msgs::msg::PoseStamped>("pose", poses[i])) {
        throw BT::RuntimeError("Failed to set output port value [next_pose] for GetNextPose");
      }

      auto child_state = child_node_->executeTick();

      switch (child_state) {
        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::RUNNING:
          continue;

        case BT::NodeStatus::FAILURE:
          // Try the next one
          i++;
          continue;

        default:
          throw BT::LogicError("Invalid status return from BT node");
      }
    }

    // None of the poses worked, so fail
    child()->halt();
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__DECORATOR__FOR_EACH_POSE_NODE_HPP_
