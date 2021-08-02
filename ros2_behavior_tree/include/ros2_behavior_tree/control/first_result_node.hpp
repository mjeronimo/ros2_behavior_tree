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

#ifndef ROS2_BEHAVIOR_TREE__CONTROL__FIRST_RESULT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONTROL__FIRST_RESULT_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace ros2_behavior_tree
{

class FirstResultNode : public BT::ControlNode
{
public:
  explicit FirstResultNode(const std::string & name)
  : BT::ControlNode::ControlNode(name, {})
  {
    setRegistrationID("FirstResult");
  }

  BT::NodeStatus tick() override
  {
    const unsigned num_children = children_nodes_.size();

    setStatus(BT::NodeStatus::RUNNING);

    for (current_child_idx_ = 0; current_child_idx_ < num_children; current_child_idx_++) {
      TreeNode * child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child_node->executeTick();

      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          haltChildren();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
          haltChildren();
          return BT::NodeStatus::FAILURE;

        case BT::NodeStatus::RUNNING:
          break;

        default:
          throw BT::LogicError("Invalid status return from BT node");
          break;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    ControlNode::halt();
    current_child_idx_ = 0;
  }

private:
  unsigned int current_child_idx_{0};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONTROL__FIRST_RESULT_NODE_HPP_
