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

#ifndef ROS2_BEHAVIOR_TREE__DECORATOR__FOREVER_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__DECORATOR__FOREVER_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace ros2_behavior_tree
{

class ForeverNode : public BT::DecoratorNode
{
public:
  explicit ForeverNode(const std::string & name)
  : BT::DecoratorNode(name, {})
  {
  }

private:
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    auto child_state = child_node_->executeTick();

    // Run the child node forever unless there is a failure
    switch (child_state) {
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::FAILURE:
      default:
        child_node_->halt();
        return BT::NodeStatus::FAILURE;
    }
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__DECORATOR__FOREVER_NODE_HPP_
