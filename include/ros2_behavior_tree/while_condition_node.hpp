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

#ifndef ROS2_BEHAVIOR_TREE__WHILE_CONDITION_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__WHILE_CONDITION_NODE_HPP_

#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace ros2_behavior_tree
{

class WhileConditionNode : public BT::DecoratorNode
{
public:
  WhileConditionNode(const std::string & name, const BT::NodeConfiguration & cfg)
  : BT::DecoratorNode(name, cfg)
  {
    getInput<std::string>("key", key_);

    // Convert the XML string param to a boolean
    getInput<bool>("value", target_value_);
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("key", "The target key to use"),
      BT::InputPort<bool>("value", "The target value")};
  }

private:
  BT::NodeStatus tick() override;

  std::string key_;
  bool target_value_;
};

inline BT::NodeStatus WhileConditionNode::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  child_node_->executeTick();

  bool current_value = false;
  config().blackboard->get<bool>(key_, current_value);

  // We're waiting for the value on the blackboard to match the target
  return (current_value == target_value_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__WHILE_CONDITION_NODE_HPP_
