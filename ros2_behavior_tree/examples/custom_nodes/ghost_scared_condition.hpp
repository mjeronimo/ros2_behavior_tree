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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__GHOST_SCARED_CONDITION_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__GHOST_SCARED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace ros2_behavior_tree
{

class GhostScaredCondition : public BT::ConditionNode
{
public:
  GhostScaredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
  }

  GhostScaredCondition() = delete;

  ~GhostScaredCondition()
  {
  }

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    printf("GhostScaredCondition::tick\n");
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__GHOST_SCARED_CONDITION_HPP_
