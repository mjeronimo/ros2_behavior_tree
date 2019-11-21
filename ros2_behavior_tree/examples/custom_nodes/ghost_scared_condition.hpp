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

#include <iostream>
#include <random>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace ros2_behavior_tree
{

class GhostScaredCondition : public BT::ConditionNode
{
public:
  explicit GhostScaredCondition(const std::string & condition_name)
  : BT::ConditionNode(condition_name, {}), gen_(rd_()), dis_(1, 2)
  {
  }

  BT::NodeStatus tick() override
  {
    auto ghost_scared = (dis_(gen_) == 1);

    if (ghost_scared) {
      std::cerr << "The ghost is scared!\n";
      return BT::NodeStatus::SUCCESS;
    }

    std::cerr << "The ghost is NOT scared!\n";
    return BT::NodeStatus::FAILURE;
  }

private:
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__GHOST_SCARED_CONDITION_HPP_
