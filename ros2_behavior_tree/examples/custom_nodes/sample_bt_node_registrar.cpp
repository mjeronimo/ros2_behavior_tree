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

#include "sample_bt_node_registrar.hpp"

#include "avoid_ghost_action.hpp"
#include "chase_ghost_action.hpp"
#include "eat_pills_action.hpp"
#include "ghost_close_condition.hpp"
#include "ghost_scared_condition.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::SampleBtNodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
SampleBtNodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Register any custom condition, action, decorator, or control nodes. These will
  // be implemented using a class derived from one of the BT.CPP base classes. You can
  // also register any custom simple condition, action, or decorator nodes. These simply
  // require a tick functor (function or method) and not a separate class.

  factory.registerNodeType<ros2_behavior_tree::AvoidGhostAction>("AvoidGhost");
  factory.registerNodeType<ros2_behavior_tree::ChaseGhostAction>("ChaseGhost");
  factory.registerNodeType<ros2_behavior_tree::EatPillsAction>("EatPills");
  factory.registerNodeType<ros2_behavior_tree::GhostCloseCondition>("GhostClose");
  factory.registerNodeType<ros2_behavior_tree::GhostScaredCondition>("GhostScared");
}

}  // namespace ros2_behavior_tree
