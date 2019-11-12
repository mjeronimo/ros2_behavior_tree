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

#include "sample_node_registrar.hpp"

#include <string>

#include "eat_pills_action.hpp"
#include "ghost_close_condition.hpp"
#include "ghost_scared_condition.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::SampleNodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
SampleNodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Register any custom simple condition, action, or decorator nodes. These simply
  // require a tick functor (function or method)

  // A simple action with no input ports
  factory.registerSimpleAction("SayHello",
    std::bind(&SampleNodeRegistrar::say_hello, std::placeholders::_1));

  // A simple action with an input port
  const BT::PortsList say_something_ports {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("SaySomething",
    std::bind(&SampleNodeRegistrar::say_something, std::placeholders::_1), say_something_ports);

  // Register any custom condition, action, decorator, or control nodes. These will
  // be implemented using a class derived from one of the BT.CPP base classes.

  factory.registerNodeType<ros2_behavior_tree::GhostCloseCondition>("GhostClose");
  factory.registerNodeType<ros2_behavior_tree::GhostScaredCondition>("GhostScared");
  factory.registerNodeType<ros2_behavior_tree::EatPillsAction>("EatPills");
}

BT::NodeStatus
SampleNodeRegistrar::say_hello(BT::TreeNode & tree_node)
{
  printf("Hello!\n");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
SampleNodeRegistrar::say_something(BT::TreeNode & tree_node)
{
  printf("Say Something!\n");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree
