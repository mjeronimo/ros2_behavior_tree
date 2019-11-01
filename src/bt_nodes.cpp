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

#include "ros2_behavior_tree/bt_conversions.hpp"
#include "ros2_behavior_tree/bt_nodes.hpp"
#include "ros2_behavior_tree/conditional_loop_node.hpp"
#include "ros2_behavior_tree/rate_controller_node.hpp"
#include "ros2_behavior_tree/recovery_node.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Register our custom action nodes

  // Register our custom condition nodes

  // Register our simple condition nodes

  // Register our custom decorator nodes
  factory.registerNodeType<ros2_behavior_tree::ConditionalLoop>("ConditionalLoop");
  factory.registerNodeType<ros2_behavior_tree::RateController>("RateController");

  // Register our custom control nodes
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("RecoveryNode");
}

}  // namespace ros2_behavior_tree
