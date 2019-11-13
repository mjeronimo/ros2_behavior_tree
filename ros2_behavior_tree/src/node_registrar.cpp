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

#include "ros2_behavior_tree/node_registrar.hpp"

#include <string>

#include "ros2_behavior_tree/async_wait_node.hpp"
#include "ros2_behavior_tree/forever_node.hpp"
#include "ros2_behavior_tree/recovery_node.hpp"
#include "ros2_behavior_tree/repeat_until_node.hpp"
#include "ros2_behavior_tree/throttle_tick_count_node.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::NodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
NodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Condition nodes

  // Action nodes
  factory.registerNodeType<ros2_behavior_tree::AsyncWait>("AsyncWait");

  const BT::PortsList message_params {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("Message",
    std::bind(&NodeRegistrar::message, std::placeholders::_1), message_params);

  const BT::PortsList set_condition_params {
    BT::InputPort<std::string>("key"), BT::InputPort<std::string>("value")};
  factory.registerSimpleAction("SetCondition",
    std::bind(&NodeRegistrar::set_condition, std::placeholders::_1), set_condition_params);

  const BT::PortsList wait_params {BT::InputPort<int>("msec")};
  factory.registerSimpleAction("Wait",
    std::bind(&NodeRegistrar::wait, std::placeholders::_1), wait_params);

  // Decorator nodes
  factory.registerNodeType<ros2_behavior_tree::ForeverNode>("Forever");
  factory.registerNodeType<ros2_behavior_tree::RepeatUntilNode>("RepeatUntil");
  factory.registerNodeType<ros2_behavior_tree::ThrottleTickCountNode>("ThrottleTickCount");

  // Control nodes
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("RecoveryNode");
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
NodeRegistrar::message(BT::TreeNode & tree_node)
{
  std::string msg;
  tree_node.getInput<std::string>("msg", msg);

  printf(ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET "\n", msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
NodeRegistrar::set_condition(BT::TreeNode & tree_node)
{
  std::string key;
  tree_node.getInput<std::string>("key", key);

  std::string value;
  tree_node.getInput<std::string>("value", value);

  tree_node.config().blackboard->template set<bool>(key, (value == "true") ? true : false);  // NOLINT

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
NodeRegistrar::wait(BT::TreeNode & tree_node)
{
  int msec = 0;
  tree_node.getInput<int>("msec", msec);
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree
