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

//#include "ros2_behavior_tree/async_wait_node.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::SampleNodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
SampleNodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
#if 0
  // Condition nodes

  // Action nodes
  factory.registerNodeType<ros2_behavior_tree::AsyncWait>("AsyncWait");

  const BT::PortsList message_params {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("Message",
    std::bind(&SampleNodeRegistrar::message, std::placeholders::_1), message_params);

  const BT::PortsList set_condition_params {
    BT::InputPort<std::string>("key"), BT::InputPort<std::string>("value")};
  factory.registerSimpleAction("SetCondition",
    std::bind(&SampleNodeRegistrar::setCondition, std::placeholders::_1), set_condition_params);

  const BT::PortsList wait_params {BT::InputPort<int>("msec")};
  factory.registerSimpleAction("Wait",
    std::bind(&SampleNodeRegistrar::wait, std::placeholders::_1), wait_params);

  // Decorator nodes
  factory.registerNodeType<ros2_behavior_tree::Forever>("Forever");
  factory.registerNodeType<ros2_behavior_tree::RateController>("RateController");
  factory.registerNodeType<ros2_behavior_tree::RepeatUntilNode>("RepeatUntil");

  // Control nodes
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("RecoveryNode");
#endif
}

#if 0
BT::NodeStatus
SampleNodeRegistrar::setCondition(BT::TreeNode & tree_node)
{
  std::string key;
  tree_node.getInput<std::string>("key", key);

  std::string value;
  tree_node.getInput<std::string>("value", value);

  tree_node.config().blackboard->template set<bool>(key, (value == "true") ? true : false);  // NOLINT

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
SampleNodeRegistrar::wait(BT::TreeNode & tree_node)
{
  int msec = 0;
  tree_node.getInput<int>("msec", msec);
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));

  return BT::NodeStatus::SUCCESS;
}
#endif

}  // namespace ros2_behavior_tree
