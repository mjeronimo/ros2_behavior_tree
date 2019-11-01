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
  ros2_behavior_tree::BtRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void 
BtRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Register our custom action nodes

  // Register our custom condition nodes

  // Register our simple condition nodes

  // Register our custom decorator nodes
  factory.registerNodeType<ros2_behavior_tree::ConditionalLoop>("ConditionalLoop");
  factory.registerNodeType<ros2_behavior_tree::RateController>("RateController");

  const BT::PortsList message_params {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("Message",
    std::bind(&BtRegistrar::message, std::placeholders::_1), message_params);

  const BT::PortsList set_condition_params {BT::InputPort<std::string>("key"), BT::InputPort<std::string>("value")};
  factory.registerSimpleAction("SetCondition",
    std::bind(&BtRegistrar::setCondition, std::placeholders::_1), set_condition_params);

  // Register our custom control nodes
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("RecoveryNode");
}

#if 0
void
BtRegistrar::registerSimpleActionWithParameters(
  BT::BehaviorTreeFactory & factory,
  const std::string & ID,
  const BT::SimpleActionNode::TickFunctor & tick_functor,
  const BT::PortsList & ports)
{
  BT::NodeBuilder builder = [tick_functor, ID](const std::string & name, const BT::NodeConfiguration & config) {
      //return std::unique_ptr<BT::TreeNode>(new BT::SimpleActionNode(name, tick_functor, config));
      return std::make_unique<BT::SimpleActionNode>(name, tick_functor, config);
    };

  BT::TreeNodeManifest manifest = {BT::NodeType::ACTION, ID, std::move(ports)};
  factory.registerBuilder(manifest, builder);
}
#endif

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
BtRegistrar::message(BT::TreeNode & tree_node)
{
  std::string msg;
  //tree_node.getParam<std::string>("msg", msg);
  tree_node.getInput<std::string>("msg", msg);

  //RCLCPP_INFO(service_client_node_->get_logger(),
  printf(ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET "\n", msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
BtRegistrar::setCondition(BT::TreeNode & tree_node)
{
#if 0
  std::string key;
  tree_node.getParam<std::string>("key", key);

  std::string value;
  tree_node.getParam<std::string>("value", value);

  tree_node.blackboard()->template set<bool>(key, (value == "true") ? true : false);  // NOLINT
#endif
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree
