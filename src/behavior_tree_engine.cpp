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

#include "ros2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>

#include "ros2_behavior_tree/conditional_loop_node.hpp"
#include "ros2_behavior_tree/rate_controller_node.hpp"
#include "ros2_behavior_tree/recovery_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace ros2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine()
{
  // Register our custom node types

  factory_.registerNodeType<RecoveryNode>("RecoveryNode");
  factory_.registerNodeType<RateController>("RateController");
  factory_.registerNodeType<ConditionalLoop>("ConditionalLoop");

  BT::PortsList message_ports{ BT::InputPort<std::string>("msg", "The message to output") };
  factory_.registerSimpleAction("Message",
    std::bind(&BehaviorTreeEngine::message, this, std::placeholders::_1),
    message_ports);

  BT::PortsList set_condition_ports{ BT::InputPort<std::string>("key", "The key to use"), BT::OutputPort<std::string>("value") };
  factory_.registerSimpleAction("SetCondition",
    std::bind(&BehaviorTreeEngine::setCondition, this, std::placeholders::_1),
    set_condition_ports);
}

BtStatus
BehaviorTreeEngine::run(
  const std::string & behavior_tree_xml,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // Parse the input XML and create the corresponding Behavior Tree
  BT::Tree tree = factory_.createTreeFromText(behavior_tree_xml);

  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree.root_node->halt();
      return BtStatus::CANCELED;
    }

    onLoop();
    result = tree.root_node->executeTick();
    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BtStatus
BehaviorTreeEngine::run(
  std::unique_ptr<BT::Tree> & tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes w/ success or failure
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree->root_node->halt();
      return BtStatus::CANCELED;
    }

    onLoop();
    result = tree->root_node->executeTick();
    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::buildTreeFromText(std::string & xml_string)
{
  return factory_.createTreeFromText(xml_string);
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
BehaviorTreeEngine::message(BT::TreeNode & tree_node)
{
  std::string msg;
  // tree_node.getParam<std::string>("msg", msg);
  tree_node.getInput<std::string>("msg", msg);

  RCLCPP_INFO(rclcpp::get_logger("BehaviorTreeEngine"),
     ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
BehaviorTreeEngine::setCondition(BT::TreeNode & tree_node)
{
  std::string key;
  //tree_node.getParam<std::string>("key", key);
  tree_node.getInput<std::string>("key", key);

  //std::string value;
  //tree_node.getInput<std::string>("value", value);

  //tree_node.blackboard()->template set<bool>(key, (value == "true") ? true : false);  // NOLINT
  //TODO:
  //tree_node.setOutput<std::string>("value", value);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree
