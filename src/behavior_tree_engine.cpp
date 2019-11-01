// Copyright (c) 2018 Intel Corporation
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
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace ros2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(std::string{"lib" + p + ".so"});
  }
}

BtStatus
BehaviorTreeEngine::run(
  BT::Blackboard::Ptr & blackboard,
  const std::string & behavior_tree_xml,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  // Parse the input XML and create the corresponding Behavior Tree
  BT::Tree tree = buildTreeFromText(behavior_tree_xml, blackboard);

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

  // Loop until something happens with ROS or the node completes
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
BehaviorTreeEngine::buildTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  BT::XMLParser p(factory_);
  p.loadFromText(xml_string);
  return p.instantiateTree(blackboard);
}

}  // namespace ros2_behavior_tree
