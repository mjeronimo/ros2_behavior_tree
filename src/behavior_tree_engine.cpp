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
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_library_names)
{
  // Load any specified BT plugins
  for (const auto & library_name : plugin_library_names) {
    factory_.registerFromPlugin(std::string{"lib" + library_name + ".so"});
  }

  // Create a blackboard for this Behavior Tree
  blackboard_ = BT::Blackboard::create();
}

BtStatus
BehaviorTreeEngine::run(
  const std::string & behavior_tree_xml,
  std::function<void()> on_loop_iteration,
  std::function<bool()> cancel_requested,
  std::chrono::milliseconds tick_period)
{
  // Parse the input XML
  BT::XMLParser p(factory_);
  p.loadFromText(behavior_tree_xml);

  // Create the corresponding Behavior Tree
  BT::Tree tree = p.instantiateTree(blackboard_);

  // Set up a loop rate controller based on the desired tick period
  rclcpp::WallRate loop_rate(tick_period);

  // Loop until something happens with ROS or the node completes
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancel_requested()) {
      tree.root_node->halt();
      return BtStatus::CANCELED;
    }

    // Give the caller a chance to do something on each loop iteration
    on_loop_iteration();

    // Execute one tick of the tree
    result = tree.root_node->executeTick();
    loop_rate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

}  // namespace ros2_behavior_tree
