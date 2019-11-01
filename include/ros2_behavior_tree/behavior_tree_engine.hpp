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

#ifndef ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

namespace ros2_behavior_tree
{

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_library_names);
  BehaviorTreeEngine() = delete;

  virtual ~BehaviorTreeEngine() {}

  BtStatus run(
    const std::string & behavior_tree_xml,
    std::function<void()> on_loop_iteration = []() {},
    std::function<bool()> cancel_requested = []() {return false;},
    std::chrono::milliseconds tick_period = std::chrono::milliseconds(10));

protected:
  BT::Tree buildTreeFromText(const std::string & xml_string);

  // The factory and blackboard that will be used to dynamically construct the
  // behavior tree from the XML specification
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
