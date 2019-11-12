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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__EAT_PILLS_ACTION_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__EAT_PILLS_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace ros2_behavior_tree
{

class EatPillsAction : public BT::SyncActionNode
{
public:
  explicit EatPillsAction(const std::string & action_name)
  : BT::SyncActionNode(action_name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "Eating pill\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  //  ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__EAT_PILLS_ACTION_HPP_
