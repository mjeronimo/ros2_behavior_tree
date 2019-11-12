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

#if 1
class EatPillsAction : public BT::CoroActionNode
{
public:
  EatPillsAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::CoroActionNode(action_name, conf)
  {
  }

  EatPillsAction() = delete;

  ~EatPillsAction()
  {
  }

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    printf("EatPillsAction::tick\n");
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }
};
#endif

}  // namespace ros2_behavior_tree

#endif  //  ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__EAT_PILLS_ACTION_HPP_
