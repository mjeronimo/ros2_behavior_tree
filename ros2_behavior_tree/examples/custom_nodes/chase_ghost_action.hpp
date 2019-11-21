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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__CHASE_GHOST_ACTION_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__CHASE_GHOST_ACTION_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace ros2_behavior_tree
{

class ChaseGhostAction : public BT::CoroActionNode
{
public:
  explicit ChaseGhostAction(const std::string & action_name)
  : BT::CoroActionNode(action_name, {})
  {
  }

  typedef std::chrono::high_resolution_clock::time_point TimePoint;

  BT::NodeStatus tick() override
  {
    auto Now = []() {return std::chrono::high_resolution_clock::now();};

    TimePoint initial_time = Now();
    TimePoint time_before_reply = initial_time + std::chrono::milliseconds(1000);

    bool caught_ghost = false;

    std::cerr << "Chasing ghost ";
    while (!caught_ghost) {
      if (Now() >= time_before_reply) {
        caught_ghost = true;
      }

      if (!caught_ghost) {
        std::cerr << ">";

        // Set status to RUNNING and "pause/sleep"
        // If halt() is called, we will NOT resume execution
        setStatusRunningAndYield();
      }
    }

    // This part of the code is never reached if halt() is invoked, only if caught_ghost == true
    std::cerr << "\n";
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    // There may be some work to do upon halting. If not, this method can be omitted and the
    // base classes halt() method will be called

    CoroActionNode::halt();
  }
};

}  // namespace ros2_behavior_tree

#endif  //  ROS2_BEHAVIOR_TREE__EXAMPLES__CUSTOM_NODES__CHASE_GHOST_ACTION_HPP_
