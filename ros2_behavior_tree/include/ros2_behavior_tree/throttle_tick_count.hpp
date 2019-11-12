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

#ifndef ROS2_BEHAVIOR_TREE__THROTTLE_TICK_COUNT_HPP_
#define ROS2_BEHAVIOR_TREE__THROTTLE_TICK_COUNT_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace ros2_behavior_tree
{

class ThrottleTickCount : public BT::DecoratorNode
{
public:
  ThrottleTickCount(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    double hz = 1.0;
    getInput("hz", hz);
    period_ = 1.0 / hz;
  }

  // Define this node's ports
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("hz", 10.0, "Rate")};
  }

private:
  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      // Reset the starting point since we're starting a new iteration
      // (moving from IDLE to RUNNING)
      start_ = std::chrono::high_resolution_clock::now();
      first_time = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Determine how long its been since we've started this iteration
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // Now, get that elapsed time in seconds
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    // If we've reached or exceed the specified period, execute the child node
    if (first_time || seconds.count() >= period_) {
      first_time = false;
      const BT::NodeStatus child_state = child_node_->executeTick();

      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          child_node_->setStatus(BT::NodeStatus::IDLE);
          start_ = std::chrono::high_resolution_clock::now();  // Reset the starting time
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          child_node_->setStatus(BT::NodeStatus::IDLE);
          return BT::NodeStatus::FAILURE;
      }
    }
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time{false};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__THROTTLE_TICK_COUNT_HPP_
