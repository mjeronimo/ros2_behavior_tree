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

#ifndef ROS2_BEHAVIOR_TREE__DECORATOR__THROTTLE_TICK_RATE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__DECORATOR__THROTTLE_TICK_RATE_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace ros2_behavior_tree
{

class ThrottleTickRateNode : public BT::DecoratorNode
{
public:
  ThrottleTickRateNode(const std::string & name, double hz)
  : BT::DecoratorNode(name, {}), read_parameters_from_ports_(false)
  {
    setRegistrationID("ThrottleTickRate");
    period_ = 1.0 / hz;
  }

  ThrottleTickRateNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config), read_parameters_from_ports_(true)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    if (read_parameters_from_ports_) {
      double hz = 1.0;
      if (!getInput("hz", hz)) {
        throw BT::RuntimeError("Missing parameter [hz] in ThrottleTickRate node");
      }
      period_ = 1.0 / hz;
    }

    if (status() == BT::NodeStatus::IDLE) {
      // Reset the start time since we're beginning a new iteration
      // (transitioning from IDLE to RUNNING)
      start_ = std::chrono::high_resolution_clock::now();
      first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Determine how long its been since we've started this iteration
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // Now, get that elapsed time in seconds
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    auto child_running = child_node_->status() == BT::NodeStatus::RUNNING;

    // The child gets ticked the first time through and any time the period has
    // expired. In addition, once the child begins to run, it is ticked each time
    if (first_time_ || child_running || seconds.count() >= period_) {
      first_time_ = false;

      auto child_state = child_node_->executeTick();

      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          start_ = std::chrono::high_resolution_clock::now();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  bool read_parameters_from_ports_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_{0.0};
  bool first_time_{false};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__DECORATOR__THROTTLE_TICK_RATE_NODE_HPP_
