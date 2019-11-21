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

#ifndef FIBONACCI_CLIENT_HPP_
#define FIBONACCI_CLIENT_HPP_

#include <string>
#include <vector>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

class FibonacciClient : public ros2_behavior_tree::ROS2ActionClientNode<Fibonacci>
{
public:
  explicit FibonacciClient(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<Fibonacci>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
      BT::InputPort<int32_t>("n", "Compute the fibonnaci sequence up to the nth value"),
      BT::OutputPort<std::vector<int32_t>>("sequence", "The output fibonacci sequence up to n")
    });
  }

  void read_input_ports() override
  {
    if (!getInput<int32_t>("n", goal_.order)) {
      throw BT::RuntimeError("Missing parameter [n] in Fibonacci node");
    }
  }

  void write_output_ports() override
  {
    // auto & sequence = result_.result->sequence;
    // std::stringstream ss;
    // std::copy(sequence.begin(), sequence.end(), std::ostream_iterator<int>(ss, ";"));
    setOutput<std::vector<int32_t>>("sequence", result_.result->sequence);
  }

  bool new_goal_received() override
  {
    // Get the current value of 'n' from the input port (may have been updated)
    int32_t n = 0;
    if (!getInput<int32_t>("n", n)) {
      throw BT::RuntimeError("Missing parameter [n] in Fibonacci node");
    }

    // If it's not the same as the goal we're currently working on, update the goal
    // and return true
    if (n != goal_.order) {
      goal_.order = n;
      return true;
    }

    // Otherwise, continue with the current goal
    return false;
  }

private:
};

#endif  // FIBONACCI_CLIENT_HPP_
