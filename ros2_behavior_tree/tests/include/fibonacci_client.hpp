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

#include <memory>
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
      BT::OutputPort<std::vector<int32_t>>("feedback",
      "Feedback from the server while computing the full fib(n) sequence"),
      BT::OutputPort<std::vector<int32_t>>("sequence", "The output fibonacci sequence up to n")
    });
  }

  void read_input_ports(Fibonacci::Goal & goal) override
  {
    if (!getInput<int32_t>("n", goal.order)) {
      throw BT::RuntimeError("Missing parameter [n] in Fibonacci node");
    }
  }

  bool read_new_goal(Fibonacci::Goal & goal) override
  {
    // Get the current value of 'n' from the input port (may have been updated)
    int32_t n = 0;
    if (!getInput<int32_t>("n", n)) {
      throw BT::RuntimeError("Missing parameter [n] in Fibonacci node");
    }

    // If it's not the same as the goal we're currently working on, update the goal
    // and return true
    return (n != goal_.order) ? goal_.order = n, true : false;
  }

  void write_feedback_ports(const std::shared_ptr<const Fibonacci::Feedback> feedback) override
  {
    RCLCPP_INFO(ros2_node_->get_logger(),
      "Feedback: Next number in sequence: %d", feedback->sequence.back());
    setOutput<std::vector<int32_t>>("feedback", feedback->sequence);
  }

  void write_output_ports(
    rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result) override
  {
    setOutput<std::vector<int32_t>>("sequence", result_.result->sequence);
  }
};

#endif  // FIBONACCI_CLIENT_HPP_
