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

#ifndef ROS2_BEHAVIOR_TREE__EXAMPLES__NODE__SAMPLE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__EXAMPLES__NODE__SAMPLE_NODE_HPP_

#include <memory>
#include <thread>

#include "ros2_behavior_tree/behavior_tree.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_behavior_tree
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();
  ~SampleNode();

protected:
  // The node executes a Behavior Tree
  BehaviorTree bt_;

  // The XML string that defines the Behavior Tree to create and execute
  static const char bt_xml_[];

  // The thread on which to execute the Behavior Tree
  std::unique_ptr<std::thread> thread_;

  // The routine to run on the thread
  BtStatus executeBehaviorTree();
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__EXAMPLES__NODE__SAMPLE_NODE_HPP_
