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

#ifndef EXAMPLES__LIFECYCLE_NODE__SAMPLE_LIFECYCLE_NODE_HPP_
#define EXAMPLES__LIFECYCLE_NODE__SAMPLE_LIFECYCLE_NODE_HPP_

#include <memory>

#include "ros2_behavior_tree/behavior_tree.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_behavior_tree
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SampleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  SampleLifecycleNode();
  virtual ~SampleLifecycleNode();

protected:
  // The lifecycle node interface
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  // The node executes a Behavior Tree
  std::unique_ptr<BehaviorTree> bt_;

  // The XML string that defines the Behavior Tree to create and execute
  static const char bt_xml_[];

  // This node executes the Behavior Tree on a separate thread
  std::unique_ptr<std::thread> thread_;

  // The routine to execute on the separate thread
  void executeBehaviorTree();
};

}  // namespace ros2_behavior_tree

#endif  // EXAMPLES__LIFECYCLE_NODE__SAMPLE_LIFECYCLE_NODE_HPP_
