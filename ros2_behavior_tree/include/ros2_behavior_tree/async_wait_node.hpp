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

#ifndef ROS2_BEHAVIOR_TREE__ASYNC_WAIT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ASYNC_WAIT_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace ros2_behavior_tree
{

class AsyncWaitNode : public BT::AsyncActionNode
{
public:
  AsyncWaitNode(const std::string & name, int milliseconds)
  : BT::AsyncActionNode(name, {}),
    wait_duration_(milliseconds), read_parameters_from_ports_(false)
  {
    setRegistrationID("AsyncWait");
  }

  AsyncWaitNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config), read_parameters_from_ports_(true)
  {
  }

  // Define this node's ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("msec")
    };
  }

  // An AsyncActionNode must provide a halt override
  void halt() override
  {
    stopAndJoinThread();
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  BT::NodeStatus tick() override
  {
    if (read_parameters_from_ports_) {
      if (!getInput<int>("msec", wait_duration_)) {
        throw BT::RuntimeError("Missing parameter [msec] in AsyncWait node");
      }
    }

    // An AsyncActionNode's tick method is happening on a separate thread, so we
    // can simply cause this thread to sleep for the specified duration

    std::this_thread::sleep_for(std::chrono::milliseconds(wait_duration_));
    return BT::NodeStatus::SUCCESS;
  }

    bool read_parameters_from_ports_;
    int wait_duration_{0};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ASYNC_WAIT_NODE_HPP_
