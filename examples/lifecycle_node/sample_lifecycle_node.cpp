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

#include "sample_lifecycle_node.hpp"

#include <memory>

namespace ros2_behavior_tree
{

// The Behavior Tree to execute
const char SampleLifecycleNode::bt_xml_[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="say_hello">
      <SetCondition key="done" value="true"/>
      <WhileCondition key="done" value="false">
        <Message msg="Hello, World!"/>
      </WhileCondition>
    </Sequence>
  </BehaviorTree>
</root>
)";

SampleLifecycleNode::SampleLifecycleNode()
: rclcpp_lifecycle::LifecycleNode("sample_lifecycle_node")
{
  RCLCPP_INFO(get_logger(), "Creating");
}

SampleLifecycleNode::~SampleLifecycleNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

CallbackReturn
SampleLifecycleNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleLifecycleNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  bt_ = std::make_unique<BehaviorTree>(bt_xml_);

    // Execute the Behavior Tree on a separate thread
  thread_ = std::make_unique<std::thread>(&SampleLifecycleNode::executeBehaviorTree, this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  should_halt_ = true;
  thread_->join();
  bt_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleLifecycleNode::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

void
SampleLifecycleNode::executeBehaviorTree()
{
  should_halt_ = false;
  auto halt_requested = [this]() {return should_halt_.load();};

  ros2_behavior_tree::BtStatus rc = bt_->execute(halt_requested);

  switch (rc) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      break;

    case ros2_behavior_tree::BtStatus::HALTED:
      RCLCPP_INFO(get_logger(), "Navigation halted");
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

}  // namespace ros2_behavior_tree
