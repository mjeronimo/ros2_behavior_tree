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

#include "sample_action_server_lifecycle_node.hpp"

#include <memory>
#include <string>

using namespace std::placeholders;

namespace ros2_behavior_tree
{

// The Behavior Tree to execute
const char SampleActionServerLifecycleNode::bt_xml_[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Repeat num_cycles="{iterations}">
      <SequenceStar name="do_work">
        <Message msg="{message}"/>
        <AsyncWait msec="{pause_ms}"/>
      </SequenceStar>
    </Repeat>
  </BehaviorTree>
</root>
)";

SampleActionServerLifecycleNode::SampleActionServerLifecycleNode()
: rclcpp_lifecycle::LifecycleNode("sample_action_server_lifecycle_node")
{
  RCLCPP_INFO(get_logger(), "Creating");
}

SampleActionServerLifecycleNode::~SampleActionServerLifecycleNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

CallbackReturn
SampleActionServerLifecycleNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleActionServerLifecycleNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Create an action server that we implement with our print_message method
  action_server_ = rclcpp_action::create_server<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "PrintMessage",
    std::bind(&SampleActionServerLifecycleNode::handle_goal, this, _1, _2),
    std::bind(&SampleActionServerLifecycleNode::handle_cancel, this, _1),
    std::bind(&SampleActionServerLifecycleNode::handle_accepted, this, _1)
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleActionServerLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  action_server_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleActionServerLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleActionServerLifecycleNode::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SampleActionServerLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
SampleActionServerLifecycleNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionServer::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SampleActionServerLifecycleNode::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
SampleActionServerLifecycleNode::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{
    std::bind(&SampleActionServerLifecycleNode::print_message, this, _1), goal_handle
  }.detach();
}

void
SampleActionServerLifecycleNode::print_message(const std::shared_ptr<GoalHandle> goal_handle)
{
  BehaviorTree bt(bt_xml_);
  auto result = std::make_shared<ActionServer::Result>();

  // Get the incoming goal from the goal handle
  auto goal = goal_handle->get_goal();

  // Pass the values from the goal to the Behavior Tree via the blackboard
  bt.blackboard()->set<std::string>("message", goal->message);  // NOLINT
  bt.blackboard()->set<int>("iterations", goal->iterations);  // NOLINT
  bt.blackboard()->set<int>("pause_ms", goal->pause_ms);  // NOLINT

  auto should_cancel = [goal_handle]() {return goal_handle->is_canceling();};

  switch (bt.execute(should_cancel)) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Behavior Tree execution succeeded");
      goal_handle->succeed(result);
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Behavior Tree execution failed!");
      goal_handle->abort(result);
      break;

    case ros2_behavior_tree::BtStatus::HALTED:
      RCLCPP_INFO(get_logger(), "Behavior Tree halted");
      goal_handle->canceled(result);
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

}  // namespace ros2_behavior_tree
