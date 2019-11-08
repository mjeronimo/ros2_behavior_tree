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

#include "sample_action_server_node.hpp"

#include <memory>
#include <string>

using namespace std::placeholders;

namespace ros2_behavior_tree
{

// The Behavior Tree to execute
const char SampleActionServerNode::bt_xml_[] =
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

SampleActionServerNode::SampleActionServerNode()
: Node("sample_action_server_node")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Create an action server that we implement with our printMessage method
  action_server_ = rclcpp_action::create_server<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "PrintMessage",
    std::bind(&SampleActionServerNode::handle_goal, this, _1, _2),
    std::bind(&SampleActionServerNode::handle_cancel, this, _1),
    std::bind(&SampleActionServerNode::handle_accepted, this, _1)
  );
}

SampleActionServerNode::~SampleActionServerNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

rclcpp_action::GoalResponse
SampleActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionServer::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SampleActionServerNode::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
SampleActionServerNode::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{
    std::bind(&SampleActionServerNode::printMessage, this, _1), goal_handle
  }.detach();
}

void
SampleActionServerNode::printMessage(const std::shared_ptr<GoalHandle> goal_handle)
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
      RCLCPP_INFO(get_logger(), "Behavior Tree execution failed!");
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
