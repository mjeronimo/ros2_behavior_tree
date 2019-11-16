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

#include <gtest/gtest.h>
#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "ros2_behavior_tree/throttle_tick_count_node.hpp"
#include "stub_action_test_node.hpp"

struct ThrottleTickCountWithStubAction : testing::Test
{
  ThrottleTickCountWithStubAction()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Create a node configurand and populate the blackboard
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    blackboard_->set("hz", "10");

    // Update the configuration with the child node's ports and tell the child node
    // to use this configuration
    BT::assignDefaultRemapping<StubActionTestNode>(config);
    child_action_ = std::make_unique<StubActionTestNode>("child", config);

    // Update the configuration with the parent node's ports and tell the parent node
    // to use this configuration
    BT::assignDefaultRemapping<ros2_behavior_tree::ThrottleTickCountNode>(config);
    root_ =
      std::make_unique<ros2_behavior_tree::ThrottleTickCountNode>("throttle_tick_count", config);

    // Create the tree structure
    root_->setChild(child_action_.get());
  }

  ~ThrottleTickCountWithStubAction()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::ThrottleTickCountNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(ThrottleTickCountWithStubAction, FirstTimeSuccess)
{
  // If the child immediately returns SUCCESS, the ThrottleTickCount
  // decorator allows it to pass
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(ThrottleTickCountWithStubAction, FailureUponFailure)
{
  // If the child returns FAILURE, the parent should also return FAILURE
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(ThrottleTickCountWithStubAction, WaitForDurationWithRunning)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Now, if the child returns RUNNING, the ThrottleTickCount decorator
  // will also return RUNNING, even though the period has expired
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
}

TEST_F(ThrottleTickCountWithStubAction, WaitForDurationWithSuccess)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Now, if the child returns SUCCESS, the ThrottleTickCount decorator
  // will return SUCCESS
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(ThrottleTickCountWithStubAction, WaitForDurationWithFailure)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // The Throttle node won't fire until the child returns SUCCESS or FAILURE
  // even if the period has expired

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  child_action_->set_return_value(BT::NodeStatus::RUNNING);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  child_action_->set_return_value(BT::NodeStatus::FAILURE);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(ThrottleTickCountWithStubAction, CheckClockResetOnSuccess)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // The Throttle node won't fire until the child returns SUCCESS or FAILURE
  // and the period has expired

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);

  // Since the period will not have expired again yet, the parent
  // should still be running
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}