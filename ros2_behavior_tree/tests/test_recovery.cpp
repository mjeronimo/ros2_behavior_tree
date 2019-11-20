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
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "ros2_behavior_tree/recovery_node.hpp"
#include "stub_action_test_node.hpp"

struct TestRecoveryNode : testing::Test
{
  TestRecoveryNode()
  {
    blackboard_ = BT::Blackboard::create();

    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    blackboard_->set("number_of_retries", "1");

    BT::assignDefaultRemapping<StubActionTestNode>(config);
    child_action_ = std::make_unique<StubActionTestNode>("child_action", config);
    recovery_action_ = std::make_unique<StubActionTestNode>("recovery_action", config);

    BT::assignDefaultRemapping<ros2_behavior_tree::RecoveryNode>(config);
    root_ = std::make_unique<ros2_behavior_tree::RecoveryNode>("recovery_node", config);

    // Create the tree structure
    root_->addChild(child_action_.get());
    root_->addChild(recovery_action_.get());
  }

  ~TestRecoveryNode()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::RecoveryNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;
  std::unique_ptr<StubActionTestNode> recovery_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(TestRecoveryNode, ChildReturnsSuccess)
{
  // If the child returns SUCCESS, the root should return SUCCESS
  blackboard_->set("number_of_retries", "1");
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  recovery_action_->set_return_value(BT::NodeStatus::IDLE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Because the child succeeded, it should become IDLE. The recovery
  // should not be called and remains IDLE
  ASSERT_EQ(child_action_->get_tick_count(), 1);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 0);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRecoveryNode, ChildReturnsRunning)
{
  // If the child returns RUNNING, the root should return RUNNING
  blackboard_->set("number_of_retries", "1");
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  recovery_action_->set_return_value(BT::NodeStatus::IDLE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);

  // The RecoveryNode returned RUNNING and the child should have
  // only been ticked once and remains RUNNING. The recovery action
  // was not invoked and remains IDLE
  ASSERT_EQ(child_action_->get_tick_count(), 1);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(recovery_action_->get_tick_count(), 0);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRecoveryNode, ChildReturnsFailure)
{
  // If the child action fails each time, the recovery action
  // and the original action will be invoked up to the # of retries
  blackboard_->set("number_of_retries", "2");
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();

  // Because the original action never succeeds in this test, the
  // RecoveryNode ultimately returns FAILURE. The child action will
  // be invoked the first time + # of retries and will go IDLE
  // because it failed. The recovery action will be invoked
  // number_of_retries times.
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->get_tick_count(), 3);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 2);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRecoveryNode, RecoveryReturnsFailure)
{
  // If the recovery action returns FAILURE, the control node will return FAILURE
  blackboard_->set("number_of_retries", "3");
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  // Both nodes would have been called once, since the recovery fails the first time
  ASSERT_EQ(child_action_->get_tick_count(), 1);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 1);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRecoveryNode, RerunChildReturnsFailure)
{
  // Try a test case so that we can then reset and try again
  blackboard_->set("number_of_retries", "2");
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();

  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->get_tick_count(), 3);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 2);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);

  // Reset the tick counts (other state should be reset automatically)
  child_action_->reset_tick_count();
  recovery_action_->reset_tick_count();

  // Halting the root (RecoveryNode) causes it to enter the IDLE state
  root_->halt();

  // Try the test again to make sure we get the same (correct) results
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();

  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->get_tick_count(), 3);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 2);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}
