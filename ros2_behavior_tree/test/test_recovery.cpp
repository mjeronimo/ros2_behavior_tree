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

struct RecoveryWithStubActions : testing::Test
{
  RecoveryWithStubActions()
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

  ~RecoveryWithStubActions()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::RecoveryNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;
  std::unique_ptr<StubActionTestNode> recovery_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(RecoveryWithStubActions, ChildReturnsSuccess)
{
  // If the child returns SUCCESS, the root should return SUCCESS
  blackboard_->set("number_of_retries", "1");
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  recovery_action_->set_return_value(BT::NodeStatus::IDLE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryWithStubActions, ChildReturnsFailure)
{
  // If the child action fails every time and the recovery
  // action succeeds each time, we will execute the child
  // a total of three times: the first time and two retries.
  blackboard_->set("number_of_retries", "2");
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();

  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->get_tick_count(), 3);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 2);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryWithStubActions, RerunChildReturnsFailure)
{
  // If the child action fails every time and the recovery
  // action succeeds each time, we will execute the child
  // a total of three times: the first time and two retries.
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

  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  recovery_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();

  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->get_tick_count(), 3);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(recovery_action_->get_tick_count(), 2);
  ASSERT_EQ(recovery_action_->status(), BT::NodeStatus::IDLE);
}