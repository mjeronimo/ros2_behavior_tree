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
#include "ros2_behavior_tree/first_result_node.hpp"
#include "stub_action_test_node.hpp"

struct FirstResultWithStubActions : testing::Test
{
  FirstResultWithStubActions()
  {
    blackboard_ = BT::Blackboard::create();

    BT::NodeConfiguration config;
    config.blackboard = blackboard_;

    BT::assignDefaultRemapping<StubActionTestNode>(config);
    first_child_ = std::make_unique<StubActionTestNode>("first_child", config);
    second_child_ = std::make_unique<StubActionTestNode>("second_child", config);

    root_ = std::make_unique<ros2_behavior_tree::FirstResultNode>("first_result_node");

    // Create the tree structure
    root_->addChild(first_child_.get());
    root_->addChild(second_child_.get());
  }

  ~FirstResultWithStubActions()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::FirstResultNode> root_;

  std::unique_ptr<StubActionTestNode> first_child_;
  std::unique_ptr<StubActionTestNode> second_child_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(FirstResultWithStubActions, FirstChildReturnsSuccess)
{
  // If the child returns SUCCESS, the root should return SUCCESS
  first_child_->set_return_value(BT::NodeStatus::SUCCESS);
  second_child_->set_return_value(BT::NodeStatus::IDLE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Because the child succeeded, it should become IDLE. The second
  // child should not be called and remains IDLE
  ASSERT_EQ(first_child_->get_tick_count(), 1);
  ASSERT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(second_child_->get_tick_count(), 0);
  ASSERT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(FirstResultWithStubActions, FirstChildReturnsFailure)
{
  // If the child returns FAILURE, the root should return FAILURE
  first_child_->set_return_value(BT::NodeStatus::FAILURE);
  second_child_->set_return_value(BT::NodeStatus::IDLE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);

  // Because the child failed, it should become IDLE. The second
  // child should not be called and remains IDLE
  ASSERT_EQ(first_child_->get_tick_count(), 1);
  ASSERT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(second_child_->get_tick_count(), 0);
  ASSERT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(FirstResultWithStubActions, FirstChildReturnsRunningSecondSuccess)
{
  // If the child returns RUNNING, the second child should also be ticked
  // In this case, it also returns SUCCESS, so the root should return SUCCESS
  first_child_->set_return_value(BT::NodeStatus::RUNNING);
  second_child_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Each of the children should have been ticked once and should be IDLE
  // because the operation succeeded, causing the node to halt its children
  ASSERT_EQ(first_child_->get_tick_count(), 1);
  ASSERT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(second_child_->get_tick_count(), 1);
  ASSERT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(FirstResultWithStubActions, FirstChildReturnsRunningSecondFailure)
{
  // If the child returns RUNNING, the second child should also be ticked
  // In this case, it also returns FAILURE, so the root should return FAILURE
  first_child_->set_return_value(BT::NodeStatus::RUNNING);
  second_child_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);

  // Each of the children should have been ticked once and should be IDLE
  // because the operation failed, causing the node to halt its children
  ASSERT_EQ(first_child_->get_tick_count(), 1);
  ASSERT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(second_child_->get_tick_count(), 1);
  ASSERT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(FirstResultWithStubActions, FirstChildReturnsRunningSecondRunning)
{
  // If the child returns RUNNING, the second child should also be ticked
  // In this case, it also returns RUNNING, so the root should return RUNNING
  first_child_->set_return_value(BT::NodeStatus::RUNNING);
  second_child_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);

  // Each of the children should have been ticked once and should be RUNNING
  ASSERT_EQ(first_child_->get_tick_count(), 1);
  ASSERT_EQ(first_child_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(second_child_->get_tick_count(), 1);
  ASSERT_EQ(second_child_->status(), BT::NodeStatus::RUNNING);
}