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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros2_behavior_tree/decorator/forever_node.hpp"
#include "stub_action_test_node.hpp"

struct TestForeverNode : testing::Test
{
  TestForeverNode()
  {
    BT::NodeConfiguration config;

    // Update the configuration with the child node's ports and tell the child node
    // to use this configuration
    BT::assignDefaultRemapping<StubActionTestNode>(config);
    child_action_ = std::make_unique<StubActionTestNode>("child", config);

    root_ = std::make_unique<ros2_behavior_tree::ForeverNode>("forever");

    // Create the tree structure
    root_->setChild(child_action_.get());
  }

  ~TestForeverNode()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::ForeverNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(TestForeverNode, ChildReturnsSuccess)
{
  // If the child returns SUCCESS, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestForeverNode, ChildReturnsRunning)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
}

TEST_F(TestForeverNode, ChildReturnsFailure)
{
  // If the child returns FAILURE, the root should return FAILURE
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestForeverNode, IdleAfterHalt)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // After halting, all nodes should be IDLE
  root_->halt();
  ASSERT_EQ(root_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}
