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
#include "ros2_behavior_tree/repeat_until_node.hpp"
#include "stub_action_test_node.hpp"

struct RepeatUntilWithStubAction : testing::Test
{
  RepeatUntilWithStubAction()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Create a node configurand and populate the blackboard
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    blackboard_->set("key", "target_key");
    blackboard_->set("value", "true");

    // Update the configuration with the child node's ports and tell the child node
    // to use this configuration
    BT::assignDefaultRemapping<StubActionTestNode>(config);
    child_action_ = std::make_unique<StubActionTestNode>("child", config);

    // Update the configuration with the parent node's ports and tell the parent node
    // to use this configuration
    BT::assignDefaultRemapping<ros2_behavior_tree::RepeatUntilNode>(config);
    root_ = std::make_unique<ros2_behavior_tree::RepeatUntilNode>("repeat_until", config);

    // Create the tree structure
    root_->setChild(child_action_.get());
  }

  ~RepeatUntilWithStubAction()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::RepeatUntilNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(RepeatUntilWithStubAction, RunningUponSuccess)
{
  // If the child returns SUCCESS, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RepeatUntilWithStubAction, RunningUponRunning)
{
  // If the child returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
}

TEST_F(RepeatUntilWithStubAction, FailureUponFailure)
{
  // If the child returns FAILURE, the root should return FAILURE
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RepeatUntilWithStubAction, HaltAfterRunning)
{
  // If the nodes are halted after running, they should go IDLE
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
  root_->halt();
  ASSERT_EQ(root_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RepeatUntilWithStubAction, CheckValueOnBlackboard)
{
  // Start everything running
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Set the flag on the blackboard
  std::string key_name;
  blackboard_->get<std::string>("key", key_name);
  blackboard_->set<bool>(key_name, true);

  // Which should cause the decorator to return SUCCESS. The
  // child node, the fake action, is still running
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Turn off the flag and both should be RUNNING again
  blackboard_->set<bool>(key_name, false);  // NOLINT
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Turn it on one more time and check
  blackboard_->set<bool>(key_name, true);  // NOLINT
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Upon halting, both should go IDLE
  root_->halt();
  ASSERT_EQ(root_->status(), BT::NodeStatus::IDLE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}
