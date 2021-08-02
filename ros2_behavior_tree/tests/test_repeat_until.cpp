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
#include "ros2_behavior_tree/decorator/repeat_until_node.hpp"
#include "stub_action_test_node.hpp"

struct TestRepeatUntilNode : testing::Test
{
  TestRepeatUntilNode()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Create a node configurand and populate the blackboard
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;

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

  ~TestRepeatUntilNode()
  {
    //BT::haltAllActions(root_.get());
    root_->halt();
  }

  std::unique_ptr<ros2_behavior_tree::RepeatUntilNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(TestRepeatUntilNode, RunningUponSuccess)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // Without the target value set on the blackboard, if the child
  // returns SUCCESS, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRepeatUntilNode, RunningUponRunning)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // Without the target value set on the blackboard, if the child
  // returns RUNNING, the root should return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
}

TEST_F(TestRepeatUntilNode, FailureUponFailure)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // Now matter what, if the child returns FAILURE, the root should return FAILURE
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  // Check the status of the nodes directly too
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRepeatUntilNode, HaltAfterRunning)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

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

TEST_F(TestRepeatUntilNode, ValueOnBlackboardSuccess)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // If the child returns SUCCESS...
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);

  // and the target value is on the blackboard...
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);
  blackboard_->set<bool>("target_key", true);

  auto status = root_->executeTick();

  // the parent should return SUCCESS
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRepeatUntilNode, ValueOnBlackboardFailure)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // If the child returns FAILURE...
  child_action_->set_return_value(BT::NodeStatus::FAILURE);

  // and the target value is on the blackboard...
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);
  blackboard_->set<bool>("target_key", true);

  auto status = root_->executeTick();

  // the parent should propagate the FAILURE
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestRepeatUntilNode, ToggleTest)
{
  blackboard_->set("key", "target_key");
  blackboard_->set<bool>("value", true);

  // Start everything running
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Then set the flag on the blackboard
  std::string key_name;
  blackboard_->get<std::string>("key", key_name);
  blackboard_->set<bool>(key_name, true);

  // Which should cause the decorator to return SUCCESS. The
  // child node, the fake action, is still running
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Turn off the flag, tick again, and both should be RUNNING
  blackboard_->set<bool>(key_name, false);  // NOLINT
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Turn the flag back on and check once more
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
