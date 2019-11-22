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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros2_behavior_tree/throttle_tick_count_node.hpp"
#include "stub_action_test_node.hpp"

struct TestThrottleTickCountNode : testing::Test
{
  TestThrottleTickCountNode()
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

  ~TestThrottleTickCountNode()
  {
    BT::haltAllActions(root_.get());
  }

  std::unique_ptr<ros2_behavior_tree::ThrottleTickCountNode> root_;
  std::unique_ptr<StubActionTestNode> child_action_;

  BT::Blackboard::Ptr blackboard_;
};

TEST_F(TestThrottleTickCountNode, FirstTimeSuccess)
{
  // If the child returns SUCCESS on the first tick, the parent should
  // also return SUCCESS
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, FailureUponFailure)
{
  // If the child returns FAILURE on the first tick, the parent should
  // also return FAILURE
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, WaitForDurationWithRunning)
{
  // If the child returns RUNNING on the first tick, the parent should
  // also return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // As long as the child is running, the parent should continue to
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);
}

TEST_F(TestThrottleTickCountNode, WaitForDurationWithSuccess)
{
  // If the child returns RUNNING on the first tick, the root should
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Now, if the child returns SUCCESS, the parent will return SUCCESS
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);

  // Upon a SUCCESS (or FAILURE) return from a child, decorators
  // automatically set the child to IDLE
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, WaitForDurationWithFailure)
{
  // If the child returns RUNNING on the first tick, the root should
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  child_action_->set_return_value(BT::NodeStatus::RUNNING);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Once the child returns FAILURE, that should be passed up
  child_action_->set_return_value(BT::NodeStatus::FAILURE);

  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, SuccessAfterRunning)
{
  // If the child returns RUNNING on the first tick, the root should
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // If we immediately tick again without waiting for the time period
  // to expire, the child should be ticked, and return SUCCESS in 
  // this case
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, FailureAfterRunning)
{
  // If the child returns RUNNING on the first tick, the root should
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // If we immediately tick again without waiting for the time period
  // to expire, the child should be ticked, and return FAILURE in 
  // this case
  child_action_->set_return_value(BT::NodeStatus::FAILURE);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  ASSERT_EQ(root_->status(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

TEST_F(TestThrottleTickCountNode, WaitForDurationWithSuccessThenSuccess)
{
  // If the child returns RUNNING on the first tick, the root should
  // return RUNNING
  child_action_->set_return_value(BT::NodeStatus::RUNNING);
  auto status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::RUNNING);

  // Wait a bit to exceed the time period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Now, if the child now returns SUCCESS, the ThrottleTickCount
  // decorator will return SUCCESS
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_->status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);

  // Now, if we immediately tick again, the node should return
  // RUNNING because the time hasn't expired
  child_action_->set_return_value(BT::NodeStatus::SUCCESS);
  status = root_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(root_->status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(child_action_->status(), BT::NodeStatus::IDLE);
}

