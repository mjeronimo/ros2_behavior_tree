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
#include "ros2_behavior_tree/async_wait_node.hpp"

struct AsyncWaitWithStubAction : testing::Test
{
  AsyncWaitWithStubAction()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Create a node configurand and populate the blackboard
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    blackboard_->set("msec", "0");

    BT::assignDefaultRemapping<ros2_behavior_tree::AsyncWaitNode>(config);
    async_wait_node_ = std::make_unique<ros2_behavior_tree::AsyncWaitNode>("async_wait", config);
  }

  ~AsyncWaitWithStubAction()
  {
    BT::haltAllActions(async_wait_node_.get());
  }

  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<ros2_behavior_tree::AsyncWaitNode> async_wait_node_;
};

// If the wait duration hasn't been exceeded, the node should return RUNNING
TEST_F(AsyncWaitWithStubAction, BeforeExpiry)
{
  blackboard_->set("msec", "100");

  auto status = async_wait_node_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(async_wait_node_->status(), BT::NodeStatus::RUNNING);
}

// If the wait duration has been exceeded, the node should return SUCCESS
TEST_F(AsyncWaitWithStubAction, AfterExpiry)
{
  blackboard_->set("msec", "100");

  // The first tick will cause the AsyncWait node to launch an async thread
  // and return RUNNING
  auto status = async_wait_node_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(async_wait_node_->status(), BT::NodeStatus::RUNNING);

  // Wait for the wait duraction
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  // Upon the next tick, the time will have been exceeded
  status = async_wait_node_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(async_wait_node_->status(), BT::NodeStatus::SUCCESS);
}

// Halting the AsyncWaitNode should cause it to go IDLE
TEST_F(AsyncWaitWithStubAction, IdleUponHalt)
{
  blackboard_->set("msec", "100");

  auto status = async_wait_node_->executeTick();
  ASSERT_EQ(status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(async_wait_node_->status(), BT::NodeStatus::RUNNING);
  async_wait_node_->halt();
  ASSERT_EQ(async_wait_node_->status(), BT::NodeStatus::IDLE);
}
