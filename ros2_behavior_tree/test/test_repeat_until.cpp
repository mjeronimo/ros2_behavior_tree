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

#include "behaviortree_cpp/behavior_tree.h"
#include "ros2_behavior_tree/repeat_until_node.hpp"

struct RepeatUntilTest : testing::Test
{
  RepeatUntilTest() 
  : root_("repeat_until", "name", "value"),
    child_("child", std::bind(&RepeatUntilTest::simple_action, this, std::placeholders::_1), {})
  {
    root_.setChild(&child_);
  }

  ~RepeatUntilTest()
  {
    BT::haltAllActions(&root_);
  }

  BT::NodeStatus simple_action(BT::TreeNode & tree_node)
  {
    return BT::NodeStatus::SUCCESS;
  }

  ros2_behavior_tree::RepeatUntilNode root_;
  BT::SimpleActionNode child_;
};

// If a child returns SUCCESS, the root should return RUNNING
TEST_F(RepeatUntilTest, ConditionTrue)
{
  root_.executeTick();

  ASSERT_EQ(child_.status(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(root_.status(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(true, true);
}
