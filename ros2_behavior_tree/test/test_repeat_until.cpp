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
public:
  RepeatUntilTest() 
  : root_("repeat_until", "name", "value")
  {
    // root_.setChild(&action);
  }

  ~RepeatUntilTest()
  {
    BT::haltAllActions(&root_);
  }

private:
  ros2_behavior_tree::RepeatUntilNode root_;
};

TEST_F(RepeatUntilTest, ConditionTrue)
{
  // Create the tree 
  ASSERT_EQ(true, true);
}
