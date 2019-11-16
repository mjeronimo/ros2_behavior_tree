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

#ifndef SYNC_ACTION_TEST_NODE_HPP_
#define SYNC_ACTION_TEST_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

class SyncActionTest : public BT::SyncActionNode
{
public:
  explicit SyncActionTest(const std::string & name)
  : SyncActionNode(name, {})
  {
  }

  void set_return_value(BT::NodeStatus return_value)
  {
    return_value_ = return_value;
  }

  int get_tick_count() const
  {
    return tick_count_;
  }

  void reset_tick_count()
  {
    tick_count_ = 0;
  }

  BT::NodeStatus tick() override
  {
    tick_count_++;
    return boolean_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  bool return_value_{BT::NodeStatus::SUCCESS};
  int tick_count_{0};
};

#endif  // SYNC_ACTION_TEST_NODE_HPP_
