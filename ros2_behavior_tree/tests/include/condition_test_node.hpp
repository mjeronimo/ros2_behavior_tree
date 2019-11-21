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

#ifndef CONDITION_TEST_NODE_HPP_
#define CONDITION_TEST_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

class ConditionTestNode : public BT::ConditionNode
{
public:
  explicit ConditionTestNode(const std::string & name)
  : BT::ConditionNode(name, {})
  {
  }

  void set_return_value(BT::NodeStatus return_value)
  {
    assert(return_value == BT::NodeStatus::SUCCESS || return_value == BT::NodeStatus::FAILURE);
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
    setStatus(return_value_);
    return return_value_;
  }

private:
  bool return_value_{BT::NodeStatus::SUCCESS};
  int tick_count_{0};
};

#endif  // CONDITION_TEST_NODE_HPP_
