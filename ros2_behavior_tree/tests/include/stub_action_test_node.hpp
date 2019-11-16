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

#ifndef STUB_ACTION_TEST_NODE_HPP_
#define STUB_ACTION_TEST_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

class StubActionTestNode : public BT::ActionNodeBase
{
public:
  explicit StubActionTestNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
  }

  // Define this node's ports
  static BT::PortsList providedPorts() {return {};}

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
    return return_value_;
  }

  void halt() override
  {
    return_value_ = BT::NodeStatus::IDLE;
  }

private:
  BT::NodeStatus return_value_{BT::NodeStatus::SUCCESS};
  int tick_count_{0};
};

#endif  // STUB_ACTION_TEST_NODE_HPP_
