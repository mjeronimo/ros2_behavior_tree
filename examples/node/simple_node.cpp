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

#include "simple_node.hpp"

#include <memory>

namespace ros2_behavior_tree
{

// The Behavior Tree to execute
static const char xml_text[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="say_hello">
      <SetCondition key="done" value="true"/>
      <WhileCondition key="done" value="false">
        <Message msg="Hello, World!"/>
      </WhileCondition>
    </Sequence>
  </BehaviorTree>
</root>
)";

SimpleNode::SimpleNode()
: Node("simple_node"), bt_engine_({"ros2_behavior_tree_nodes"})
{
  thread_ = std::make_unique<std::thread>(&SimpleNode::run, this);
}

SimpleNode::~SimpleNode()
{
  thread_->join();
}

BtStatus
SimpleNode::run()
{
  return bt_engine_.run(xml_text);
}

}  // namespace ros2_behavior_tree
