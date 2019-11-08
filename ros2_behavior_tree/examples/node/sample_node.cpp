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

#include "sample_node.hpp"

#include <memory>

namespace ros2_behavior_tree
{

// The Behavior Tree to execute. This Behavior Tree will endlessly print
// "Hello, World!" at 1 second intervals
const char SampleNode::bt_xml_[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Forever>
      <Sequence name="say_hello">
        <Message msg="Hello, World!"/>
        <Wait msec="1000"/>
      </Sequence>
    </Forever>
  </BehaviorTree>
</root>
)";

SampleNode::SampleNode()
: Node("sample_node"), bt_(bt_xml_)
{
  // Execute the Behavior Tree on a separate thread
  thread_ = std::make_unique<std::thread>(&SampleNode::print_message, this);
}

SampleNode::~SampleNode()
{
  thread_->join();
}

BtStatus
SampleNode::print_message()
{
  return bt_.execute();
}

}  // namespace ros2_behavior_tree
