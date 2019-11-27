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

#include <iostream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"

// The Behavior Tree to execute. This tree simple prints "Hello," and
// "World!" on separate lines and then terminates.
#if 0
static const char bt_xml[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="say_hello">
      <Message msg="Hello,"/>
      <Message msg="World!"/>
    </Sequence>
  </BehaviorTree>
</root>
)";
#else
static const char bt_xml[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="say_hello">
      <SetBlackboard output_key="first_path_available" value="0"/>
      <CreateROS2Node node_name="some_name" spin="true" node_handle="{ros_node}"/>
      <CreateTransformBuffer node_handle="{ros_node}" transform_buffer="{tf}"/>
      <Wait msec="3000"/>
      <Recovery num_retries="5">
        <GetRobotPose transform_buffer="{tf}" pose="{goal}"/>
        <Wait msec="1000"/>
	    </Recovery>
    </Sequence>
  </BehaviorTree>
</root>
)";
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a behavior tree using the input XML
  ros2_behavior_tree::BehaviorTree bt(bt_xml);

  // Execute the BT and determine the result
  switch (bt.execute()) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      std::cout << "BT failed\n";
      break;

    case ros2_behavior_tree::BtStatus::HALTED:
      std::cout << "BT was halted\n";
      break;

    default:
      throw std::logic_error("Invalid return value from the BT");
  }

  rclcpp::shutdown();
  return 0;
}
