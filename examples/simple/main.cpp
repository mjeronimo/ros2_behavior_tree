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

#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/behavior_tree_engine.hpp"

static const std::string xml_text = R"(

<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <Message msg="Hello, World!"/>
      <Message msg="Hello, World!"/>
      <Message msg="Hello, World!"/>
      <RateController hz="1.0">
        <Message msg="Hello, World!"/>
      </RateController>
    </Sequence>
  </BehaviorTree>
</root>

)";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  ros2_behavior_tree::BehaviorTreeEngine bt_engine({"ros2_behavior_tree_nodes"});

  // Create the blackboard that will be shared by all of the nodes in the tree
  auto blackboard = BT::Blackboard::create();

  // Put items on the blackboard
  //blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  //blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT

  BT::Tree tree = bt_engine.buildTreeFromText(xml_text, blackboard);

  auto is_canceling = []() {return false;};
  auto on_loop = []() {};

  auto rc = bt_engine.run(&tree, on_loop, is_canceling);

  switch (rc) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      break;

    case ros2_behavior_tree::BtStatus::CANCELED:
      break;

    default:
      break; // throw std::logic_error("Invalid status return from BT");
  }

  rclcpp::shutdown();

  return 0;
}
