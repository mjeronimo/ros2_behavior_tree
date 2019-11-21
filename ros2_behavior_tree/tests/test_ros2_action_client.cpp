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
#include <string>

#include "fibonacci_client.hpp"
// #include "fibonacci_server.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_service_client_node.hpp"
#include "rclcpp/rclcpp.hpp"

struct TestROS2ActionClientNode : testing::Test
{
#if 0
  static void SetUpTestCase()
  {
    // Create a server that we can use to test our service client
    service_node_ = std::make_shared<FibonacciServer>("fibonaaci_server");

    // Spin this server node on a separate thread
    service_node_thread_ = std::make_unique<ros2_behavior_tree::NodeThread>(service_node_);
  }

  static void TearDownTestCase()
  {
    // The server's spin loop running on the separate thread will break upon rclcpp::shutdown
    rclcpp::shutdown();

    // We can then stop the thread and delete the service node
    service_node_thread_.reset();
    service_node_.reset();
  }
#endif

  void SetUp()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Set the blackboard to use in the node configuration
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;

    // Set this configuration to the Fibonacci input and output ports
    BT::assignDefaultRemapping<FibonacciClient>(config);

    client_node_ = std::make_shared<rclcpp::Node>("client_node1");
    fibonacci_client_ = std::make_unique<FibonacciClient>("fibonacci", config);
  }

  void TearDown()
  {
  }

  // static std::shared_ptr<FibonacciServer> service_node_;
  // static std::shared_ptr<ros2_behavior_tree::NodeThread> service_node_thread_;

  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<FibonacciClient> fibonacci_client_;

  std::shared_ptr<rclcpp::Node> client_node_;
};

// std::shared_ptr<FibonacciServer> TestROS2ActionClientNode::service_node_;
// std::shared_ptr<ros2_behavior_tree::NodeThread> TestROS2ActionClientNode::service_node_thread_;

// Set a couple values on the blackboard, which will be picked up by the BT node's
// input ports and tick the node, which will cause it to execute the action call
TEST_F(TestROS2ActionClientNode, SimpleCall)
{
  blackboard_->set("action_name", "fibonacci");
  blackboard_->set("wait_timeout", "1000");
  blackboard_->set("call_timeout", "1000");
  blackboard_->set<std::shared_ptr<rclcpp::Node>>("client_node", client_node_);  // NOLINT
  blackboard_->set("n", "10");

  // Manually run the node to completion
  while (fibonacci_client_->executeTick() == BT::NodeStatus::RUNNING)
    ;

  int sequence = 0;
  auto rc = blackboard_->get("sequence", sequence);
  ASSERT_EQ(rc, true);
  ASSERT_EQ(sequence, 101);
}

TEST_F(TestROS2ActionClientNode, CallUsingXML)
{
  static const char * xml_text =
    R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CreateROS2Node node_name="client_node2" spin="false" node_handle="{client_node}"/>
            <Fibonacci action_name="fibonacci" wait_timeout="1000" call_timeout="1000" client_node="{client_node}" n="10" sequence="{sequence}"/>
            <Fibonacci action_name="fibonacci" wait_timeout="1000" call_timeout="1000" client_node="{client_node}" n="11" sequence="{sequence2}"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

  // Have to load the cutom test nodes to make Fibonacci available to the XML
  ros2_behavior_tree::BehaviorTree bt(xml_text, {"ros2_behavior_tree_nodes", "custom_test_nodes"});

  // Execute the Behavior Tree and make sure that was successful
  auto bt_result = bt.execute();
  ASSERT_EQ(bt_result, ros2_behavior_tree::BtStatus::SUCCEEDED);

  // Check all of the output values from the blackboard (output ports)

  int sequence = 0;
  auto rc = bt.blackboard()->get("sequence", sequence);
  ASSERT_EQ(rc, true);
  ASSERT_EQ(sequence, 101);

  printf("sequence: %d\n", sequence);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
