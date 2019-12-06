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
#include <vector>

#include "fibonacci_client.hpp"
#include "fibonacci_server.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_service_client_node.hpp"
#include "rclcpp/rclcpp.hpp"

struct TestROS2ActionClientNode : testing::Test
{
  static void SetUpTestCase()
  {
    // Create a server that we can use to test our service client
    action_node_ = std::make_shared<FibonacciServer>("fibonaaci_server");

    // Spin this server node on a separate thread
    action_node_thread_ = std::make_unique<ros2_behavior_tree::NodeThread>(action_node_);
  }

  static void TearDownTestCase()
  {
    // The server's spin loop running on the separate thread will break upon rclcpp::shutdown
    rclcpp::shutdown();

    // We can then stop the thread and delete the service node
    action_node_thread_.reset();
    action_node_.reset();
  }

  void SetUp()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Set the blackboard to use in the node configuration
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;

    // Set this configuration to the Fibonacci input and output ports
    BT::assignDefaultRemapping<FibonacciClient>(config);

    ros2_node_ = std::make_shared<rclcpp::Node>("ros2_node1");
    ros2_node_thread_ = std::make_unique<ros2_behavior_tree::NodeThread>(ros2_node_);

    fibonacci_client_ = std::make_unique<FibonacciClient>("fibonacci", config);
  }

  void TearDown()
  {
    ros2_node_thread_.reset();
    ros2_node_.reset();
  }

  static std::shared_ptr<FibonacciServer> action_node_;
  static std::shared_ptr<ros2_behavior_tree::NodeThread> action_node_thread_;

  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<FibonacciClient> fibonacci_client_;

  std::shared_ptr<rclcpp::Node> ros2_node_;
  std::shared_ptr<ros2_behavior_tree::NodeThread> ros2_node_thread_;
};

std::shared_ptr<FibonacciServer> TestROS2ActionClientNode::action_node_;
std::shared_ptr<ros2_behavior_tree::NodeThread> TestROS2ActionClientNode::action_node_thread_;

// Set a couple values on the blackboard, which will be picked up by the BT node's
// input ports and tick the node, which will cause it to execute the action call
TEST_F(TestROS2ActionClientNode, SimpleCall)
{
  blackboard_->set("action_name", "fibonacci");
  blackboard_->set("server_timeout", "1000");
  blackboard_->set<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_);  // NOLINT
  blackboard_->set("n", "10");

  // Manually run the node to completion
  while (fibonacci_client_->executeTick() == BT::NodeStatus::RUNNING) {
  }

  // Check the resulting value from the 'sequence' output port
  std::vector<int32_t> sequence;
  auto rc = blackboard_->get("sequence", sequence);

  std::vector<int32_t> expected_result{0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55};

  ASSERT_EQ(rc, true);
  ASSERT_EQ(sequence, expected_result);
}

TEST_F(TestROS2ActionClientNode, CallUsingXML)
{
  static const char * xml_text =
    R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CreateROS2Node node_name="ros2_node2" namespace="" spin="true" node_handle="{ros2_node}"/>
            <Fibonacci action_name="fibonacci" server_timeout="1000" ros2_node="{ros2_node}" n="10" sequence="{sequence}"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

  // Load the custom test nodes to make the Fibonacci node available to the XML
  ros2_behavior_tree::BehaviorTree bt(xml_text, {"ros2_behavior_tree_nodes", "custom_test_nodes"});

  // Execute the Behavior Tree and make sure that was successful
  auto bt_result = bt.execute();
  ASSERT_EQ(bt_result, ros2_behavior_tree::BtStatus::SUCCEEDED);

  // Check the resulting value from the 'sequence' output port

  std::vector<int32_t> sequence;
  auto rc = bt.blackboard()->get("sequence", sequence);

  std::vector<int32_t> expected_result{0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55};

  ASSERT_EQ(rc, true);
  ASSERT_EQ(sequence, expected_result);
}

TEST_F(TestROS2ActionClientNode, SendNewGoal)
{
  blackboard_->set("action_name", "fibonacci");
  blackboard_->set("server_timeout", "1000");
  blackboard_->set<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_);  // NOLINT
  blackboard_->set("n", "10");

  // This time, dynamically update the goal on the blackboard. This should cause the
  // Fibonacci node to cancel the currently running goal and execute the new goal
  while (fibonacci_client_->executeTick() == BT::NodeStatus::RUNNING) {
    blackboard_->set("n", "4");
  }

  // Check the resulting value from the 'sequence' output port

  std::vector<int32_t> sequence;
  auto rc = blackboard_->get("sequence", sequence);

  // Since we updated the goal, we should get the sequence for fib(4)
  std::vector<int32_t> expected_result{0, 1, 1, 2, 3};

  ASSERT_EQ(rc, true);
  ASSERT_EQ(sequence, expected_result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
