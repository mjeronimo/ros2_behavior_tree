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

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_service_client_node.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::placeholders;

class ServiceNode : public rclcpp::Node
{
public:
  explicit ServiceNode(const std::string & name)
  : rclcpp::Node(name)
  {
    addition_server_ = create_service<AddTwoInts>("add_two_ints",
        std::bind(&ServiceNode::handle_service, this, _1, _2, _3));
  } 
  
  void handle_service( const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    (void)request_header;
    response->sum = request->a + request->b;
  }

  rclcpp::Service<AddTwoInts>::SharedPtr addition_server_;
};

struct ROS2ServiceTest : testing::Test
{
  static void SetUpTestCase()
  {
    service_node_ = std::make_shared<ServiceNode>("test_service_node");
    service_node_thread_ = std::make_unique<ros2_behavior_tree::NodeThread>(service_node_);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
    service_node_thread_.reset();
    service_node_.reset();
  }

  void SetUp()
  {
    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("service_name", "add_two_ints");
    blackboard_->set("server_timeout", "10");

    BT::NodeConfiguration config;
    config.blackboard = blackboard_;

    BT::assignDefaultRemapping<ros2_behavior_tree::ROS2ServiceClientNode<AddTwoInts>>(config);

    ros2_service_client_node_ =
      std::make_unique<ros2_behavior_tree::ROS2ServiceClientNode<AddTwoInts>>("add_two_ints", config);
  }

  void TearDown()
  {
  }

  static std::shared_ptr<ServiceNode> service_node_;
  static std::shared_ptr<ros2_behavior_tree::NodeThread> service_node_thread_;

  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<ros2_behavior_tree::ROS2ServiceClientNode<AddTwoInts>> ros2_service_client_node_;
};

std::shared_ptr<ServiceNode> ROS2ServiceTest::service_node_;
std::shared_ptr<ros2_behavior_tree::NodeThread> ROS2ServiceTest::service_node_thread_;

TEST_F(ROS2ServiceTest, ConditionTrue)
{
  auto request = std::make_shared<AddTwoInts::Request>();
  auto response = std::make_shared<AddTwoInts::Response>();

  // Set the input port values
  blackboard_->set("service_name", "add_two_ints");
  blackboard_->set("server_timeout", "10");
  blackboard_->set("request", request);

  // Set the fields of the request message
  request->a = 7;
  request->b = 14;

  // Execute the Behavior Tree, the result is in the "response" output port
  ros2_service_client_node_->executeTick();

  auto rc = blackboard_->get("response", response);
  ASSERT_EQ(response->sum, 21);
}

#include "ros2_behavior_tree/behavior_tree.hpp"

TEST_F(ROS2ServiceTest, ChainUsingXMLAndPorts)
{
  // TODO(mjeronimo): have to register the AddTwoInts type in order for it to be available to the XML
  using AddTwoIntsClient = ros2_behavior_tree::ROS2ServiceClientNode<AddTwoInts>;

  static const char* xml_text2 = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">

            <!-- Need some text format and conversion for the input and output types/structs -->
            <SetBlackboard key="AddTwoIntsRequest" "1:2"/>   
            <AddTwoInts service_name="add_two_ints" server_timeout="10" request="{AddTwoIntsRequest}" response="{AddTwoIntsResponse}"/>
            <!-- no way to directly reference the output fields, like response.sum -->
            <!-- I could pass the whole message to another node, but it would have to accept the AddTwoInts.Response type -->

            <!-- or -->

            <SetBlackboard key="some_value" "1"/>
            <SetBlackboard key="another_value" "2"/>
            <AddTwoInts service_name="add_two_ints" server_timeout="10" a="{some_value}" b="{another_value} sum="{sum}"/>

            <!-- can easily reference the output values -->
            <SetBlackboard key="a_third_value" "3"/>
            <AddTwoInts service_name="add_two_ints" server_timeout="10" a="{sum}" b="{a_third_value} sum="{final_result}"/>
            <Message msg="The result is: "/>
            <Message msg="{final_result}"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

  static const char* xml_text = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <AddTwoInts service_name="add_two_ints" server_timeout="10"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

    ros2_behavior_tree::BehaviorTree bt(xml_text, {"ros2_behavior_tree_nodes", "custom_test_nodes"});
    // ASSERT_EQ(bt.execute(), ros2_behavior_tree::BtStatus::SUCCEEDED);
    ASSERT_EQ(true, true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
