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

#include <cinttypes>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "ros2_behavior_tree/ros2_service_node.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::placeholders;

class NodeThread
{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : node_(node_base)
  {
    thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_.add_node(node_);
        executor_.spin();
        executor_.remove_node(node_);
      });
  }

  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  ~NodeThread()
  {
    executor_.cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

class ServiceNode : public rclcpp::Node
{
public:
  explicit ServiceNode(const std::string & name)
  : rclcpp::Node(name)
  {
    addition_server_ = create_service<AddTwoInts>("add_two_ints",
        std::bind(&ServiceNode::handle_service, this, _1, _2, _3));

    // Create a blackboard which will be shared among the nodes
    blackboard_ = BT::Blackboard::create();

    // Create a node configurand and populate the blackboard
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    // TODO(mjeronimo): Put required items on the blackboard
    // blackboard_->set("msec", "0");
    // blackboard_->set("msec", "0");
    // blackboard_->set("msec", "0");
    // blackboard_->set("msec", "0");

    BT::assignDefaultRemapping<ros2_behavior_tree::ROS2ServiceNode<AddTwoInts>>(config);
    ros2_service_node_ =
      std::make_unique<ros2_behavior_tree::ROS2ServiceNode<AddTwoInts>>("ROS2 service", config);
  }

  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    (void)request_header;
    RCLCPP_INFO(get_logger(), "request: %" PRId64 " + %" PRId64, request->a, request->b);
    response->sum = request->a + request->b;
  }

  rclcpp::Service<AddTwoInts>::SharedPtr addition_server_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<ros2_behavior_tree::ROS2ServiceNode<AddTwoInts>> ros2_service_node_;
};

struct ROS2ServiceTest : testing::Test
{
  static void run_service_thread()
  {
    rclcpp::spin(service_node_->get_node_base_interface());
  }

  static void SetUpTestCase()
  {
    std::cerr << "SetUpTestCase\n";
    service_node_ = std::make_shared<ServiceNode>("service_node");
    service_node_thread_ = std::make_unique<std::thread>(&ROS2ServiceTest::run_service_thread);
  }

  static void TearDownTestCase()
  {
    std::cerr << "TearDownTestCase\n";
    service_node_.reset();
  }

  void SetUp()
  {
    std::cerr << "SetUp\n";
  }

  void TearDown()
  {
    std::cerr << "TearDown\n";
  }

  static std::shared_ptr<ServiceNode> service_node_;
  static std::shared_ptr<std::thread> service_node_thread_;
};

std::shared_ptr<ServiceNode> ROS2ServiceTest::service_node_;
std::shared_ptr<std::thread> ROS2ServiceTest::service_node_thread_;

TEST_F(ROS2ServiceTest, ConditionTrue)
{
  ASSERT_EQ(true, true);
}

TEST_F(ROS2ServiceTest, ConditionTrue2)
{
  ASSERT_EQ(true, true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  auto rc = RUN_ALL_TESTS();

  // TODO(mjeronimo): stop and join the thread before ROS shutdown
  rclcpp::shutdown();
  ROS2ServiceTest::service_node_thread_->join();

  return rc;
}
