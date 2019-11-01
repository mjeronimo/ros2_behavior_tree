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

#include "bt_executor.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>

#include "ros2_behavior_tree/bt_conversions.hpp"

namespace ros2_behavior_tree
{

BtExecutor::BtExecutor()
: rclcpp_lifecycle::LifecycleNode("bt_executor")
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("bt_xml_filename", rclcpp::ParameterValue(std::string("bt_executor.xml")));
}

BtExecutor::~BtExecutor()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

CallbackReturn
BtExecutor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();

  auto options = rclcpp::NodeOptions().arguments({std::string("__node:=") +
        get_name() + "_client_node"});

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  using namespace std::placeholders;

  // Create an action server that we implement with our executeBehaviorTree method
  // action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "ExecuteBehaviorTree",
  // std::bind(&BtExecutor::executeBehaviorTree, this), false);
  action_server_ = rclcpp_action::create_server<ActionServer>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "execute_behavior_tree",
    std::bind(&BtExecutor::handle_goal, this, _1, _2),
    std::bind(&BtExecutor::handle_cancel, this, _1),
    std::bind(&BtExecutor::handle_accepted, this, _1)
  );

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<BehaviorTreeEngine>();

  // Put items on the blackboard
  // blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  // blackboard_->set<std::chrono::milliseconds>("node_loop_timeout", std::chrono::milliseconds(10));  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter("bt_xml_filename", bt_xml_filename);

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  // Create the Behavior Tree from the XML input (after registering our own node types)
  BT::Tree temp_tree = bt_->buildTreeFromText(xml_string_);

  // Unfortunately, the BT library provides the tree as a struct instead of a pointer. So, we will
  // createa new BT::Tree ourselves and move the data over
  tree_ = std::make_unique<BT::Tree>();
  tree_->root_node = temp_tree.root_node;
  tree_->nodes = std::move(temp_tree.nodes);
  temp_tree.root_node = nullptr;

  return CallbackReturn::SUCCESS;
}

CallbackReturn
BtExecutor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // action_server_->activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
BtExecutor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // action_server_->deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
BtExecutor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  client_node_.reset();
  // action_server_.reset();
  xml_string_.clear();
  tree_.reset();
  bt_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
BtExecutor::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
BtExecutor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

void
BtExecutor::executeBehaviorTree()
{
  // auto is_canceling = [this]() {return action_server_->is_cancel_requested();};
  auto is_canceling = []() {return false;};

  // auto on_loop = [this]() {
  // if (action_server_->preempt_requested()) {
  // RCLCPP_INFO(get_logger(), "Received goal preemption request");
  // action_server_->accept_pending_goal();
  // }
  // };

  auto on_loop = []() {};

  // Execute the BT that was previously created in the configure step
  ros2_behavior_tree::BtStatus rc = bt_->run(tree_, on_loop, is_canceling);

  switch (rc) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      // action_server_->succeeded_current();
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      // action_server_->abort_all();
      break;

    case ros2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      // action_server_->cancel_all();
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

}  // namespace ros2_behavior_tree
