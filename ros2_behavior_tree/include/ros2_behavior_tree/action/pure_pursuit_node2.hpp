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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE2_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE2_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/pure_pursuit_controller.hpp"
#include "tf2_ros/transform_listener.h"

namespace ros2_behavior_tree
{

class PurePursuitNode2 : public BT::SyncActionNode
{
public:
  PurePursuitNode2(const std::string & name, const BT::NodeConfiguration & config);
  virtual ~PurePursuitNode2();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "The ROS2 node to use"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "The transform buffer to use")
    };
  }

  BT::NodeStatus tick() override;

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<PurePursuitController> controller_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE2_HPP_
