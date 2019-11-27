// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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

#ifndef ROS2_BEHAVIOR_TREE__CREATE_TRANSFORM_BUFFER_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CREATE_TRANSFORM_BUFFER_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace ros2_behavior_tree
{

class CreateTransformBufferNode : public BT::SyncActionNode
{
public:
  CreateTransformBufferNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "The ROS2 node to use"),
      BT::OutputPort<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", "The created transform buffer")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<rclcpp::Node> node;

    if (!getInput<std::shared_ptr<rclcpp::Node>>("node_handle", node)) {
      throw BT::RuntimeError("Missing parameter [node_handle] in CreateTransformBuffer node");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node->get_node_base_interface(),
        node->get_node_timers_interface());

    tf_buffer_->setCreateTimerInterface(timer_interface);
    //tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // , node, false);

    if (!setOutput<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", tf_buffer_)) {
      throw BT::RuntimeError("Failed to set output port value [transform_buffer] in CreateTransformBuffer");
    }

    return BT::NodeStatus::SUCCESS;
  }

protected:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CREATE_TRANSFORM_BUFFER_NODE_HPP_
