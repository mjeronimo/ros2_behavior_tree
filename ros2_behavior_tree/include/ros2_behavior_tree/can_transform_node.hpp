// Copyright (c) 2018 Intel Corporation
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

#ifndef ROS2_BEHAVIOR_TREE__CAN_TRANSFORM_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CAN_TRANSFORM_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

namespace ros2_behavior_tree
{

class CanTransformNode : public BT::ConditionNode
{
public:
  CanTransformNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "The ROS2 node to use"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", "The transform buffer to use"),
      BT::InputPort<std::string>("source_frame", "The source frame for the transform"),
      BT::InputPort<std::string>("target_frame", "The target frame for the transform")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<rclcpp::Node> node;
    if (!getInput<std::shared_ptr<rclcpp::Node>>("node_handle", node)) {
      throw BT::RuntimeError("Missing parameter [node_handle] in CanTransform node");
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    if (!getInput<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", tf_buffer)) {
      throw BT::RuntimeError("Missing parameter [transform_buffer] in CanTransform node");
    }

    std::string source_frame;
    if (!getInput<std::string>("source_frame", source_frame)) {
      throw BT::RuntimeError("Missing parameter [source_frame] in CanTransform node");
    }

    std::string target_frame;
    if (!getInput<std::string>("target_frame", target_frame)) {
      throw BT::RuntimeError("Missing parameter [target_frame] in CanTransform node");
    }

    rclcpp::Time transform_time = node->now();
    std::string tf_error;
    double timeout = 0.0;

    if (tf_buffer->canTransform(target_frame, source_frame,
      tf2_ros::fromMsg(transform_time), tf2::durationFromSec(timeout), &tf_error))
    {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node->get_logger(), "Transform from %s to %s failed: %s",
        source_frame.c_str(), target_frame.c_str(), tf_error.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CAN_TRANSFORM_NODE_HPP_
