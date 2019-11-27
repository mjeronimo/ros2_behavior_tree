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

#ifndef ROS2_BEHAVIOR_TREE__GET_CURRENT_POSE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__GET_CURRENT_POSE_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

namespace ros2_behavior_tree
{

class GetCurrentPoseNode : public BT::SyncActionNode
{
public:
  GetCurrentPoseNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", "The transform buffer to use"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "The current pose of the robot")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    if (!getInput<std::shared_ptr<tf2_ros::Buffer>>("transform_buffer", tf_buffer)) {
      throw BT::RuntimeError("Missing parameter [transform_buffer] in GetCurrentPose node");
    }

    geometry_msgs::msg::PoseStamped current_pose = geometry_msgs::msg::PoseStamped();

    if (getCurrentPose(current_pose, tf_buffer)) {
      if (!setOutput<geometry_msgs::msg::PoseStamped>("pose", current_pose)) {
        throw BT::RuntimeError("Failed to set output port value [transform_buffer] in CreateTransformBuffer");
      }
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  bool getCurrentPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string global_frame = "map",
    const std::string robot_frame = "base_link", const double transform_timeout = 0.1)
  {
    /*static*/ rclcpp::Logger logger = rclcpp::get_logger("getCurrentPose");
    geometry_msgs::msg::PoseStamped robot_pose;

    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_frame;
    robot_pose.header.stamp = rclcpp::Time(0);  // Time(now()) versus Time(0)

    try {
      global_pose = tf_buffer->transform(robot_pose, global_frame,
          tf2::durationFromSec(transform_timeout));

      RCLCPP_INFO(logger, "x,y: %f,%f", global_pose.pose.position.x, global_pose.pose.position.y);
      return true;
    } catch (tf2::LookupException & ex) {
      RCLCPP_ERROR(logger,
        "No Transform available Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::ConnectivityException & ex) {
      RCLCPP_ERROR(logger,
        "Connectivity Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(logger,
        "Extrapolation Error looking up robot pose: %s\n", ex.what());
    } catch (tf2::TimeoutException & ex) {
      RCLCPP_ERROR(logger,
        "Transform timeout with tolerance: %.4f", transform_timeout);
    } catch (...) {
      RCLCPP_ERROR(logger, "Failed to transform from %s to %s",
        global_frame.c_str(), robot_frame.c_str());
    }

    return false;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__GET_CURRENT_POSE_NODE_HPP_
