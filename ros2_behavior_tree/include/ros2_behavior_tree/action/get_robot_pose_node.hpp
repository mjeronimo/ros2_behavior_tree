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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__GET_ROBOT_POSE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__GET_ROBOT_POSE_NODE_HPP_

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

class GetRobotPoseNode : public BT::SyncActionNode
{
public:
  GetRobotPoseNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "The transform buffer to use"),
      BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose",
        "The current pose of the robot")
    };
  }

  BT::NodeStatus tick() override
  {
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    if (!getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer)) {
      throw BT::RuntimeError("Missing parameter [tf_buffer] in GetRobotPose node");
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (get_robot_pose(current_pose, tf_buffer)) {
      auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
      *pose = current_pose;
      if (!setOutput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", pose)) {
        throw BT::RuntimeError("Failed to set output port value [pose] for GetRobotPose");
      }
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

protected:
  bool get_robot_pose(
    geometry_msgs::msg::PoseStamped & target_pose,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string source_frame = "base_link",
    const std::string target_frame = "map",
    const double transform_timeout = 0.1)
  {
    /*static*/ rclcpp::Logger logger = rclcpp::get_logger("get_robot_pose");

    // Initialize the robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = source_frame;
    robot_pose.header.stamp = rclcpp::Time();

    try {
      // Get the pose in the target frame
      tf2::toMsg(tf2::Transform::getIdentity(), target_pose.pose);
      target_pose = tf_buffer->transform(robot_pose, target_frame,
          tf2::durationFromSec(transform_timeout));

      RCLCPP_INFO(logger, "x,y: %f,%f", target_pose.pose.position.x, target_pose.pose.position.y);
      return true;
    } catch (tf2::LookupException & ex) {
      RCLCPP_WARN(logger,
        "Failed to get robot pose: %s\n", ex.what());
    } catch (tf2::ConnectivityException & ex) {
      RCLCPP_WARN(logger,
        "Failed to get robot pose: %s\n", ex.what());
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_WARN(logger,
        "Failed to get robot pose: %s\n", ex.what());
    } catch (tf2::TimeoutException & ex) {
      RCLCPP_WARN(logger,
        "Transform timeout with tolerance: %.4f", transform_timeout);
    } catch (...) {
      RCLCPP_WARN(logger, "Failed to transform from %s to %s",
        source_frame.c_str(), target_frame.c_str());
    }

    return false;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__GET_ROBOT_POSE_NODE_HPP_
