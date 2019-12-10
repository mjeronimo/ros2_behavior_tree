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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace ros2_behavior_tree
{

class GetPosesNearRobotNode : public BT::SyncActionNode
{
public:
  GetPosesNearRobotNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle",
        "A node for publishing the markers for the poses"),
      BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose",
        "The current pose of the robot"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("nearby_poses",
        "Poses near the robot")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      if (!getInput<std::shared_ptr<rclcpp::Node>>("node_handle", node_)) {
        throw BT::RuntimeError("Missing parameter [node_handle] in GetPosesNearRobot node");
      }

      marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("follower_goal", 1);
      initialized_ = true;
    }

    std::shared_ptr<geometry_msgs::msg::PoseStamped> robot_pose;
    if (!getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("robot_pose", robot_pose)) {
      throw BT::RuntimeError("Missing parameter [robot_pose] in GetPosesNearRobot node");
    }

    if (!setOutput<std::vector<geometry_msgs::msg::PoseStamped>>("nearby_poses",
      getNearbyPoses(*robot_pose)))
    {
      throw BT::RuntimeError(
              "Failed to set output port value [nearby_poses] for GetPosesNearRobot");
    }

    return BT::NodeStatus::SUCCESS;
  }

  std::vector<geometry_msgs::msg::PoseStamped> getNearbyPoses(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    double min_distance = 0.5;
    double max_distance = 1.5;
    double step = 0.1;

    int num_steps = (max_distance - min_distance) / step;
    std::vector<double> distances;
    for (int s = 0; s <= num_steps; s++) {
      distances.push_back(min_distance + s * step);
    }

    // Considering poses behind, left and right
    auto diag = std::sqrt(0.5);
    std::vector<std::vector<double>> positions{{-1, 0}, {0, 1},{0, -1}, {-diag, diag}, {-diag, -diag}};

    std::vector<geometry_msgs::msg::PoseStamped> nearby_poses;
    for (const auto & d : distances) {
      for (const auto & p : positions) {
        nearby_poses.push_back(
          getOffsettedPoseAround(pose, p[0] * d, p[1] * d));
      }
    }
    publishPosesMarker(nearby_poses);

    return nearby_poses;
  }

  // Get a pose with offsets defined with respect to a
  // coordinate system aligned to the reference's orientation
  geometry_msgs::msg::PoseStamped getOffsettedPoseAround(
    const geometry_msgs::msg::PoseStamped & reference,
    double x_offset, double y_offset, double z_offset = 0.0, double angle_offset = 0.0)
  {
    // TODO(orduno) Perform tranformation using tf2

    // Apply the translational offset
    double x = reference.pose.position.x + x_offset;
    double y = reference.pose.position.y + y_offset;
    double z = reference.pose.position.z + z_offset;
    double angle = tf2::getYaw(reference.pose.orientation);

    // normalize
    angle = atan2(sin(angle), cos(angle));

    // Rotate to align with reference, first translate such that the reference is now the origin
    double x_t = x - reference.pose.position.x;
    double y_t = y - reference.pose.position.y;

    // second, rotate
    double x_tr = x_t * cos(angle) - y_t * sin(angle);
    double y_tr = x_t * sin(angle) + y_t * cos(angle);

    // third, undo the translation
    x = x_tr + reference.pose.position.x;
    y = y_tr + reference.pose.position.y;

    angle += angle_offset;

    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    ps.pose.orientation = tf2::toMsg(q);

    return ps;
  }

  void publishPosesMarker(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    visualization_msgs::msg::Marker marker;

    builtin_interfaces::msg::Time time;
    time.sec = 0;
    time.nanosec = 0;
    marker.header.stamp = time;
    marker.header.frame_id = "map";

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "follower_goal";
    // static int index;
    // marker.id = index++;
    marker.id = 1;

    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    // Set the marker action.
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 0 indicates the object should last forever
    marker.lifetime = rclcpp::Duration(0);

    marker.frame_locked = false;

    marker.points.resize(poses.size());

    // std_msgs::msg::ColorRGBA color;
    // color.r = 0.0;
    // color.g = 0.0;
    // color.b = 1.0;
    // color.a = 1.0;
    for (const auto & p : poses) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = p.pose.position.x;
      pose.position.y = p.pose.position.y;
      marker.points.push_back(pose.position);
      // marker.colors.push_back(color);

      // color.b = color.b >= 0.2 ? color.b - 0.2 : 1.0;
      // color.r = color.r <= 0.8 ? color.r + 0.2 : 0.0;

      // marker.pose.orientation = pose.orientation;
    }

    marker_pub_->publish(marker);
  }

protected:
  bool initialized_{false};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__GET_POSES_NEAR_ROBOT_NODE_HPP_
