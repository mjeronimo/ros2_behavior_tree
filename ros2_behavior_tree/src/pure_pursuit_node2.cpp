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

#include "ros2_behavior_tree/action/pure_pursuit_node2.hpp"

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace ros2_behavior_tree
{

PurePursuitNode2::PurePursuitNode2(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
#if 0
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(path_topic_name_, queue_depth_, 
	  std::bind(&PurePursuitNode2::path_callback, this, _1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name_, queue_depth_, 
	  std::bind(&PurePursuitNode2::odometry_callback, this, _1));

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name_, queue_depth_);

  /////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Subscriber waypoint_sub =
      nh.subscribe("final_waypoints", 1, &PurePursuitController::callback_from_waypoints, &pp);

  ros::Subscriber ndt_sub =
      nh.subscribe("current_pose", 1, &PurePursuitController::callback_from_current_pose, &pp);

  ros::Subscriber twist_sub =
      nh.subscribe("current_velocity", 1, &PurePursuitController::callback_from_current_velocity, &pp);


  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1);
#endif
}

PurePursuitNode2::~PurePursuitNode2()
{
}

BT::NodeStatus
PurePursuitNode2::tick()
{
  if (node_ == nullptr) {
    if (!getInput<std::shared_ptr<rclcpp::Node>>("node_handle", node_)) {
      throw BT::RuntimeError("Missing parameter [node_handle] in PurePursuitNode2 node");
    }

    if (!getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_)) {
      throw BT::RuntimeError("Missing parameter [tf_buffer] in TransformPose node");
    }

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name_, 1);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace 
