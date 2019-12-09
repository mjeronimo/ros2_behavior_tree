/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "ros2_behavior_tree/action/pure_pursuit_node.hpp"

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

namespace ros2_behavior_tree
{

PurePursuitController::PurePursuitController(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput<std::shared_ptr<rclcpp::Node>>("node_handle", node_)) {
    throw BT::RuntimeError("Missing parameter [node_handle] in PurePursuitController node");
  }

  if (!getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_)) {
    throw BT::RuntimeError("Missing parameter [tf_buffer] in TransformPose node");
  }

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(path_topic_name_, queue_depth_, std::bind(
        &PurePursuitController::path_callback, this,
        _1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name_, queue_depth_, std::bind(
        &PurePursuitController::odometry_callback, this,
        _1));

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name_, queue_depth_);

  cmd_traj_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    cmd_traj_topic_name_, queue_depth_);
}

PurePursuitController::~PurePursuitController()
{
}

geometry_msgs::msg::PoseStamped
PurePursuitController::get_current_pose() const
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::PoseStamped transformed_pose;

  pose.header.frame_id = pose_frame_id_;
  double transform_timeout = 0.1;   // TODO

  try {
    // tf_listener_->transformPose(cur_ref_path_.header.frame_id, pose, transformed_pose);
    transformed_pose = tf_buffer_->transform(pose, cur_ref_path_.header.frame_id, tf2::durationFromSec(transform_timeout));
  } catch (tf2::TransformException & exception) {
    RCLCPP_ERROR(node_->get_logger(), "PurePursuitController::get_current_pose: %s",
      exception.what());
  }

  return transformed_pose;
}

void
PurePursuitController::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  cur_ref_path_ = *msg;
  next_waypoint_ = -1;
}

void
PurePursuitController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg->twist.twist;
}

BT::NodeStatus
PurePursuitController::tick()
{
  geometry_msgs::msg::Twist cmd_vel;

  if (step(cmd_vel)) {
    const size_t numPoints = 20;

    double lookAheadThreshold = get_lookahead_threshold();
    visualization_msgs::msg::Marker cmd_trajectory;

    cmd_trajectory.header.frame_id = pose_frame_id_;
    cmd_trajectory.header.stamp = node_->now();
    cmd_trajectory.ns = "solution_trajectory";
    cmd_trajectory.type = 4;
    cmd_trajectory.action = 0;
    cmd_trajectory.scale.x = 0.12;
    cmd_trajectory.color.r = 0.0;
    cmd_trajectory.color.g = 0.0;
    cmd_trajectory.color.b = 1.0;
    cmd_trajectory.color.a = 1.0;
    cmd_trajectory.lifetime = rclcpp::Duration(0);
    cmd_trajectory.frame_locked = true;
    cmd_trajectory.pose = geometry_msgs::msg::Pose();
    cmd_trajectory.points.resize(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
      geometry_msgs::msg::Pose pose;
      double dt = lookAheadThreshold * static_cast<double>(i) / static_cast<double>(numPoints);

      pose.orientation.z = cmd_vel.angular.x * dt;
      pose.position.x = cmd_vel.linear.x * std::cos(pose.orientation.z) * dt;
      pose.position.y = cmd_vel.linear.x * std::sin(pose.orientation.z) * dt;

      cmd_trajectory.points[i] = pose.position;
    }

    cmd_traj_pub_->publish(cmd_trajectory);
    cmd_vel_pub_->publish(cmd_vel);

    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

bool
PurePursuitController::step(geometry_msgs::msg::Twist & twist)
{
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  next_waypoint_ = get_next_waypoint(next_waypoint_);

  if (next_waypoint_ >= 0) {
    geometry_msgs::msg::PoseStamped pose;

    if (get_interpolated_pose(next_waypoint_, pose)) {
      double lookAheadDistance = get_lookahead_distance(pose);
      double lookAheadAngle = get_lookahead_angle(pose);
      double angularVelocity = 0.0;

      if (std::abs(std::sin(lookAheadAngle)) >= epsilon_) {
        double radius = 0.5 * (lookAheadDistance / std::sin(lookAheadAngle));
        double linearVelocity = velocity_;

        if (std::abs(radius) >= epsilon_) {
          angularVelocity = linearVelocity / radius;
        }

        twist.linear.x = linearVelocity;
        twist.angular.z = angularVelocity;

        return true;
      }
    }
  }

  return false;
}

double
PurePursuitController::get_lookahead_distance(const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped origin = get_current_pose();
  geometry_msgs::msg::PoseStamped transformed_pose;
  double transform_timeout = 0.1; // TODO:

  try {
    transformed_pose = tf_buffer_->transform(pose, cur_ref_path_.header.frame_id, tf2::durationFromSec(transform_timeout));
  } catch (tf2::TransformException & exception) {
    RCLCPP_ERROR(node_->get_logger(),
      "PurePursuitController::get_lookahead_distance: %s", exception.what());
    return -1.0;
  }

  tf2::Vector3 v1(origin.pose.position.x,
    origin.pose.position.y,
    origin.pose.position.z);

  tf2::Vector3 v2(transformed_pose.pose.position.x,
    transformed_pose.pose.position.y,
    transformed_pose.pose.position.z);

  return tf2::tf2Distance(v1, v2);
}

double
PurePursuitController::get_lookahead_angle(const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped origin = get_current_pose();
  geometry_msgs::msg::PoseStamped transformed_pose;

  double transform_timeout = 0.1; // TODO

  try {
    transformed_pose = tf_buffer_->transform(pose, cur_ref_path_.header.frame_id, tf2::durationFromSec(transform_timeout));
  } catch (tf2::TransformException & exception) {
    RCLCPP_ERROR(node_->get_logger(),
      "PurePursuitController::get_lookahead_angle: %s", exception.what());
    return -1.0;
  }

  tf2::Vector3 v1(origin.pose.position.x,
    origin.pose.position.y,
    origin.pose.position.z);

  tf2::Vector3 v2(transformed_pose.pose.position.x,
    transformed_pose.pose.position.y,
    transformed_pose.pose.position.z);

  return tf2::tf2Angle(v1, v2);
}

double
PurePursuitController::get_lookahead_threshold() const
{
  return lookahead_ratio_ * current_velocity_.linear.x;
}

double
PurePursuitController::get_arc_distance(const geometry_msgs::msg::PoseStamped & pose) const
{
  double lookAheadDistance = get_lookahead_distance(pose);
  double lookAheadAngle = get_lookahead_angle(pose);

  if (std::abs(std::sin(lookAheadAngle)) >= epsilon_) {
    return lookAheadDistance / sin(lookAheadAngle) * lookAheadAngle;
  } else {
    return lookAheadDistance;
  }
}

int
PurePursuitController::get_next_waypoint(int /*wayPoint*/) const
{
  if (!cur_ref_path_.poses.empty()) {
    if (next_waypoint_ >= 0) {
      geometry_msgs::msg::PoseStamped origin = get_current_pose();
      tf2::Vector3 v_1(origin.pose.position.x,
        origin.pose.position.y,
        origin.pose.position.z);
      double lookAheadThreshold = get_lookahead_threshold();

      for (size_t i = next_waypoint_; i < cur_ref_path_.poses.size(); ++i) {
        tf2::Vector3 v_2(cur_ref_path_.poses[i].pose.position.x,
          cur_ref_path_.poses[i].pose.position.y,
          cur_ref_path_.poses[i].pose.position.z);

        if (tf2::tf2Distance(v_1, v_2) > lookAheadThreshold) {
          return i;
        }
      }

      return next_waypoint_;
    } else {
      return 0;
    }
  }

  return -1;
}

int
PurePursuitController::get_closest_waypoint() const
{
  if (!cur_ref_path_.poses.empty()) {
    int closestWaypoint = -1;
    double minDistance = -1.0;

    for (size_t i = 0; i < cur_ref_path_.poses.size(); ++i) {
      double distance = get_arc_distance(cur_ref_path_.poses[i]);

      if ((minDistance < 0.0) || (distance < minDistance)) {
        closestWaypoint = i;
        minDistance = distance;
      }
    }

    return closestWaypoint;
  }

  return -1;
}

bool
PurePursuitController::get_interpolated_pose(
  int wayPoint, geometry_msgs::msg::PoseStamped & interpolatedPose) const
{
  if (!cur_ref_path_.poses.empty()) {
    if (wayPoint > 0) {
      double l_t = get_lookahead_threshold();
      double p_t = get_lookahead_distance(
        cur_ref_path_.poses[next_waypoint_ - 1]);

      if (p_t < l_t) {
        geometry_msgs::msg::PoseStamped p_0 = get_current_pose();
        geometry_msgs::msg::PoseStamped p_1 = cur_ref_path_.poses[wayPoint - 1];
        geometry_msgs::msg::PoseStamped p_2 = cur_ref_path_.poses[wayPoint];

        tf2::Vector3 v_1(p_2.pose.position.x - p_0.pose.position.x,
          p_2.pose.position.y - p_0.pose.position.y,
          p_2.pose.position.z - p_0.pose.position.z);
        tf2::Vector3 v_2(p_1.pose.position.x - p_0.pose.position.x,
          p_1.pose.position.y - p_0.pose.position.y,
          p_1.pose.position.z - p_0.pose.position.z);
        tf2::Vector3 v_0(p_2.pose.position.x - p_1.pose.position.x,
          p_2.pose.position.y - p_1.pose.position.y,
          p_2.pose.position.z - p_1.pose.position.z);

        // double l_0 = v_0.length();
        // double l_1 = v_1.length();
        double l_2 = v_2.length();

        v_0.normalize();
        v_2.normalize();

        double alpha_1 = M_PI - tf2::tf2Angle(v_0, v_2);
        double beta_2 = asin(l_2 * sin(alpha_1) / l_t);
        double beta_0 = M_PI - alpha_1 - beta_2;
        double l_s = l_2 * sin(beta_0) / sin(beta_2);

        tf2::Vector3 p_s(p_1.pose.position.x + v_0[0] * l_s,
          p_1.pose.position.x + v_0[1] * l_s,
          p_1.pose.position.x + v_0[2] * l_s);

        interpolatedPose.pose.position.x = p_s[0];
        interpolatedPose.pose.position.y = p_s[1];
        interpolatedPose.pose.position.z = p_s[2];

        return true;
      }
    }

    interpolatedPose = cur_ref_path_.poses[wayPoint];
    return true;
  }

  return false;
}

}  // namespace ros2_behavior_tree
