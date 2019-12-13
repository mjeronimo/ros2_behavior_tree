/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "libwaypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_behavior_tree
{

class PurePursuitController
{
public:
  PurePursuitController(std::shared_ptr<rclcpp::Node> node, bool linear_interpolate_mode = true);
  PurePursuitController() = delete;

  ~PurePursuitController();

  geometry_msgs::msg::TwistStamped go();

private:
  void callback_from_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr & msg);
  void callback_from_current_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr & msg);
  void callback_from_waypoints(const ros2_behavior_tree_msgs::msg::Lane::SharedPtr & msg);

  geometry_msgs::msg::Point get_pose_of_next_waypoint() const;
  geometry_msgs::msg::Point get_pose_of_next_target() const;
  geometry_msgs::msg::Pose get_current_pose() const;
  double get_lookahead_distance() const;
  double get_cmd_velocity(int waypoint) const;
  void calc_lookahead_distance(int waypoint);
  double calc_curvature(geometry_msgs::msg::Point target) const;
  double calc_radius(geometry_msgs::msg::Point target) const; /* not used */
  bool interpolate_next_target(int next_waypoint, geometry_msgs::msg::Point *next_target) const;
  bool verify_following() const;
  geometry_msgs::msg::Twist calc_twist(double curvature, double cmd_velocity) const;
  void get_closest_waypoint();  // Find closest waypoint to current pose
  void get_next_waypoint();
  geometry_msgs::msg::TwistStamped output_zero() const;
  geometry_msgs::msg::TwistStamped output_twist(geometry_msgs::msg::Twist t) const;

  std::shared_ptr<rclcpp::Node> node_;

  // Incoming state
  bool waypoint_set_{false};
  bool pose_set_{false};
  bool velocity_set_{false};
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_velocity_;
  WayPoints current_waypoints_;

  const double RADIUS_MAX_;
  const double KAPPA_MIN_;
  bool linear_interpolate_;
  double const_lookahead_distance_{4.0};  // meters
  double initial_velocity_{5.0};     // km/h
  double lookahead_distance_calc_ratio_{2.0};
  double minimum_lookahead_distance_{6.0};  // The next waypoint must be outside of this threshold. Shorten for tight turn
  double displacement_threshold_{0.05};
  double relative_angle_threshold_{0.5};
  int num_of_next_waypoint_{-1};
  int closest_waypoint_idx_{0};  // Index of closest waypoint to current pose. Default to first waypoint for index lookup
  geometry_msgs::msg::Point position_of_next_target_;
  double lookahead_distance_{0};
};

}  // namespace ros2_behavior_tree

#endif  // PURE_PURSUIT_CONTROLLER_HPP_
