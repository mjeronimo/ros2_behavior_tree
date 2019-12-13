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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace ros2_behavior_tree
{

class PurePursuitNode : public BT::SyncActionNode
{
public:
  PurePursuitNode(const std::string & name, const BT::NodeConfiguration & config);
  virtual ~PurePursuitNode();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "The ROS2 node to use"),
      BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", "The transform buffer to use")
    };
  }

  BT::NodeStatus tick() override;

protected:
  bool step(geometry_msgs::msg::Twist & twist);
  geometry_msgs::msg::PoseStamped get_current_pose() const;
  double get_lookahead_distance(const geometry_msgs::msg::PoseStamped & pose) const;
  double get_lookahead_angle(const geometry_msgs::msg::PoseStamped & pose) const;
  double get_lookahead_threshold() const;
  double get_arc_distance(const geometry_msgs::msg::PoseStamped & pose) const;
  int get_next_waypoint(int wayPoint) const;
  int get_closest_waypoint() const;
  bool get_interpolated_pose(int wayPoint, geometry_msgs::msg::PoseStamped & interpolatedPose) const;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path cur_ref_path_;
  geometry_msgs::msg::Twist current_velocity_;

  std::string pose_frame_id_{"base"};
  int next_waypoint_{-1};
  double velocity_{0.2};         // m/s
  double lookahead_ratio_{1.0};  // w.r.t. velocity
  double epsilon_{1e-6};

  //////////////////////////////////////
  // Odom and command velocity -related
  int queue_depth_{100};
  std::string path_topic_name_{"reference_path"};
  std::string odom_topic_name_{"odom"};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string cmd_vel_topic_name_{"cmd_vel"};
  std::string cmd_traj_topic_name_{"/local_planner_solution_trajectory"};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_traj_pub_;
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  //////////////////////////////////////
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PURE_PURSUIT_NODE_HPP_
