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

#ifndef LIBWAYPOINT_FOLLOWER_HPP_
#define LIBWAYPOINT_FOLLOWER_HPP_

#include <iostream>
#include <sstream>
#include <fstream>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "ros2_behavior_tree_msgs/msg/lane.hpp"

class WayPoints
{
protected:
  ros2_behavior_tree_msgs::msg::Lane current_waypoints_;

public:
  void setPath(const ros2_behavior_tree_msgs::msg::Lane &waypoints)
  {
    current_waypoints_ = waypoints;
  }

  int getSize() const;

  bool isEmpty() const
  {
    return current_waypoints_.waypoints.empty();
  };

  double getInterval() const;
  geometry_msgs::msg::Point getWaypointPosition(int waypoint) const;
  geometry_msgs::msg::Quaternion getWaypointOrientation(int waypoint) const;
  geometry_msgs::msg::Pose getWaypointPose(int waypoint) const;
  double getWaypointVelocityMPS(int waypoint) const;

  ros2_behavior_tree_msgs::msg::Lane getCurrentWaypoints() const
  {
    return current_waypoints_;
  }

  bool isFront(int waypoint, geometry_msgs::msg::Pose current_pose) const;
};

// inline function (less than 10 lines )
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}  // convert degree to radian

tf2::Vector3 point2vector(geometry_msgs::msg::Point point);  // convert point to vector

geometry_msgs::msg::Point vector2point(tf2::Vector3 vector);  // convert vector to point

tf2::Vector3 rotateUnitVector(tf2::Vector3 unit_vector, double degree);  // rotate unit vector by degree

geometry_msgs::msg::Point rotatePoint(geometry_msgs::msg::Point point, double degree);  // rotate point vector by degree

double DecelerateVelocity(double distance, double prev_velocity);

geometry_msgs::msg::Point calcRelativeCoordinate(geometry_msgs::msg::Point point,
                                            geometry_msgs::msg::Pose current_pose);  // transform point into the coordinate
                                                                                // of current_pose
geometry_msgs::msg::Point calcAbsoluteCoordinate(geometry_msgs::msg::Point point,
                                            geometry_msgs::msg::Pose current_pose);  // transform point into the global
                                                                                // coordinate
double getPlaneDistance(geometry_msgs::msg::Point target1,
                        geometry_msgs::msg::Point target2);  // get 2 dimentional distance between target 1 and target 2
int getClosestWaypoint(const ros2_behavior_tree_msgs::msg::Lane &current_path, geometry_msgs::msg::Pose current_pose);

bool getLinearEquation(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end, double *a, double *b, double *c);

double getDistanceBetweenLineAndPoint(geometry_msgs::msg::Point point, double sa, double b, double c);

double getRelativeAngle(geometry_msgs::msg::Pose waypoint_pose, geometry_msgs::msg::Pose vehicle_pose);

#endif  // LIBWAYPOINT_FOLLOWER_HPP_

