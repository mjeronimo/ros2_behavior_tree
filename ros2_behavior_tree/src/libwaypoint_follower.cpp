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

#include "ros2_behavior_tree/libwaypoint_follower.hpp"

#include <vector>

namespace tf2 {

typedef tf2::Vector3 Point;
static inline void poseMsgToTF(const geometry_msgs::msg::Pose& msg, Transform & bt)
{
  bt = Transform(
      Quaternion(
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}
static inline void pointTFToMsg(const tf2::Point& bt_v, geometry_msgs::msg::Point& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();}
static inline void pointMsgToTF(const geometry_msgs::msg::Point& msg_v, tf2::Point& bt_v) {bt_v = tf2::Vector3(msg_v.x, msg_v.y, msg_v.z);}

}  // namespace tf2

int
WayPoints::getSize() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;
  else
    return current_waypoints_.waypoints.size();
}

double
WayPoints::getInterval() const
{
  if (current_waypoints_.waypoints.empty())
    return 0;

  // interval between 2 waypoints
  tf2::Vector3 v1(current_waypoints_.waypoints[0].pose.pose.position.x,
                 current_waypoints_.waypoints[0].pose.pose.position.y, 0);

  tf2::Vector3 v2(current_waypoints_.waypoints[1].pose.pose.position.x,
                 current_waypoints_.waypoints[1].pose.pose.position.y, 0);
  return tf2::tf2Distance(v1, v2);
}

geometry_msgs::msg::Point
WayPoints::getWaypointPosition(int waypoint) const
{
  geometry_msgs::msg::Point p;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return p;

  p = current_waypoints_.waypoints[waypoint].pose.pose.position;
  return p;
}

geometry_msgs::msg::Quaternion
WayPoints::getWaypointOrientation(int waypoint) const
{
  geometry_msgs::msg::Quaternion q;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return q;

  q = current_waypoints_.waypoints[waypoint].pose.pose.orientation;
  return q;
}

geometry_msgs::msg::Pose
WayPoints::getWaypointPose(int waypoint) const
{
  geometry_msgs::msg::Pose pose;
  if (waypoint > getSize() - 1 || waypoint < 0)
    return pose;

  pose = current_waypoints_.waypoints[waypoint].pose.pose;
  return pose;
}

double
WayPoints::getWaypointVelocityMPS(int waypoint) const
{
  if (waypoint > getSize() - 1 || waypoint < 0)
    return 0;

  return current_waypoints_.waypoints[waypoint].twist.twist.linear.x;
}

bool
WayPoints::isFront(int waypoint, geometry_msgs::msg::Pose current_pose) const
{
  double x = calcRelativeCoordinate(current_waypoints_.waypoints[waypoint].pose.pose.position, current_pose).x;

  if (x < 0)
    return false;
  else
    return true;
}

double
DecelerateVelocity(double distance, double prev_velocity)
{
  double decel_ms = 1.0;  // m/s
  double decel_velocity_ms = sqrt(2 * decel_ms * distance);

  std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
  if (decel_velocity_ms < prev_velocity) {
    return decel_velocity_ms;
  } else {
    return prev_velocity;
  }
}

// calculation relative coordinate of point from current_pose frame
geometry_msgs::msg::Point
calcRelativeCoordinate(geometry_msgs::msg::Point point_msg, geometry_msgs::msg::Pose current_pose)
{
  tf2::Transform inverse;
  tf2::poseMsgToTF(current_pose, inverse);
  tf2::Transform transform = inverse.inverse();

  tf2::Point p;
  tf2::pointMsgToTF(point_msg, p);
  tf2::Point tf_p = transform * p;
  geometry_msgs::msg::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// calculation absolute coordinate of point on current_pose frame
geometry_msgs::msg::Point
calcAbsoluteCoordinate(geometry_msgs::msg::Point point_msg, geometry_msgs::msg::Pose current_pose)
{
  tf2::Transform inverse;
  tf2::poseMsgToTF(current_pose, inverse);

  tf2::Point p;
  tf2::pointMsgToTF(point_msg, p);
  tf2::Point tf_p = inverse * p;

  geometry_msgs::msg::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// distance between target 1 and target2 in 2-D
double
getPlaneDistance(geometry_msgs::msg::Point target1, geometry_msgs::msg::Point target2)
{
  tf2::Vector3 v1 = point2vector(target1);
  v1.setZ(0);

  tf2::Vector3 v2 = point2vector(target2);
  v2.setZ(0);

  return tf2::tf2Distance(v1, v2);
}

double
getRelativeAngle(geometry_msgs::msg::Pose waypoint_pose, geometry_msgs::msg::Pose vehicle_pose)
{
  geometry_msgs::msg::Point relative_p1 = calcRelativeCoordinate(waypoint_pose.position, vehicle_pose);
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  geometry_msgs::msg::Point relative_p2 = calcRelativeCoordinate(calcAbsoluteCoordinate(p2, waypoint_pose), vehicle_pose);
  tf2::Vector3 relative_waypoint_v(relative_p2.x - relative_p1.x, relative_p2.y - relative_p1.y,
                                  relative_p2.z - relative_p1.z);
  relative_waypoint_v.normalize();
  tf2::Vector3 relative_pose_v(1, 0, 0);
  double angle = relative_pose_v.angle(relative_waypoint_v) * 180 / M_PI;
  // ROS_INFO("angle : %lf",angle);

  return angle;
}

// get closest waypoint from current pose
int
getClosestWaypoint(const ros2_behavior_tree_msgs::msg::Lane &current_path, geometry_msgs::msg::Pose current_pose)
{
  WayPoints wp;
  wp.setPath(current_path);

  if (wp.isEmpty())
    return -1;

  // search closest candidate within a certain meter
  double search_distance = 5.0;
  std::vector<int> waypoint_candidates;
  for (int i = 1; i < wp.getSize(); i++)
  {
    if (getPlaneDistance(wp.getWaypointPosition(i), current_pose.position) > search_distance)
      continue;

    if (!wp.isFront(i, current_pose))
      continue;

    double angle_threshold = 90;
    if (getRelativeAngle(wp.getWaypointPose(i), current_pose) > angle_threshold)
      continue;

    waypoint_candidates.push_back(i);
  }

  // get closest waypoint from candidates
  if (!waypoint_candidates.empty()) {
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (auto el : waypoint_candidates)
    {
      // ROS_INFO("closest_candidates : %d",el);
      double d = getPlaneDistance(wp.getWaypointPosition(el), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = el;
        distance_min = d;
      }
    }
    return waypoint_min;
  } else {
    // ROS_INFO("no candidate. search closest waypoint from all waypoints...");
    // if there is no candidate...
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (int i = 1; i < wp.getSize(); i++)
    {
      if (!wp.isFront(i, current_pose))
        continue;

      // if (!wp.isValid(i, current_pose))
      //  continue;

      double d = getPlaneDistance(wp.getWaypointPosition(i), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = i;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool
getLinearEquation(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end, double *a, double *b, double *c)
{
  // (x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    // ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}

double
getDistanceBetweenLineAndPoint(geometry_msgs::msg::Point point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));
  return d;
}

tf2::Vector3
point2vector(geometry_msgs::msg::Point point)
{
  tf2::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

geometry_msgs::msg::Point
vector2point(tf2::Vector3 vector)
{
  geometry_msgs::msg::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();

  return point;
}

tf2::Vector3
rotateUnitVector(tf2::Vector3 unit_vector, double degree)
{
  tf2::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
                 sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
  tf2::Vector3 unit_w1 = w1.normalize();

  return unit_w1;
}

geometry_msgs::msg::Point
rotatePoint(geometry_msgs::msg::Point point, double degree)
{
  geometry_msgs::msg::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}

