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

#ifndef ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data types.

namespace BT
{

template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

template<>
inline int64_t convertFromString<int64_t>(const StringView key)
{
  return std::strtoll(key.data(), NULL, 10);
}

template<>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 2) {
    throw std::runtime_error("Invalid number of fields for PoseStamped attribute)");
  } else {
    auto pose_fields = BT::splitString(parts[0], ',');
    auto orientation_fields = BT::splitString(parts[1], ',');

    if (pose_fields.size() != 3 || orientation_fields.size() != 4) {
      throw std::runtime_error("Invalid number of fields for PoseStamped attribute)");
    } 

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = BT::convertFromString<double>(pose_fields[0]);
    pose.pose.position.y = BT::convertFromString<double>(pose_fields[1]);
    pose.pose.position.z = BT::convertFromString<double>(pose_fields[2]);
    pose.pose.orientation.x = BT::convertFromString<double>(orientation_fields[0]);
    pose.pose.orientation.y = BT::convertFromString<double>(orientation_fields[1]);
    pose.pose.orientation.z = BT::convertFromString<double>(orientation_fields[2]);
    pose.pose.orientation.w = BT::convertFromString<double>(orientation_fields[3]);

    return pose;
  }
}

}  // namespace BT

#endif  // ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
