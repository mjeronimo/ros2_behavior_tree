// Copyright (c) 2018 Intel Corporation
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

#include "behaviortree_cpp/behavior_tree.h"

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

template<>
inline long convertFromString<long>(const StringView key)
{
  long l = std::stol(key.data());
}

}  // namespace BT

#endif  // ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
