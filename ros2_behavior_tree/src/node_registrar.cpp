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

#include "ros2_behavior_tree/node_registrar.hpp"

#include <string>

#include "ros2_behavior_tree/action/async_wait_node.hpp"
#include "ros2_behavior_tree/action/compute_path_to_pose_node.hpp"
#include "ros2_behavior_tree/action/create_ros2_node.hpp"
#include "ros2_behavior_tree/action/create_transform_buffer_node.hpp"
#include "ros2_behavior_tree/action/follow_path_node.hpp"
#include "ros2_behavior_tree/action/get_poses_near_robot_node.hpp"
#include "ros2_behavior_tree/action/transform_pose_node.hpp"
#include "ros2_behavior_tree/condition/can_transform_node.hpp"
#include "ros2_behavior_tree/control/first_result_node.hpp"
#include "ros2_behavior_tree/control/pipeline_sequence_node.hpp"
#include "ros2_behavior_tree/control/recovery_node.hpp"
#include "ros2_behavior_tree/control/round_robin_node.hpp"
#include "ros2_behavior_tree/decorator/distance_constraint_node.hpp"
#include "ros2_behavior_tree/decorator/for_each_pose_node.hpp"
#include "ros2_behavior_tree/decorator/forever_node.hpp"
#include "ros2_behavior_tree/decorator/repeat_until_node.hpp"
#include "ros2_behavior_tree/decorator/throttle_tick_rate_node.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::NodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
NodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  const BT::PortsList message_params {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("Message",
    std::bind(&NodeRegistrar::message, std::placeholders::_1), message_params);

  factory.registerNodeType<ros2_behavior_tree::AsyncWaitNode>("AsyncWait");
  factory.registerNodeType<ros2_behavior_tree::CanTransformNode>("CanTransform");
  factory.registerNodeType<ros2_behavior_tree::ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<ros2_behavior_tree::CreateROS2Node>("CreateROS2Node");
  factory.registerNodeType<ros2_behavior_tree::CreateTransformBufferNode>("CreateTransformBuffer");
  factory.registerNodeType<ros2_behavior_tree::DistanceConstraintNode>("DistanceConstraint");
  factory.registerNodeType<ros2_behavior_tree::FirstResultNode>("FirstResult");
  factory.registerNodeType<ros2_behavior_tree::FollowPathNode>("FollowPath");
  factory.registerNodeType<ros2_behavior_tree::ForeverNode>("Forever");
  factory.registerNodeType<ros2_behavior_tree::ForEachPoseNode>("ForEachPose");
  factory.registerNodeType<ros2_behavior_tree::GetPosesNearRobotNode>("GetPosesNearRobot");
  factory.registerNodeType<ros2_behavior_tree::PipelineSequenceNode>("PipelineSequence");
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("Recovery");
  factory.registerNodeType<ros2_behavior_tree::RepeatUntilNode>("RepeatUntil");
  factory.registerNodeType<ros2_behavior_tree::RoundRobinNode>("RoundRobin");
  factory.registerNodeType<ros2_behavior_tree::ThrottleTickRateNode>("ThrottleTickRate");
  factory.registerNodeType<ros2_behavior_tree::TransformPoseNode>("TransformPose");
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
NodeRegistrar::message(BT::TreeNode & tree_node)
{
  std::string msg;
  tree_node.getInput<std::string>("msg", msg);

  printf(ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET "\n", msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree
