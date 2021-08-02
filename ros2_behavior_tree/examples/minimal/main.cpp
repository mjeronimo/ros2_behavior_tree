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

#include <iostream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"

#if 1
// The Behavior Tree to execute. This tree simple prints "Hello," and
// "World!" on separate lines and then terminates.
static const char bt_xml[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="say_hello">
      <Message msg="Hello,"/>
      <Message msg="World!"/>
    </Sequence>
  </BehaviorTree>
</root>
)";
#else
static const char bt_xml[] =
  R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="FollowTheLeader">
      <Sequence name="startup">
        <CreateROS2Node node_name="node1" namespace="robot1" spin="true" node_handle="{ros_node_1}"/>
        <CreateROS2Node node_name="node2" namespace="robot2" spin="true" node_handle="{ros_node_2}"/>
        <CreateTransformBuffer node_handle="{ros_node_1}" tf_buffer="{tf_1}"/>
        <CreateTransformBuffer node_handle="{ros_node_2}" tf_buffer="{tf_2}"/>
        <Recovery num_retries="7">
          <Sequence>
            <Message msg="Trying transforms..."/>
            <CanTransform node_handle="{ros_node_1}" tf_buffer="{tf_1}" source_frame="base_link" target_frame="map"/>
            <CanTransform node_handle="{ros_node_2}" tf_buffer="{tf_2}" source_frame="base_link" target_frame="map"/>
          </Sequence>
          <Sequence>
            <Message msg="Waiting for base_link to map transform to become available..."/>
            <AsyncWait msec="1000"/>
          </Sequence>
        </Recovery>
      </Sequence>
      <Message msg="Getting robot poses..."/>
      <Forever>
        <ReactiveSequence>
          <Message msg="Checking for a safe distance"/>
          <TransformPose tf_buffer="{tf_1}" source_frame="base_link" target_frame="map" pose="{leader_pose}"/>
          <TransformPose tf_buffer="{tf_2}" source_frame="base_link" target_frame="map" pose="{follower_pose}"/>
          <DistanceConstraint threshold="0.4" pose_1="{leader_pose}" pose_2="{follower_pose}">
            <PipelineSequence>
              <ThrottleTickRate hz="1.0">
                <Sequence>
                  <Message msg="GetPoseNearRobot"/>
                  <GetPosesNearRobot node_handle="{ros_node_1}" min_distance="0.5" max_distance="1.5" step_distance="0.25" robot_pose="{leader_pose}" nearby_poses="{nearby_poses}"/>
                  <ForEachPose poses="{nearby_poses}" pose="{goal_pose}">
                    <Sequence>
                      <Message msg="Computing path to the goal"/>
                      <ComputePathToPose action_name="compute_path_to_pose" server_timeout="1000" ros2_node="{ros_node_2}" goal="{goal_pose}" planner_id="GridBased" path="{path_to_follow}"/>
                    </Sequence>
                  </ForEachPose>
                  <AsyncWait msec="1000"/>
                </Sequence>
              </ThrottleTickRate>
              <FollowPath action_name="follow_path" server_timeout="1000" ros2_node="{ros_node_2}" path="{path_to_follow}" controller_id="FollowPath"/>
            </PipelineSequence>
          </DistanceConstraint>
        </ReactiveSequence>
      </Forever>
    </Sequence>
  </BehaviorTree>
</root>
)";
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a behavior tree using the input XML
  ros2_behavior_tree::BehaviorTree bt(bt_xml);

  // Execute the BT and determine the result
  switch (bt.execute()) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      std::cout << "BT failed\n";
      break;

    case ros2_behavior_tree::BtStatus::HALTED:
      std::cout << "BT was halted\n";
      break;

    default:
      throw std::logic_error("Invalid return value from the BT");
  }

  rclcpp::shutdown();
  return 0;
}
