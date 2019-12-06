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

#ifndef ROS2_BEHAVIOR_TREE__CONTROL__RECOVERY_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONTROL__RECOVERY_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace ros2_behavior_tree
{

//
// @brief The RecoveryNode allows a "recovery action" to be invoked upon failure of
// a node, retrying the node that failed after a successful call to the recovery action.
//
// The RecoverNode must have exactly two children and behaves as follows:
//
// - If the first child returns SUCCESS, the second child is not invoked and the node
//   returns SUCCESS
//
// - If the first child returns FAILURE
//     If the retry count has not been exceeded, the second child will be executed
//       If the second child returns FAILURE, the node will return FAILURE
//       If the second child returns SUCCESS, the retry count is incremented and the
//       first child is executed again
//     If the retry count has been exceeded, the node returns FAILURE
//
// - If the first or second child returns RUNNING, this node returns RUNNING.
//
class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(const std::string & name, int retries)
  : BT::ControlNode::ControlNode(name, {}),
    num_retries_(retries), read_parameters_from_ports_(false)
  {
    setRegistrationID("Recovery");
  }

  RecoveryNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode::ControlNode(name, config), read_parameters_from_ports_(true)
  {
  }

  // Define this node's ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("num_retries", 1, "Number of retries")
    };
  }

  BT::NodeStatus tick() override
  {
    if (read_parameters_from_ports_) {
      if (!getInput("num_retries", num_retries_)) {
        throw BT::RuntimeError("Missing parameter [num_retries] in Recovery node");
      }
    }

    const unsigned children_count = children_nodes_.size();

    if (children_count != 2) {
      throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
    }

    if (status() == BT::NodeStatus::IDLE) {
      current_child_idx_ = 0;
      retry_count_ = 0;
    }

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < children_count && retry_count_ <= num_retries_) {
      TreeNode * child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child_node->executeTick();

      if (current_child_idx_ == 0) {
        switch (child_status) {
          case BT::NodeStatus::SUCCESS:
            haltChildren(0);
            return BT::NodeStatus::SUCCESS;

          case BT::NodeStatus::FAILURE:
            // tick the recovery action
            if (retry_count_ < num_retries_) {
              current_child_idx_++;
              break;
            } else {
              haltChildren(0);
              return BT::NodeStatus::FAILURE;
            }

          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

          default:
            throw BT::LogicError("Invalid status return from BT node");
            break;
        }

      } else if (current_child_idx_ == 1) {
        switch (child_status) {
          case BT::NodeStatus::SUCCESS:
            retry_count_++;
            current_child_idx_--;
            break;

          case BT::NodeStatus::FAILURE:
            haltChildren(0);
            return BT::NodeStatus::FAILURE;

          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

          default:
            throw BT::LogicError("Invalid status return from BT node");
            break;
        }
      }
    }

    haltChildren(0);
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    ControlNode::halt();
    current_child_idx_ = 0;
    retry_count_ = 0;
  }

private:
  bool read_parameters_from_ports_;
  unsigned int current_child_idx_{0};
  unsigned int num_retries_{0};
  unsigned int retry_count_{0};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONTROL__RECOVERY_NODE_HPP_
