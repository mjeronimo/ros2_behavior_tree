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


#include "test_node_registrar.hpp"

#include "ros2_behavior_tree/node_registrar.hpp"
#include "add_two_ints_client.hpp"
#include "fibonacci_client.hpp"

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::TestNodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
TestNodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<AddTwoIntsClient>("AddTwoInts");
  factory.registerNodeType<FibonacciClient>("Fibonacci");
}

}  // namespace ros2_behavior_tree
