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

#ifndef ADD_TWO_INTS_SERVER_HPP_
#define ADD_TWO_INTS_SERVER_HPP_

#include <memory>
#include <string>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  explicit AddTwoIntsServer(const std::string & name)
  : rclcpp::Node(name)
  {
    server_ = create_service<AddTwoInts>("add_two_ints",
        std::bind(&AddTwoIntsServer::handle_service, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    (void)request_header;
    response->sum = request->a + request->b;
  }

  rclcpp::Service<AddTwoInts>::SharedPtr server_;
};

#endif  // ADD_TWO_INTS_SERVER_HPP_
