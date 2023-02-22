// Copyright (c) 2022 Arnix Robotix
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

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "gpioDefinitions.hpp"

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

using LifecycleCallbackReturn_t =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template<class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;

public:
  explicit ServiceNodeSync(std::string name)
  : node(std::make_shared<rclcpp::Node>(name)) {}

  ~ServiceNodeSync()
  {
    node.reset();
    client.reset();
  }

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT & req)
  {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  std::shared_ptr<ResponseT> sendRequest(const std::shared_ptr<RequestT> & req_ptr)
  {
    std::shared_ptr<ResponseT> response;
    auto result = client->async_send_request(req_ptr);
    auto status = rclcpp::spin_until_future_complete(
      node, result, std::chrono::duration<int64_t, std::milli>(1000));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      response = result.get();
    }

    return response;
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};


#endif  // COMMON_HPP_
