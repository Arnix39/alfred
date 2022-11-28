#ifndef COMMON_HPP_
#define COMMON_HPP_

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

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync(std::string name): node(std::make_shared<rclcpp::Node>(name)) 
  {}

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT &req)
  {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  ResponseT sendRequest(const std::shared_ptr<RequestT> &req_ptr)
  {
    auto result = client->async_send_request(req_ptr);
    rclcpp::spin_until_future_complete(node, result);
    return *result.get();
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};

#endif  // COMMON_HPP_
