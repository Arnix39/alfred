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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_proxsens.hpp"

namespace hal
{
namespace proxsens
{
namespace test
{

#define PROX_SENS_DISTANCE_10CM 0.1
#define PROX_SENS_DISTANCE_1M 1.0
#define PROX_SENS_DISTANCE_DEFAULT_VALUE (UINT16_MAX / 100.0)

using namespace std::placeholders;
using namespace std::chrono_literals;

const pigpio_msg::HalPigpioEdgeChange & edgeChangeMessage(
  uint8_t gpioId,
  EdgeChangeType edgeChangeType,
  uint32_t timeSinceBoot_us)
{
  auto message = pigpio_msg::HalPigpioEdgeChange();

  message.gpio_id = gpioId;
  message.edge_change_type = static_cast<uint8_t>(edgeChangeType);
  message.time_since_boot_us = timeSinceBoot_us;
  const pigpio_msg::HalPigpioEdgeChange & messageToSend = message;

  return messageToSend;
}

class ProxsensCheckerNode : public rclcpp::Node
{
public:
  ProxsensCheckerNode()
  : rclcpp::Node{"hal_proxsens_checker_node"},
    proxsensSub{this->create_subscription
      <ProxsensMsg_t>(
        "proximitySensor",
        10,
        std::bind(&ProxsensCheckerNode::getProxsensDistance, this, _1))},
    changeStateClient{this->create_client<lifecycle_msgs::srv::ChangeState>(
        "hal_proxsens_node/change_state")}
  {
  }
  ~ProxsensCheckerNode() = default;
  void getProxsensDistance(const ProxsensMsg_t & msg)
  {
    distance.set_value(msg.range);
  }
  std::promise<float> distance;

  void changeProxsensNodeToState(std::uint8_t transition)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = changeStateClient->async_send_request(request);
  }

private:
  rclcpp::Subscription<ProxsensMsg_t>::SharedPtr proxsensSub;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;
};

/* Test fixture */
class ProxsensTest : public testing::Test
{
protected:
  std::shared_ptr<Proxsens> proxsens;
  std::shared_ptr<ProxsensCheckerNode> proxsensChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    proxsensChecker = std::make_shared<ProxsensCheckerNode>();
    proxsens = std::make_shared<Proxsens>();

    executor.add_node(proxsens->get_node_base_interface());
    executor.add_node(proxsensChecker);

    proxsensChecker->changeProxsensNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    proxsensChecker->changeProxsensNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executor.spin_some();
  }

  void TearDown()
  {
    proxsensChecker->changeProxsensNodeToState(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
    executor.spin_some();
    executor.cancel();
    executor.remove_node(proxsens->get_node_base_interface());
    executor.remove_node(proxsensChecker);
    proxsens.reset();
    proxsensChecker.reset();
  }
};

/* Test cases */
TEST_F(ProxsensTest, sensorDistanceDefaultValue)
{
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxsensTest, sensorDistance10cm)
{
  uint32_t timestampRisingEdge = 10000;
  uint32_t timestampFallingEdge = 10580;
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::rising,
      timestampRisingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampFallingEdge));
  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxsensTest, sensorDistanceTwoFallingEdges)
{
  uint32_t timestampFirstFallingEdge = 10000;
  uint32_t timestampSecondFallingEdge = 10580;
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampFirstFallingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampSecondFallingEdge));
  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxsensTest, sensorDistance10cmWithTimestampRollout)
{
  uint32_t timestampRisingEdge = UINT32_MAX - 290;
  uint32_t timestampFallingEdge = 290;
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::rising,
      timestampRisingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampFallingEdge));
  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxsensTest, sensorDistanceFallingEdgeFirstWithRisingEdge)
{
  uint32_t timestampFirstFallingEdge = 9000;
  uint32_t timestampRisingEdge = 10000;
  uint32_t timestampSecondFallingEdge = 15800;
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampFirstFallingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::rising,
      timestampRisingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampSecondFallingEdge));
  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_1M);
}

TEST_F(ProxsensTest, sensorDistanceTwoRisingEdges)
{
  uint32_t timestampFirstRisingEdge = 10000;
  uint32_t timestampSecondRisingEdge = 11000;
  uint32_t timestampFallingEdge = 11580;
  auto future = std::shared_future<float>(proxsensChecker->distance.get_future());

  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::rising,
      timestampFirstRisingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::rising,
      timestampSecondRisingEdge));
  proxsens->edgeChangeCallback(
    edgeChangeMessage(
      PROXSENS_ECHO_GPIO, EdgeChangeType::falling,
      timestampFallingEdge));
  proxsens->publishDistance();
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FLOAT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

}  // namespace test
}  // namespace proxsens
}  // namespace hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
