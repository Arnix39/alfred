#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "hal_proxsens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_100CM 100
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

using namespace std::placeholders;
using namespace std::chrono_literals;

const hal_pigpio_interfaces::msg::HalPigpioEdgeChange & edgeChangeMessage(uint8_t gpioId, uint8_t edgeChangeType, uint32_t timeSinceBoot_us)
{
    auto message = hal_pigpio_interfaces::msg::HalPigpioEdgeChange();

    message.gpio_id = gpioId;
    message.edge_change_type = edgeChangeType;
    message.time_since_boot_us = timeSinceBoot_us;
    const hal_pigpio_interfaces::msg::HalPigpioEdgeChange  &messageToSend = message;

    return messageToSend;
}

class ProxsensCheckerNode : public rclcpp::Node
{
public : 
    ProxsensCheckerNode() : rclcpp::Node("hal_proxsens_checker_node"),
                            proxsensSub(this->create_subscription<hal_proxsens_interfaces::msg::HalProxsens>("proxSensorValue", 1000, std::bind(&ProxsensCheckerNode::getProxsensDistance, this, _1)))
    {
    }
    ~ProxsensCheckerNode() = default;
    void getProxsensDistance(const hal_proxsens_interfaces::msg::HalProxsens &msg)
    {
        distanceInCm.set_value(msg.distance_in_cm);
    }
    std::promise<uint16_t> distanceInCm;

private :
    rclcpp::Subscription<hal_proxsens_interfaces::msg::HalProxsens>::SharedPtr proxsensSub;
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

        executor.add_node(proxsens);
        executor.add_node(proxsensChecker);
    }

    void TearDown()
    {
        executor.cancel();
        executor.remove_node(proxsens);
        executor.remove_node(proxsensChecker);
        proxsens.reset();
        proxsensChecker.reset();
    }
};

/* Test cases */
TEST_F(ProxsensTest, sensorDistanceDefaultValue)
{
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxsensTest, sensorDistance10cm)
{
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 10580;
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxsensTest, sensorDistanceTwoFallingEdges)
{
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 15800;
    uint32_t timestampFirstFallingEdge = 20000;
    uint32_t timestampSecondFallingEdge = 20580;
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_100CM);

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFirstFallingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampSecondFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_100CM);
}

TEST_F(ProxsensTest, sensorDistance10cmWithTimestampRollout)
{
    uint32_t timestampRisingEdge = UINT32_MAX - 290;
    uint32_t timestampFallingEdge = 290;
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxsensTest, sensorDistanceFallingEdgeFirstWithRisingEdge)
{
    uint32_t timestampFirstFallingEdge = 9000;
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampSecondFallingEdge = 15800;
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFirstFallingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampSecondFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_100CM);
}

TEST_F(ProxsensTest, sensorDistanceTwoRisingEdges)
{
    uint32_t timestampFirstRisingEdge = 10000;
    uint32_t timestampSecondRisingEdge = 10580;
    uint32_t timestampFallingEdge = 11160;
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampFirstRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampSecondRisingEdge));
    proxsens->edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_10CM);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}