#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "hal_proxsens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

using namespace std::placeholders;
using namespace std::chrono_literals;

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

/* Test fixtures */
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

private:
};

/* Test cases */
TEST_F(ProxsensTest, sensorDistanceDefaultValue)
{
    auto future = std::shared_future<uint16_t>(proxsensChecker->distanceInCm.get_future());

    proxsens->publishDistance();
    ASSERT_EQ(executor.spin_until_future_complete(future, 1s), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_EQ(future.get(), PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

/*TEST_F(ProxsensTest, sensorDistanceFallingEdgeFirst)
{
    uint32_t timestampFallingEdge = 10000;
    uint32_t timestampRisingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxsensTest, sensorDistance10cm)
{
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxsensTest, sensorDistance10cmWithTimestampRollout)
{
    uint32_t timestampRisingEdge = UINT32_MAX - 295;
    uint32_t timestampFallingEdge = 295;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_10CM);
}*/

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}