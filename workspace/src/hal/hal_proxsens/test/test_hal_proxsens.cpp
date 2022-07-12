#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "hal_proxsens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

using namespace std::placeholders;

class ProxsensCheckerNode : public rclcpp::Node
{
public : 
    ProxsensCheckerNode() : rclcpp::Node("hal_proxsens_checker_node"),
                            proxsensSub(this->create_subscription<hal_proxsens_interfaces::msg::HalProxsens>("proxsensDistance", 1000, std::bind(&ProxsensCheckerNode::getProxsensDistance, this, _1)))
    {
    }
    ~ProxsensCheckerNode() = default;
    void getProxsensDistance(const hal_proxsens_interfaces::msg::HalProxsens &msg)
    {
        distanceInCm = msg.distance_in_cm;
    }
    uint16_t distanceInCm;

private :
    rclcpp::Subscription<hal_proxsens_interfaces::msg::HalProxsens>::SharedPtr proxsensSub;
};

/* Test fixtures */
class ProxsensTest : public testing::Test
{
protected:
    std::shared_ptr<Proxsens> proxsens;
    std::shared_ptr<ProxsensCheckerNode> proxsensChecker;

    void SetUp()
    {
        proxsensChecker = std::make_shared<ProxsensCheckerNode>();
        proxsens = std::make_shared<Proxsens>();
    }

    void TearDown()
    {
        proxsens.reset();
        proxsensChecker.reset();
    }

private:
};

/* Test cases */
TEST_F(ProxsensTest, sensorDistanceDefaultValue)
{
    proxsens->publishDistance();

    ASSERT_EQ(proxsensChecker->distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
    //ASSERT_EQ(proxsensChecker->distanceInCm, 0);
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