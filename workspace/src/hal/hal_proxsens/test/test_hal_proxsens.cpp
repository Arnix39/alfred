#include "gtest/gtest.h"
//#include "gmock/gmock.h"
#include "rclcpp/rclcpp.hpp"

#include "hal_proxsens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

/* Test fixtures */
class ProxsensTest : public testing::Test
{
protected:
    void SetUp()
    {
        testerNode = std::make_shared<rclcpp::Node>("hal_proxsens_tester_node");
        Proxsens proxsens(testerNode);
    }

    void TearDown()
    {
    }

private:
    std::shared_ptr<rclcpp::Node> testerNode;
};

/* Test cases */
/*TEST_F(ProxsensTest, sensorDistanceDefaultValue)
{
    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxsensTest, sensorDistanceFallingEdgeFirst)
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
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}