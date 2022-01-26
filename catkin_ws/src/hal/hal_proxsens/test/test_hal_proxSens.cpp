#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_proxSens.hpp"

static uint16_t distanceInCm = 0;

class ProxSensPublisher_mock : public ProxSensPublisher
{
public:
    ProxSensPublisher_mock() = default;
    ~ProxSensPublisher_mock() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
    uint16_t distanceInCm;
};

void ProxSensPublisher_mock::publish(hal_proxsens::hal_proxsensMsg message)
{
    distanceInCm = message.distanceInCm;
}

TEST(TestHalProxSens, sensorDistanceDefaultValue)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisher_mock proxSensPublisher = ProxSensPublisher_mock();

    ProxSens proxSens = ProxSens(&node, &proxSensPublisher);

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, UINT16_MAX);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}