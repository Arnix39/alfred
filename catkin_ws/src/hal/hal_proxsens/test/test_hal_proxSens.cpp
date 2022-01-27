#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_proxSens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

class ProxSensPublisherMock : public ProxSensPublisher
{
public:
    ProxSensPublisherMock() = default;
    ~ProxSensPublisherMock() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
    uint16_t distanceInCm = 0;
};

void ProxSensPublisherMock::publish(hal_proxsens::hal_proxsensMsg message)
{
    distanceInCm = message.distanceInCm;
}

class ProxSensSubscriberMock : public ProxSensSubscriber
{
public:
    ProxSensSubscriberMock() = default;
    ~ProxSensSubscriberMock() = default;
    void subscribe(ProxSens *proxSens) override;
};

void ProxSensSubscriberMock::subscribe(ProxSens *proxSens)
{
    (void)proxSens;
}

const hal_pigpio::hal_pigpioEdgeChangeMsg &edgeChangeMessage(uint8_t gpioId, uint8_t edgeChangeType, uint32_t timeSinceBoot_us)
{
    hal_pigpio::hal_pigpioEdgeChangeMsg msg;
    msg.gpioId = gpioId;
    msg.edgeChangeType = edgeChangeType;
    msg.timeSinceBoot_us = timeSinceBoot_us;
    const hal_pigpio::hal_pigpioEdgeChangeMsg &msgToSend = msg;

    return msgToSend;
}

class ProxSensTest : public testing::Test
{
protected:
    ros::NodeHandle node;
    ProxSensPublisherMock proxSensPublisher;
    ProxSensSubscriberMock proxSensSubscriber;
    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);
};

TEST_F(ProxSensTest, sensorDistanceDefaultValue)
{
    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxSensTest, sensorDistanceFallingEdgeFirst)
{
    uint32_t timestampFallingEdge = 10000;
    uint32_t timestampRisingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxSensTest, sensorDistance10cm)
{
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxSensTest, sensorDistance10cmWithTimestampRollout)
{
    uint32_t timestampRisingEdge = UINT32_MAX - 295;
    uint32_t timestampFallingEdge = 295;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}