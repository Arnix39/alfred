#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_proxSens.hpp"

static uint16_t distanceInCm = 0;

class ProxSensPublisherMock : public ProxSensPublisher
{
public:
    ProxSensPublisherMock() = default;
    ~ProxSensPublisherMock() = default;
    void publish(hal_proxsens::hal_proxsensMsg message) override;
    uint16_t distanceInCm;
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

TEST(TestHalProxSens, sensorDistanceDefaultValue)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisherMock proxSensPublisher = ProxSensPublisherMock();
    ProxSensSubscriberMock proxSensSubscriber = ProxSensSubscriberMock();
    distanceInCm = 0;

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, UINT16_MAX);
}

TEST(TestHalProxSens, sensorDistance10cm)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisherMock proxSensPublisher = ProxSensPublisherMock();
    ProxSensSubscriberMock proxSensSubscriber = ProxSensSubscriberMock();

    hal_pigpio::hal_pigpioEdgeChangeMsg msg2;
    msg2.gpioId = PROXSENS_ECHO_GPIO;
    msg2.edgeChangeType = FALLING_EDGE;
    msg2.timeSinceBoot_us = 10590;
    const hal_pigpio::hal_pigpioEdgeChangeMsg &msgFallingEdge = msg2;

    distanceInCm = 0;

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, 10000));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, 10590));

    proxSens.publishMessage();
    ASSERT_EQ(proxSensPublisher.distanceInCm, 10);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}