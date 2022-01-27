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

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST(TestHalProxSens, sensorDistance10cm)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisherMock proxSensPublisher = ProxSensPublisherMock();
    ProxSensSubscriberMock proxSensSubscriber = ProxSensSubscriberMock();

    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 10590;

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

TEST(TestHalProxSens, sensorDistance10cmWithTimestampRollout)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisherMock proxSensPublisher = ProxSensPublisherMock();
    ProxSensSubscriberMock proxSensSubscriber = ProxSensSubscriberMock();

    uint32_t timestampRisingEdge = UINT32_MAX - 295;
    uint32_t timestampFallingEdge = 295;

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

TEST(TestHalProxSens, sensorDistanceFallingEdgeFirst)
{
    ros::NodeHandle node;
    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;
    ProxSensPublisherMock proxSensPublisher = ProxSensPublisherMock();
    ProxSensSubscriberMock proxSensSubscriber = ProxSensSubscriberMock();

    uint32_t timestampFallingEdge = 10000;
    uint32_t timestampRisingEdge = 10590;

    ProxSens proxSens = ProxSens(&node, &proxSensSubscriber, &proxSensPublisher);

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisher.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

// TODO: add test fixture

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}