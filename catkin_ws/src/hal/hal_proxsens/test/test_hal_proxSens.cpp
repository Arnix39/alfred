#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_proxSens.hpp"

#define PROX_SENS_DISTANCE_10CM 10
#define PROX_SENS_DISTANCE_DEFAULT_VALUE UINT16_MAX

/* Publisher interface mock */
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

/* Subscriber interface mock */
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

/* Services interface mock */
class ProxSensClientsMock : public ProxSensClients
{
private:
    ros::ServiceClient SetInputClientMock;
    ros::ServiceClient SetCallbackClientMock;
    ros::ServiceClient SetOutputClientmock;
    ros::ServiceClient SendTriggerPulseClientMock;

public:
    ProxSensClientsMock() = default;
    ~ProxSensClientsMock() = default;
    ros::ServiceClient *getSetInputClientHandle() override;
    ros::ServiceClient *getSetCallbackClientHandle() override;
    ros::ServiceClient *getSetOutputClientHandle() override;
    ros::ServiceClient *getSendTriggerPulseClientHandle() override;
};

ros::ServiceClient *ProxSensClientsMock::getSetInputClientHandle()
{
    return &SetInputClientMock;
}

ros::ServiceClient *ProxSensClientsMock::getSetCallbackClientHandle()
{
    return &SetCallbackClientMock;
}

ros::ServiceClient *ProxSensClientsMock::getSetOutputClientHandle()
{
    return &SetOutputClientmock;
}

ros::ServiceClient *ProxSensClientsMock::getSendTriggerPulseClientHandle()
{
    return &SendTriggerPulseClientMock;
}

/* Helper functions */
const hal_pigpio::hal_pigpioEdgeChangeMsg &edgeChangeMessage(uint8_t gpioId, uint8_t edgeChangeType, uint32_t timeSinceBoot_us)
{
    hal_pigpio::hal_pigpioEdgeChangeMsg msg;
    msg.gpioId = gpioId;
    msg.edgeChangeType = edgeChangeType;
    msg.timeSinceBoot_us = timeSinceBoot_us;
    const hal_pigpio::hal_pigpioEdgeChangeMsg &msgToSend = msg;

    return msgToSend;
}

/* Test fixtures */
class ProxSensTest : public testing::Test
{
protected:
    ros::NodeHandle node;
    ProxSensPublisherMock proxSensPublisherMock;
    ProxSensSubscriberMock proxSensSubscriberMock;
    ProxSensClientsMock proxSensServiceClientsMock;
    ProxSens proxSens;

public:
    ProxSensTest() : proxSens(&proxSensSubscriberMock, &proxSensPublisherMock, &proxSensServiceClientsMock)
    {
    }
};

/* Test cases */
TEST_F(ProxSensTest, sensorDistanceDefaultValue)
{
    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxSensTest, sensorDistanceFallingEdgeFirst)
{
    uint32_t timestampFallingEdge = 10000;
    uint32_t timestampRisingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_DEFAULT_VALUE);
}

TEST_F(ProxSensTest, sensorDistance10cm)
{
    uint32_t timestampRisingEdge = 10000;
    uint32_t timestampFallingEdge = 10590;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

TEST_F(ProxSensTest, sensorDistance10cmWithTimestampRollout)
{
    uint32_t timestampRisingEdge = UINT32_MAX - 295;
    uint32_t timestampFallingEdge = 295;

    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, RISING_EDGE, timestampRisingEdge));
    proxSens.edgeChangeCallback(edgeChangeMessage(PROXSENS_ECHO_GPIO, FALLING_EDGE, timestampFallingEdge));

    proxSens.publishMessage();

    ASSERT_EQ(proxSensPublisherMock.distanceInCm, PROX_SENS_DISTANCE_10CM);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}