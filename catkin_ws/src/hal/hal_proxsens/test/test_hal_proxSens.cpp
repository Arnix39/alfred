#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_proxSens.hpp"

TEST(TestHalProxSens, testHalProxSens_success)
{
    ros::NodeHandle node;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    uint16_t distanceInCm = 0;

    hal_pigpio::hal_pigpioEdgeChangeMsg edgeChangeMsg;

    ProxSens proxSens = ProxSens(&node);

    publisher = node.advertise<hal_pigpio::hal_pigpioEdgeChangeMsg>("gpioEdgeChange", 1000);
    subscriber = node.subscribe<hal_proxsens::hal_proxsensMsg>("proxSensorValue", 1000, [&distanceInCm](const ros::MessageEvent<hal_proxsens::hal_proxsensMsg const> &event)
                                                               {
                                                                auto msg = *(event.getConstMessage());
                                                                distanceInCm = msg.distanceInCm; });

    edgeChangeMsg.gpioId = PROXSENS_ECHO_GPIO;
    edgeChangeMsg.edgeChangeType = RISING_EDGE;
    edgeChangeMsg.timeSinceBoot_us = 10000;

    publisher.publish(edgeChangeMsg);

    ros::WallDuration(1.0).sleep();
    ros::spinOnce();

    edgeChangeMsg.gpioId = PROXSENS_ECHO_GPIO;
    edgeChangeMsg.edgeChangeType = FALLING_EDGE;
    edgeChangeMsg.timeSinceBoot_us = 10590;

    publisher.publish(edgeChangeMsg);

    ros::WallDuration(1.0).sleep();
    ros::spinOnce();

    proxSens.publishMessage();

    ros::spinOnce();

    ASSERT_EQ(distanceInCm, 10);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_proxSens");
    return RUN_ALL_TESTS();
}