#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hal_imu.hpp"

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hal_imu");
    return RUN_ALL_TESTS();
}