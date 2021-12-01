#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestHal, testHal_succeed)
{
    SUCCEED();
}

TEST(TestHal, testHal_fail)
{
    FAIL();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testHal");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}