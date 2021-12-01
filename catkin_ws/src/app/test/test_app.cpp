#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestApp, testApp_succeed)
{
    SUCCEED();
}

TEST(TestApp, testApp_fail)
{
    FAIL();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testApp");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}