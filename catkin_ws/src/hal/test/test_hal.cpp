#include <ros/ros.h>
#include <gtest/gtest.h>

// Declare a test
TEST(TestHal, testCase1)
{
    //SUCCEED();
    FAIL();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testHal");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}