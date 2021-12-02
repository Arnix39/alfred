#include "gtest/gtest.h"
#include "gmock/gmock.h"

TEST(TestApp, testApp_success)
{
    SUCCEED();
}

TEST(TestApp, testApp_failure)
{
    FAIL();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}