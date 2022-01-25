#include "gtest/gtest.h"
#include "gmock/gmock.h"

TEST(TestHalProxSens, testHalProxSens_success)
{
    SUCCEED();
}

TEST(TestHalProxSens, testHalProxSens_failure)
{
    FAIL();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}