#include "gtest/gtest.h"
#include "gmock/gmock.h"

TEST(TestHal, testHal_success)
{
    SUCCEED();
}

TEST(TestHal, testHal_failure)
{
    FAIL();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}