//
// Created by Megan Finch on 14/01/2022.
//


#include <gtest/gtest.h>

TEST(ExampleTest, BasicAssertions) {
    EXPECT_STRNE("hello", "world");
    EXPECT_EQ(7 * 6, 42);
}
