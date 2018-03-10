#include <gtest/gtest.h>

#define GTEST

TEST(TestSuite, test_trivialtest)
{
  ASSERT_TRUE(true);
  ASSERT_GE(1,0);
  ASSERT_FALSE(false);
  ASSERT_NE(1,0);
  ASSERT_LE(0,1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}