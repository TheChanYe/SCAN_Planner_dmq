#include <gtest/gtest.h>
#include "navdog_runtime/final_command_publisher.hpp"

#include <limits>

TEST(FinalCommandPublisher, ForwardsFiniteCommandUnchanged)
{
  geometry_msgs::Twist input;
  input.linear.x = 0.2;
  input.linear.y = -0.1;
  input.angular.z = 0.3;
  const auto output = navdog_runtime::FinalCommandPublisher::validated(input);
  EXPECT_DOUBLE_EQ(input.linear.x, output.linear.x);
  EXPECT_DOUBLE_EQ(input.linear.y, output.linear.y);
  EXPECT_DOUBLE_EQ(input.angular.z, output.angular.z);
}

TEST(FinalCommandPublisher, RejectsNonFiniteAndStaleInput)
{
  geometry_msgs::Twist input;
  input.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(0.0,
      navdog_runtime::FinalCommandPublisher::validated(input).linear.x);
  EXPECT_TRUE(navdog_runtime::FinalCommandPublisher::fresh(1.0, 1.2, 0.3));
  EXPECT_FALSE(navdog_runtime::FinalCommandPublisher::fresh(1.0, 1.4, 0.3));
}
