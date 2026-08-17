#include "navdog_runtime/cmd_speed_limit.hpp"

#include <gtest/gtest.h>

TEST(CmdSpeedLimit, UsesTaskLimitWhenLowerThanModeLimit)
{
  EXPECT_DOUBLE_EQ(0.2,
      navdog_runtime::effectiveLinearSpeedLimit(0.3, 0.2, true));
}

TEST(CmdSpeedLimit, RouteFollowUsesTaskLimitWhenLower)
{
  EXPECT_DOUBLE_EQ(0.2,
      navdog_runtime::effectiveLinearSpeedLimit(0.7, 0.2, true));
}

TEST(CmdSpeedLimit, LocalAvoidUsesTaskLimitWhenLower)
{
  EXPECT_DOUBLE_EQ(0.2,
      navdog_runtime::effectiveLinearSpeedLimit(0.52, 0.2, true));
}

TEST(CmdSpeedLimit, KeepsModeLimitWhenTaskLimitIsHigher)
{
  EXPECT_DOUBLE_EQ(0.3,
      navdog_runtime::effectiveLinearSpeedLimit(0.3, 0.9, true));
}

TEST(CmdSpeedLimit, IgnoresMissingOrInvalidTaskLimit)
{
  EXPECT_DOUBLE_EQ(0.3,
      navdog_runtime::effectiveLinearSpeedLimit(0.3, 0.0, false));
  EXPECT_DOUBLE_EQ(0.3,
      navdog_runtime::effectiveLinearSpeedLimit(0.3, -1.0, true));
}
