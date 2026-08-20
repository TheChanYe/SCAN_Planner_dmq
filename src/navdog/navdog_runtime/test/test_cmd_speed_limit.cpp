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

TEST(CmdSpeedLimit, MotionScalePreservesCurvatureWhenLimited)
{
  const double scale =
      navdog_runtime::proportionalMotionScale(0.52, 0.0, 0.30);
  EXPECT_NEAR(0.30 / 0.52, scale, 1e-9);

  const double limited_vx = 0.52 * scale;
  const double limited_w = 0.40 * scale;
  EXPECT_NEAR(0.30, limited_vx, 1e-9);
  EXPECT_NEAR(0.2307692308, limited_w, 1e-9);
  EXPECT_NEAR(0.40 / 0.52, limited_w / limited_vx, 1e-9);
}

TEST(CmdSpeedLimit, MotionScaleKeepsPureRotation)
{
  EXPECT_DOUBLE_EQ(1.0,
      navdog_runtime::proportionalMotionScale(0.0, 0.0, 0.30));
}
