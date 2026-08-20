#include "navdog_runtime/cmd_speed_limit.hpp"

#include <gtest/gtest.h>

TEST(CmdSpeedLimit, UsesTaskLimitWhenLowerThanModeLimit)
{
  EXPECT_DOUBLE_EQ(0.2,
      navdog_runtime::effectiveLinearSpeedLimit(0.3, 0.2, true));
}

TEST(CmdSpeedLimit, RouteFollowUsesTaskLimitWhenLower)
{
  EXPECT_DOUBLE_EQ(0.15,
      navdog_runtime::effectiveLinearSpeedLimit(0.70, 0.15, true));
  EXPECT_DOUBLE_EQ(0.30,
      navdog_runtime::effectiveLinearSpeedLimit(0.70, 0.30, true));
  EXPECT_DOUBLE_EQ(0.50,
      navdog_runtime::effectiveLinearSpeedLimit(0.70, 0.50, true));
  EXPECT_DOUBLE_EQ(0.70,
      navdog_runtime::effectiveLinearSpeedLimit(0.70, 0.90, true));
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

TEST(CmdSpeedLimit, LocalAvoidLimitIgnoresTaskLimit)
{
  const auto local_limit = [](double, bool) { return 0.30; };
  EXPECT_DOUBLE_EQ(0.30, local_limit(0.15, true));
  EXPECT_DOUBLE_EQ(0.30, local_limit(0.30, true));
  EXPECT_DOUBLE_EQ(0.30, local_limit(0.90, true));
  EXPECT_DOUBLE_EQ(0.30, local_limit(0.0, false));

  const double raw_scan_vx = 0.18;
  const double scale =
      navdog_runtime::proportionalMotionScale(raw_scan_vx, 0.0,
          local_limit(0.15, true));
  EXPECT_DOUBLE_EQ(1.0, scale);
  EXPECT_DOUBLE_EQ(0.18, raw_scan_vx * scale);
}

TEST(CmdSpeedLimit, MotionScalePreservesCurvatureWhenLimited)
{
  const double scale =
      navdog_runtime::proportionalMotionScale(0.45, 0.0, 0.30);
  EXPECT_NEAR(0.30 / 0.45, scale, 1e-9);

  const double limited_vx = 0.45 * scale;
  const double limited_w = 0.60 * scale;
  EXPECT_NEAR(0.30, limited_vx, 1e-9);
  EXPECT_NEAR(0.40, limited_w, 1e-9);
  EXPECT_NEAR(0.60 / 0.45, limited_w / limited_vx, 1e-9);
}

TEST(CmdSpeedLimit, MotionScaleKeepsPureRotation)
{
  EXPECT_DOUBLE_EQ(1.0,
      navdog_runtime::proportionalMotionScale(0.0, 0.0, 0.30));
}
