#include "dmq_dog/driver_motion_mode.hpp"

#include <gtest/gtest.h>

TEST(DriverMotionModeTest, PreservesHolonomicCommandInLocalAvoid)
{
  EXPECT_FALSE(dmq_dog::forwardMotionAdapterEnabled(
      true, dmq_dog::kNavigationModeLocalAvoid));
}

TEST(DriverMotionModeTest, KeepsForwardAdapterForRouteFollow)
{
  EXPECT_TRUE(dmq_dog::forwardMotionAdapterEnabled(true, 1));
  EXPECT_FALSE(dmq_dog::forwardMotionAdapterEnabled(false, 1));
}
