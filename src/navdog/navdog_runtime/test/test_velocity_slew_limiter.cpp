#include "navdog_runtime/velocity_slew_limiter.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace navdog_runtime
{
namespace
{

constexpr double kTolerance = 1e-6;

TEST(VelocitySlewLimiterTest, ZeroToZeroPassThrough)
{
  VelocitySlewLimiter limiter;
  double vx, vy, yaw;
  limiter.update(0.0, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.0, kTolerance);
  EXPECT_NEAR(vy, 0.0, kTolerance);
  EXPECT_NEAR(yaw, 0.0, kTolerance);
}

TEST(VelocitySlewLimiterTest, AccelerationLimitRespected)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  double vx, vy, yaw;
  // dt=0.02s, target=0.50, max change = 0.50*0.02 = 0.01
  limiter.update(0.50, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.01, kTolerance);
}

TEST(VelocitySlewLimiterTest, DecelerationLimitRespected)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  // Set current to 0.50
  limiter.setCurrent(0.50, 0.0, 0.0);

  double vx, vy, yaw;
  // dt=0.02s, target=0.22, delta=-0.28, max change = 0.80*0.02 = 0.016
  limiter.update(0.22, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.50 - 0.016, kTolerance);
}

TEST(VelocitySlewLimiterTest, DirectionReversalDecelerates)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  limiter.setCurrent(0.20, 0.0, 0.0);

  double vx, vy, yaw;
  // target = -0.20, direction reversal: only decelerate toward zero.
  // max change = 0.80 * 0.02 = 0.016
  limiter.update(-0.20, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.20 - 0.016, kTolerance);
}

TEST(VelocitySlewLimiterTest, DirectionReversalCrossesZero)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  // Small positive current; one decel step is enough to reach zero.
  limiter.setCurrent(0.01, 0.0, 0.0);

  double vx, vy, yaw;
  limiter.update(-0.20, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.0, kTolerance);

  // Next cycle starts accelerating into the negative direction.
  limiter.update(-0.20, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, -0.01, kTolerance);
}

TEST(VelocitySlewLimiterTest, MultipleStepsConverge)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  double vx, vy, yaw;
  // Run 500 steps (10 seconds at 50Hz) from 0 to 0.50
  for (int i = 0; i < 500; ++i)
    limiter.update(0.50, 0.0, 0.0, 0.02, vx, vy, yaw);

  EXPECT_NEAR(vx, 0.50, 0.01);
}

TEST(VelocitySlewLimiterTest, ResetClearsState)
{
  VelocitySlewLimiter limiter;
  limiter.setCurrent(0.50, 0.30, 1.0);
  limiter.reset();

  double vx, vy, yaw;
  limiter.update(0.0, 0.0, 0.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.0, kTolerance);
  EXPECT_NEAR(vy, 0.0, kTolerance);
  EXPECT_NEAR(yaw, 0.0, kTolerance);
}

TEST(VelocitySlewLimiterTest, YawLimitRespected)
{
  VelocitySlewLimiter::Config config;
  config.accel_yaw = 0.80;
  config.decel_yaw = 1.20;
  VelocitySlewLimiter limiter(config);

  limiter.setCurrent(0.0, 0.0, 0.0);

  double vx, vy, yaw;
  // dt=0.02, target_yaw=1.0, max change = 0.80*0.02 = 0.016
  limiter.update(0.0, 0.0, 1.0, 0.02, vx, vy, yaw);
  EXPECT_NEAR(yaw, 0.016, kTolerance);
}

TEST(VelocitySlewLimiterTest, AbnormalDtKeepsCurrent)
{
  VelocitySlewLimiter limiter;
  limiter.setCurrent(0.10, 0.20, 0.30);

  double vx, vy, yaw;
  // dt > 0.2 — must keep previous velocity, not jump to target.
  limiter.update(0.50, 0.80, 1.0, 0.5, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.10, kTolerance);
  EXPECT_NEAR(vy, 0.20, kTolerance);
  EXPECT_NEAR(yaw, 0.30, kTolerance);

  // dt == 0 — same behaviour.
  limiter.update(0.50, 0.80, 1.0, 0.0, vx, vy, yaw);
  EXPECT_NEAR(vx, 0.10, kTolerance);
  EXPECT_NEAR(vy, 0.20, kTolerance);
  EXPECT_NEAR(yaw, 0.30, kTolerance);
}

TEST(VelocitySlewLimiterTest, RouteToScanSmoothDeceleration)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  // Simulate ROUTE at 0.50 m/s
  limiter.setCurrent(0.50, 0.0, 0.0);

  double vx, vy, yaw;
  // Switch to SCAN: target=0 (no scan cmd yet), decelerate over ~0.35s
  for (int i = 0; i < 18; ++i)  // 0.36s at 50Hz
    limiter.update(0.0, 0.0, 0.0, 0.02, vx, vy, yaw);

  // Should be close to 0 after ~0.36s with decel=0.80
  EXPECT_LT(std::abs(vx), 0.05);
}

TEST(VelocitySlewLimiterTest, ScanToRouteSmoothAcceleration)
{
  VelocitySlewLimiter::Config config;
  config.accel_x = 0.50;
  config.decel_x = 0.80;
  VelocitySlewLimiter limiter(config);

  // Simulate SCAN at 0.22 m/s
  limiter.setCurrent(0.22, 0.0, 0.0);

  double vx, vy, yaw;
  // Switch to ROUTE: target=0.50, accelerate over ~0.56s
  for (int i = 0; i < 28; ++i)  // 0.56s at 50Hz
    limiter.update(0.50, 0.0, 0.0, 0.02, vx, vy, yaw);

  // Should be close to 0.50 after ~0.56s with accel=0.50
  EXPECT_NEAR(vx, 0.50, 0.02);
}

}  // namespace
}  // namespace navdog_runtime

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
