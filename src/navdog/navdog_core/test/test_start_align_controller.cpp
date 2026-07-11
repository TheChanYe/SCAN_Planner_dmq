#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/start_align_controller.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helpers
// =============================================================================

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

NavigationTask makeSimpleTask(double x, double y)
{
  NavigationTask task{};
  task.sequence = 1u;
  RoutePoint p{};
  p.x = x;
  p.y = y;
  task.points.push_back(p);
  return task;
}

NavigationTask makeTwoPointTask(
    double x0, double y0,
    double x1, double y1)
{
  NavigationTask task{};
  task.sequence = 1u;
  RoutePoint p0{};
  p0.x = x0;
  p0.y = y0;
  RoutePoint p1{};
  p1.x = x1;
  p1.y = y1;
  task.points.push_back(p0);
  task.points.push_back(p1);
  return task;
}

RobotState makeRobot(double x, double y, double yaw)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.yaw = yaw;
  robot.valid = true;
  return robot;
}

// =============================================================================
// 25.1 DefaultStateIsInactive
// =============================================================================

TEST(StartAlignControllerTest, DefaultStateIsInactive)
{
  StartAlignController controller;
  EXPECT_FALSE(controller.active());
}

// =============================================================================
// 25.2 InvalidTimeIsRejected
// =============================================================================

TEST(StartAlignControllerTest, InvalidTimeIsRejected)
{
  StartAlignController controller;

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 0);

  StartAlignOutput output = controller.update(
      task, robot, std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(output.result, StartAlignResult::INVALID_TIME);
  EXPECT_DOUBLE_EQ(output.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.command.yaw_rate, 0.0);
  EXPECT_FALSE(controller.active());
}

// =============================================================================
// 25.3 InvalidConfigIsRejected
// =============================================================================

TEST(StartAlignControllerTest, InvalidConfigIsRejected)
{
  {
    StartAlignConfig config{};
    config.max_yaw_rate = 0.0;
    StartAlignController controller(config);

    NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
    RobotState robot = makeRobot(0, 0, 0);

    StartAlignOutput output = controller.update(task, robot, 1.0);
    EXPECT_EQ(output.result, StartAlignResult::INVALID_CONFIG);
  }
  {
    StartAlignConfig config{};
    config.kp_yaw = std::numeric_limits<double>::quiet_NaN();
    StartAlignController controller(config);

    NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
    RobotState robot = makeRobot(0, 0, 0);

    StartAlignOutput output = controller.update(task, robot, 1.0);
    EXPECT_EQ(output.result, StartAlignResult::INVALID_CONFIG);
  }
  {
    StartAlignConfig config{};
    config.max_hold_sec = -1.0;
    StartAlignController controller(config);

    NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
    RobotState robot = makeRobot(0, 0, 0);

    StartAlignOutput output = controller.update(task, robot, 1.0);
    EXPECT_EQ(output.result, StartAlignResult::INVALID_CONFIG);
  }
}

// =============================================================================
// 25.4 UsesFirstValidRouteSegment
// =============================================================================

TEST(StartAlignControllerTest, UsesFirstValidRouteSegment)
{
  StartAlignController controller;

  NavigationTask task{};
  task.sequence = 1u;

  RoutePoint p0{};
  p0.x = 0.0;
  p0.y = 0.0;
  RoutePoint p1{};
  p1.x = 0.0;
  p1.y = 0.0;  // duplicate
  RoutePoint p2{};
  p2.x = 1.0;
  p2.y = 0.0;

  task.points.push_back(p0);
  task.points.push_back(p1);
  task.points.push_back(p2);

  RobotState robot = makeRobot(0, 0, kPi / 2);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_TRUE(output.has_target);
  EXPECT_NEAR(output.target_yaw, 0.0, 1e-9);
}

// =============================================================================
// 25.5 UsesExplicitYawWhenNoValidSegment
// =============================================================================

TEST(StartAlignControllerTest, UsesExplicitYawWhenNoValidSegment)
{
  StartAlignController controller;

  NavigationTask task{};
  task.sequence = 1u;

  RoutePoint p{};
  p.x = 0.0;
  p.y = 0.0;
  p.has_yaw = true;
  p.yaw = kPi / 2.0;
  task.points.push_back(p);

  RobotState robot = makeRobot(0, 0, 0);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_TRUE(output.has_target);
  EXPECT_NEAR(output.target_yaw, kPi / 2.0, 1e-9);
}

// =============================================================================
// 25.6 UsesRobotToSingleTargetPoint
// =============================================================================

TEST(StartAlignControllerTest, UsesRobotToSingleTargetPoint)
{
  StartAlignController controller;

  NavigationTask task = makeSimpleTask(0.0, 1.0);

  RobotState robot = makeRobot(0, 0, 0);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_TRUE(output.has_target);
  EXPECT_NEAR(output.target_yaw, kPi / 2.0, 1e-9);
}

// =============================================================================
// 25.7 WaitsForRobotWhenDirectionNeedsRobot
// =============================================================================

TEST(StartAlignControllerTest, WaitsForRobotWhenDirectionNeedsRobot)
{
  StartAlignController controller;

  NavigationTask task = makeSimpleTask(0.0, 0.0);

  RobotState robot{};
  robot.valid = false;

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::WAITING_FOR_ROBOT);
  EXPECT_DOUBLE_EQ(output.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.command.yaw_rate, 0.0);
}

// =============================================================================
// 25.8 InvalidTaskWhenDirectionCannotBeResolved
// =============================================================================

TEST(StartAlignControllerTest, InvalidTaskWhenDirectionCannotBeResolved)
{
  StartAlignController controller;

  // Single point at robot location, no yaw
  NavigationTask task = makeSimpleTask(0.0, 0.0);

  RobotState robot = makeRobot(0, 0, 0);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::INVALID_TASK);
}

// =============================================================================
// 25.9 SkipsAlignmentInsideEnterThreshold
// =============================================================================

TEST(StartAlignControllerTest, SkipsAlignmentInsideEnterThreshold)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  // Route east, robot yaw 10° off
  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 10.0 * kDeg);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNED);
  EXPECT_DOUBLE_EQ(output.command.yaw_rate, 0.0);
}

// =============================================================================
// 25.10 StartsAlignmentOutsideEnterThreshold
// =============================================================================

TEST(StartAlignControllerTest, StartsAlignmentOutsideEnterThreshold)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 30.0 * kDeg);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNING);
  EXPECT_LT(output.command.yaw_rate, 0.0);  // negative error → negative rate
}

// =============================================================================
// 25.11 NegativeErrorProducesNegativeYawRate
// =============================================================================

TEST(StartAlignControllerTest, NegativeErrorProducesNegativeYawRate)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  // target=0, robot=+30° → error = -30°
  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, -30.0 * kDeg);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNING);
  EXPECT_GT(output.command.yaw_rate, 0.0);
}

// =============================================================================
// 25.12 YawRateIsCapped
// =============================================================================

TEST(StartAlignControllerTest, YawRateIsCapped)
{
  StartAlignConfig config{};
  config.max_yaw_rate = 0.3;
  config.kp_yaw = 1.2;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  // Large error: robot at 170°
  RobotState robot = makeRobot(0, 0, 170.0 * kDeg);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNING);
  EXPECT_NEAR(
      std::fabs(output.command.yaw_rate),
      config.max_yaw_rate,
      1e-9);
}

// =============================================================================
// 25.13 ProportionalYawRateBelowCap
// =============================================================================

TEST(StartAlignControllerTest, ProportionalYawRateBelowCap)
{
  const double kDeg =
      3.14159265358979323846 / 180.0;

  StartAlignConfig config{};

  // 12 度必须触发旋转，不能被默认 15 度进入阈值判定为已对齐。
  config.enter_deg = 5.0;
  config.exit_deg = 2.0;

  config.kp_yaw = 1.0;
  config.max_yaw_rate = 1.0;
  config.max_hold_sec = 2.0;
  config.target_min_dist_m = 0.20;

  StartAlignController controller(config);

  NavigationTask task{};

  RoutePoint target{};
  target.x = 1.0;
  target.y = 0.0;
  target.z = 0.0;

  task.points.push_back(target);

  RobotState robot{};
  robot.x = 0.0;
  robot.y = 0.0;
  robot.yaw = 12.0 * kDeg;
  robot.valid = true;

  const StartAlignOutput output =
      controller.update(task, robot, 1.0);

  const double expected_error =
      -12.0 * kDeg;

  EXPECT_EQ(
      output.result,
      StartAlignResult::ALIGNING);

  EXPECT_NEAR(
      output.yaw_error,
      expected_error,
      1e-6);

  EXPECT_NEAR(
      output.command.yaw_rate,
      expected_error,
      1e-6);

  EXPECT_DOUBLE_EQ(output.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vy, 0.0);
}


// =============================================================================
// 25.14 AlignmentAlwaysKeepsLinearVelocityZero
// =============================================================================

TEST(StartAlignControllerTest, AlignmentAlwaysKeepsLinearVelocityZero)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 45.0 * kDeg);

  StartAlignOutput output = controller.update(task, robot, 1.0);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNING);
  EXPECT_DOUBLE_EQ(output.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vy, 0.0);
}

// =============================================================================
// 25.15 UsesShortestAngleAcrossPi
// =============================================================================

TEST(StartAlignControllerTest, UsesShortestAngleAcrossPi)
{
  const double kDeg =
      3.14159265358979323846 / 180.0;

  StartAlignConfig config{};

  // 2 度误差需要进入旋转，避免直接判定 ALIGNED。
  config.enter_deg = 0.5;
  config.exit_deg = 0.25;

  config.kp_yaw = 1.0;
  config.max_yaw_rate = 1.0;
  config.max_hold_sec = 2.0;
  config.target_min_dist_m = 0.20;

  StartAlignController controller(config);

  NavigationTask task{};

  // 使用单个带显式 yaw 的路点，确保不会被路线段方向优先覆盖。
  RoutePoint point{};
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  point.has_yaw = true;
  point.yaw = -179.0 * kDeg;

  task.points.push_back(point);

  RobotState robot{};
  robot.x = 0.0;
  robot.y = 0.0;
  robot.yaw = 179.0 * kDeg;
  robot.valid = true;

  const StartAlignOutput output =
      controller.update(task, robot, 1.0);

  EXPECT_EQ(
      output.result,
      StartAlignResult::ALIGNING);

  // 从 +179° 到 -179° 的最短误差应为正 2°。
  EXPECT_NEAR(
      output.yaw_error,
      2.0 * kDeg,
      1e-6);

  EXPECT_GT(output.command.yaw_rate, 0.0);
  EXPECT_LT(
      std::fabs(output.command.yaw_rate),
      5.0 * kDeg);
}


// =============================================================================
// 25.16 HysteresisRequiresExitThreshold
// =============================================================================

TEST(StartAlignControllerTest, HysteresisRequiresExitThreshold)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);

  // First: 20° error → ALIGNING
  RobotState robot1 = makeRobot(0, 0, 20.0 * kDeg);
  StartAlignOutput out1 = controller.update(task, robot1, 1.0);
  EXPECT_EQ(out1.result, StartAlignResult::ALIGNING);

  // Second: 10° error → still ALIGNING (above exit=5°)
  RobotState robot2 = makeRobot(0, 0, 10.0 * kDeg);
  StartAlignOutput out2 = controller.update(task, robot2, 1.1);
  EXPECT_EQ(out2.result, StartAlignResult::ALIGNING);

  // Third: 4° error → ALIGNED (below exit=5°)
  RobotState robot3 = makeRobot(0, 0, 4.0 * kDeg);
  StartAlignOutput out3 = controller.update(task, robot3, 1.2);
  EXPECT_EQ(out3.result, StartAlignResult::ALIGNED);
}

// =============================================================================
// 25.17 DoesNotTimeoutAtExactBoundary
// =============================================================================

TEST(StartAlignControllerTest, DoesNotTimeoutAtExactBoundary)
{
  StartAlignConfig config{};
  config.max_hold_sec = 2.0;
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 30.0 * kDeg);

  controller.update(task, robot, 1.0);  // start at 1.0

  RobotState robot2 = makeRobot(0, 0, 30.0 * kDeg);
  StartAlignOutput output = controller.update(task, robot2, 3.0);

  EXPECT_NE(output.result, StartAlignResult::TIMED_OUT);
}

// =============================================================================
// 25.18 TimesOutAfterBoundary
// =============================================================================

TEST(StartAlignControllerTest, TimesOutAfterBoundary)
{
  StartAlignConfig config{};
  config.max_hold_sec = 2.0;
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);
  RobotState robot = makeRobot(0, 0, 30.0 * kDeg);

  controller.update(task, robot, 1.0);  // start at 1.0

  RobotState robot2 = makeRobot(0, 0, 30.0 * kDeg);
  StartAlignOutput output = controller.update(task, robot2, 3.001);

  EXPECT_EQ(output.result, StartAlignResult::TIMED_OUT);
  EXPECT_DOUBLE_EQ(output.command.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.command.vy, 0.0);
}

// =============================================================================
// 25.19 AlignedTakesPriorityOverTimeout
// =============================================================================

TEST(StartAlignControllerTest, AlignedTakesPriorityOverTimeout)
{
  StartAlignConfig config{};
  config.max_hold_sec = 2.0;
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);

  // Start aligning
  RobotState robot1 = makeRobot(0, 0, 30.0 * kDeg);
  controller.update(task, robot1, 1.0);

  // Past timeout but within exit threshold
  RobotState robot2 = makeRobot(0, 0, 3.0 * kDeg);
  StartAlignOutput output = controller.update(task, robot2, 3.001);

  EXPECT_EQ(output.result, StartAlignResult::ALIGNED);
}

// =============================================================================
// 25.20 ResetClearsAlignmentContext
// =============================================================================

TEST(StartAlignControllerTest, ResetClearsAlignmentContext)
{
  StartAlignConfig config{};
  config.enter_deg = 15.0;
  config.exit_deg = 5.0;
  StartAlignController controller(config);

  NavigationTask task = makeTwoPointTask(0, 0, 1, 0);

  // Start alignment with large error
  RobotState robot1 = makeRobot(0, 0, 30.0 * kDeg);
  StartAlignOutput out1 = controller.update(task, robot1, 1.0);
  EXPECT_EQ(out1.result, StartAlignResult::ALIGNING);
  EXPECT_TRUE(controller.active());

  controller.reset();
  EXPECT_FALSE(controller.active());

  // Next call should use enter threshold again
  RobotState robot2 = makeRobot(0, 0, 10.0 * kDeg);
  StartAlignOutput out2 = controller.update(task, robot2, 2.0);
  EXPECT_EQ(out2.result, StartAlignResult::ALIGNED);
}

}  // namespace
}  // namespace navdog

// =============================================================================
// main
// =============================================================================

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
