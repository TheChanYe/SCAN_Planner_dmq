#include <gtest/gtest.h>

#include "navdog_core/route_follower.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace navdog
{
namespace
{

NavigationTask makeStraightTask(
    std::uint64_t sequence = 1,
    double length = 10.0)
{
  NavigationTask task{};
  task.sequence = sequence;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;

  RoutePoint p0{};
  task.points.push_back(p0);

  RoutePoint p1{};
  p1.x = length;
  task.points.push_back(p1);

  return task;
}

RobotState makeRobot(
    double x = 0.0,
    double y = 0.0,
    double yaw = 0.0)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.z = 0.0;
  robot.yaw = yaw;
  robot.valid = true;
  return robot;
}

RouteProgress makeProgress(
    std::uint64_t sequence,
    double arc_length,
    double remaining = 10.0,
    double route_yaw = 0.0)
{
  RouteProgress progress{};
  progress.task_sequence = sequence;
  progress.arc_length_m = arc_length;
  progress.remaining_distance_m = remaining;
  progress.total_length_m = arc_length + remaining;
  progress.route_yaw = route_yaw;
  progress.valid = true;
  return progress;
}

NavigationTask makeCornerTask(double y)
{
  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 0.8;
  p2.x = 0.8;
  p2.y = y;
  task.points = {p0, p1, p2};
  return task;
}

NavigationTask makeDiagonalTask(double y)
{
  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  RoutePoint p0{};
  RoutePoint p1{};
  p1.x = 10.0;
  p1.y = y;
  task.points = {p0, p1};
  return task;
}

}  // namespace

TEST(RouteFollowerTest, OutputsForwardOnlyStraightCommand)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_yaw = 1.2;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeStraightTask(), makeRobot(), makeProgress(1, 0.0), 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NEAR(cmd.yaw_rate, 0.0, 1e-12);
  EXPECT_EQ(cmd.source, CommandSource::PLANNER);
}

TEST(RouteFollowerTest, SinglePointGoalUsesForwardOnlyCommand)
{
  RouteFollower follower(RouteFollowerConfig{});
  NavigationTask task{};
  task.sequence = 1;
  RoutePoint target{};
  target.x = 5.0;
  task.points.push_back(target);

  RouteProgress progress = makeProgress(1, 0.0, 0.0);
  progress.total_length_m = 0.0;
  const VelocityCommand cmd =
      follower.update(task, makeRobot(), progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
}

TEST(RouteFollowerTest, LookaheadBehindRobotTurnsOnly)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.max_lookahead_distance_m = 1.0;
  config.max_vx = 0.5;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeStraightTask(), makeRobot(0.0, 0.0, 2.4),
      makeProgress(1, 0.0), 0.5, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, ForwardHalfPlaneDrivesAndTurnsContinuously)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_yaw = 1.2;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeStraightTask(), makeRobot(0.0, 0.0, -0.79),
      makeProgress(1, 0.0), 0.5, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, EffectiveSpeedExtendsLookaheadAcrossCorner)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.4;
  config.max_lookahead_distance_m = 1.0;
  config.lookahead_time_sec = 1.0;

  RouteFollower follower(config);
  const RouteProgress progress = makeProgress(1, 0.2, 2.8);
  const RobotState stopped = makeRobot(0.2, 0.0, 0.0);

  const VelocityCommand slow_cmd = follower.update(
      makeCornerTask(2.0), stopped, progress, 0.0, 1.0);
  const VelocityCommand route_cmd = follower.update(
      makeCornerTask(2.0), stopped, progress, 0.5, 1.0);

  EXPECT_NEAR(slow_cmd.yaw_rate, 0.0, 1e-12);
  EXPECT_GT(route_cmd.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(route_cmd.vy, 0.0);
}

TEST(RouteFollowerTest, RightTurnUsesNegativeYawWithoutLateralVelocity)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.4;
  config.max_lookahead_distance_m = 1.0;
  config.lookahead_time_sec = 1.0;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeCornerTask(-2.0), makeRobot(0.2, 0.0, 0.0),
      makeProgress(1, 0.2, 2.8), 0.5, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_LT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, CurvatureAwareSpeedKeepsYawExecutable)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.4;
  config.max_lookahead_distance_m = 1.0;
  config.lookahead_time_sec = 1.0;
  config.kp_x = 1.2;
  config.kp_yaw = 1.2;
  config.max_vx = 0.70;
  config.max_yaw_rate = 0.65;

  RouteFollower follower(config);
  const RouteProgress progress = makeProgress(1, 0.2, 2.8);
  const RobotState robot = makeRobot(0.2, 0.0, 0.0);

  const VelocityCommand straight =
      follower.update(makeStraightTask(), robot, progress, 0.70, 1.0);
  const VelocityCommand mild =
      follower.update(makeDiagonalTask(3.0), robot, progress, 0.70, 1.0);
  const VelocityCommand sharp =
      follower.update(makeCornerTask(2.0), robot, progress, 0.70, 1.0);
  const VelocityCommand right =
      follower.update(makeCornerTask(-2.0), robot, progress, 0.70, 1.0);

  EXPECT_TRUE(straight.valid);
  EXPECT_TRUE(mild.valid);
  EXPECT_TRUE(sharp.valid);
  EXPECT_TRUE(right.valid);
  EXPECT_DOUBLE_EQ(straight.vy, 0.0);
  EXPECT_DOUBLE_EQ(mild.vy, 0.0);
  EXPECT_DOUBLE_EQ(sharp.vy, 0.0);
  EXPECT_DOUBLE_EQ(right.vy, 0.0);
  EXPECT_GT(straight.vx, mild.vx);
  EXPECT_GT(mild.vx, sharp.vx);
  EXPECT_GT(mild.yaw_rate, 0.0);
  EXPECT_GT(sharp.yaw_rate, 0.0);
  EXPECT_LT(right.yaw_rate, 0.0);
  EXPECT_LE(std::abs(mild.yaw_rate), config.max_yaw_rate + 1e-9);
  EXPECT_LE(std::abs(sharp.yaw_rate), config.max_yaw_rate + 1e-9);
  EXPECT_LE(std::abs(right.yaw_rate), config.max_yaw_rate + 1e-9);
}

TEST(RouteFollowerTest, BlockedConfirmationUsesShorterSpeedHint)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 1.20;
  config.lookahead_time_sec = 1.20;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeCornerTask(2.0), makeRobot(), makeProgress(1, 0.0, 3.0),
      0.30, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
  EXPECT_LE(std::hypot(cmd.vx, cmd.vy), 0.30 + 1e-9);
}

TEST(RouteFollowerTest, LeftCornerYawDoesNotOscillateAcrossSamples)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 1.20;
  config.lookahead_time_sec = 1.20;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.5;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 1.0;
  p2.x = 1.0;
  p2.y = 2.0;
  task.points = {p0, p1, p2};

  RouteFollower follower(config);
  const double samples[][3] = {
      {0.20, 0.00, 0.00},
      {0.55, 0.00, 0.18},
      {0.85, 0.02, 0.55},
      {1.00, 0.22, 1.05},
      {1.00, 0.70, 1.45},
  };
  double previous_abs_yaw = std::numeric_limits<double>::infinity();
  bool saw_positive = false;
  for (const auto& sample : samples)
  {
    RouteProgress progress = makeProgress(1, sample[0] + sample[1],
        3.0 - sample[0] - sample[1]);
    const VelocityCommand cmd = follower.update(
        task, makeRobot(sample[0], sample[1], sample[2]),
        progress, 0.50, 1.0);
    EXPECT_TRUE(cmd.valid);
    EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
    EXPECT_GE(cmd.yaw_rate, -1e-6);
    if (cmd.yaw_rate > 1e-6)
      saw_positive = true;
    EXPECT_LE(std::fabs(cmd.yaw_rate), previous_abs_yaw + 0.25);
    previous_abs_yaw = std::fabs(cmd.yaw_rate);
  }
  EXPECT_TRUE(saw_positive);
}

TEST(RouteFollowerTest, RejectsInvalidProgress)
{
  RouteFollower follower(RouteFollowerConfig{});
  RouteProgress progress = makeProgress(1, -1.0, 8.0);
  const VelocityCommand cmd =
      follower.update(makeStraightTask(), makeRobot(), progress, 0.4, 1.0);

  EXPECT_FALSE(cmd.valid);
}

}  // namespace navdog
