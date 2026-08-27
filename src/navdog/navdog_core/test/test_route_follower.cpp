#include <gtest/gtest.h>

#include "navdog_core/route_follower.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace navdog
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

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
  progress.on_route = true;
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

NavigationTask makeNoisyStraightTask()
{
  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  const double xy[][2] = {
      {0.0, 0.00},
      {0.2, 0.02},
      {0.4, -0.02},
      {0.6, 0.01},
      {0.8, 0.00},
      {1.0, 0.02},
      {1.2, 0.00},
  };
  for (const auto& p : xy)
  {
    RoutePoint point{};
    point.x = p[0];
    point.y = p[1];
    task.points.push_back(point);
  }
  return task;
}

NavigationTask makePolylineTask(
    const double (&xy)[4][2])
{
  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  for (const auto& p : xy)
  {
    RoutePoint point{};
    point.x = p[0];
    point.y = p[1];
    task.points.push_back(point);
  }
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

TEST(RouteFollowerTest, DirectGoalNearBoundaryStillMoves)
{
  RouteFollowerConfig config{};
  config.kp_x = 0.8;
  config.max_vx = 0.70;
  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RouteProgress progress = makeProgress(1, 9.0, 1.0);

  const VelocityCommand cmd = follower.updateDirectGoal(
      task, makeRobot(9.79), progress, 0.30, 1.0);

  ASSERT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.06);
  EXPECT_NEAR(cmd.vx, 0.168, 1e-9);
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

TEST(RouteFollowerTest, RearTargetRecoveryTurnsTowardActualPoint)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.kp_yaw = 1.2;
  config.max_yaw_rate = 0.65;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 1.0;
  p2.x = 2.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 1.40, 0.60);
  progress.total_length_m = 2.0;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(2.30, 0.0, 0.0), progress, 0.50, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NEAR(std::abs(cmd.yaw_rate), config.max_yaw_rate, 1e-9);
}

TEST(RouteFollowerTest, NearOppositeRearTargetKeepsEffectiveTurnRate)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.kp_yaw = 1.2;
  config.max_yaw_rate = 0.65;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.70;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 1.0;
  p2.x = 2.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 1.40, 0.60);
  progress.total_length_m = 2.0;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(2.30, 0.01, kPi / 3.0),
      progress, 0.50, 1.0);

  ASSERT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(std::abs(cmd.yaw_rate), 0.20);
  EXPECT_GT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, RearBoundaryKeepsTurnDirectionContinuous)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.kp_yaw = 1.2;
  config.max_yaw_rate = 0.65;

  NavigationTask task{};
  task.sequence = 1;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 1.0;
  p2.x = 2.0;
  task.points = {p0, p1, p2};
  RouteProgress progress = makeProgress(1, 1.40, 0.60);
  progress.total_length_m = 2.0;

  RouteFollower follower(config);
  const VelocityCommand front = follower.update(
      task, makeRobot(1.95, -0.40, 0.0), progress, 0.50, 1.0);
  const VelocityCommand boundary = follower.update(
      task, makeRobot(2.00, -0.40, 0.0), progress, 0.50, 1.1);
  const VelocityCommand rear = follower.update(
      task, makeRobot(2.05, -0.40, 0.0), progress, 0.50, 1.2);

  ASSERT_TRUE(front.valid);
  ASSERT_TRUE(boundary.valid);
  ASSERT_TRUE(rear.valid);
  EXPECT_GT(front.yaw_rate, 0.0);
  EXPECT_GT(boundary.yaw_rate, 0.0);
  EXPECT_GT(rear.yaw_rate, 0.0);
  EXPECT_GT(front.vx, 0.0);
  EXPECT_DOUBLE_EQ(boundary.vx, 0.0);
  EXPECT_DOUBLE_EQ(rear.vx, 0.0);
  EXPECT_DOUBLE_EQ(front.vy, 0.0);
  EXPECT_DOUBLE_EQ(boundary.vy, 0.0);
  EXPECT_DOUBLE_EQ(rear.vy, 0.0);
  EXPECT_LT(std::abs(boundary.yaw_rate - front.yaw_rate), 0.20);
  EXPECT_LT(std::abs(rear.yaw_rate - boundary.yaw_rate), 0.20);
}

TEST(RouteFollowerTest, OnRouteDiagnosticDoesNotChangeControl)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  RouteFollower follower(config);
  RouteProgress on_route = makeProgress(1, 0.40, 2.60);
  RouteProgress off_route = on_route;
  off_route.on_route = false;

  const VelocityCommand on_route_cmd = follower.update(
      makeCornerTask(2.0), makeRobot(0.40, 0.0, 0.0),
      on_route, 0.50, 1.0);
  const VelocityCommand off_route_cmd = follower.update(
      makeCornerTask(2.0), makeRobot(0.40, 0.0, 0.0),
      off_route, 0.50, 1.1);

  ASSERT_TRUE(on_route_cmd.valid);
  ASSERT_TRUE(off_route_cmd.valid);
  EXPECT_DOUBLE_EQ(off_route_cmd.vx, on_route_cmd.vx);
  EXPECT_DOUBLE_EQ(off_route_cmd.vy, on_route_cmd.vy);
  EXPECT_DOUBLE_EQ(off_route_cmd.yaw_rate, on_route_cmd.yaw_rate);
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

TEST(RouteFollowerTest, SimplifiesSmallZigzagIntoStraightTrackingPath)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      makeNoisyStraightTask(), makeRobot(), makeProgress(1, 0.0, 1.0),
      0.50, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NEAR(cmd.yaw_rate, 0.0, 1e-9);
}

TEST(RouteFollowerTest, LateralOffsetSteersTowardFuturePoint)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;

  RouteFollower follower(config);
  const RouteProgress progress = makeProgress(1, 0.5, 2.5);
  const VelocityCommand left_of_route = follower.update(
      makeStraightTask(1, 3.0), makeRobot(0.5, 0.30, 0.0),
      progress, 0.50, 1.0);
  const VelocityCommand right_of_route = follower.update(
      makeStraightTask(1, 3.0), makeRobot(0.5, -0.30, 0.0),
      progress, 0.50, 1.0);

  EXPECT_TRUE(left_of_route.valid);
  EXPECT_TRUE(right_of_route.valid);
  EXPECT_GT(left_of_route.vx, 0.0);
  EXPECT_GT(right_of_route.vx, 0.0);
  EXPECT_DOUBLE_EQ(left_of_route.vy, 0.0);
  EXPECT_DOUBLE_EQ(right_of_route.vy, 0.0);
  EXPECT_LT(left_of_route.yaw_rate, 0.0);
  EXPECT_GT(right_of_route.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, NoisyStraightRouteDoesNotFlipYawAcrossProgressSamples)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  RouteFollower follower(config);
  const NavigationTask task = makeNoisyStraightTask();
  const double progress_samples[] = {0.0, 0.2, 0.4, 0.6};
  for (double arc : progress_samples)
  {
    const VelocityCommand cmd = follower.update(
        task, makeRobot(arc, 0.0, 0.0), makeProgress(1, arc, 1.2 - arc),
        0.50, 1.0);
    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
    EXPECT_NEAR(cmd.yaw_rate, 0.0, 1e-9);
  }
}

TEST(RouteFollowerTest, PreservesRealNinetyDegreeCorner)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.40;
  config.max_lookahead_distance_m = 0.40;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  NavigationTask task{};
  task.sequence = 1;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 1.0;
  p2.x = 1.0;
  p2.y = 1.0;
  task.points = {p0, p1, p2};

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.8, 0.0, 0.0), makeProgress(1, 0.7, 1.3),
      0.50, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, NinetyDegreeApproachTurnsInProgressively)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  const double xy[4][2] = {
      {0.0, 0.0},
      {1.0, 0.0},
      {1.0, 1.0},
      {1.0, 2.0},
  };
  const NavigationTask task = makePolylineTask(xy);
  RouteFollower follower(config);

  const double samples[] = {0.2, 0.4, 0.6};
  double previous_yaw = -std::numeric_limits<double>::infinity();
  for (double x : samples)
  {
    const VelocityCommand cmd = follower.update(
        task, makeRobot(x, 0.0, 0.0), makeProgress(1, x, 3.0 - x),
        0.50, 1.0);
    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
    EXPECT_GE(cmd.yaw_rate, previous_yaw - 1e-9);
    previous_yaw = cmd.yaw_rate;
  }
  EXPECT_GT(previous_yaw, 0.0);
}

TEST(RouteFollowerTest, AfterCornerDoesNotImmediatelyCounterSteer)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.60;
  config.max_lookahead_distance_m = 0.60;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  const double xy[4][2] = {
      {0.0, 0.0},
      {1.0, 0.0},
      {1.0, 1.0},
      {1.0, 2.0},
  };
  const NavigationTask task = makePolylineTask(xy);
  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(1.0, 0.3, kPi / 2.0),
      makeProgress(1, 1.3, 1.7), 0.50, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, -0.05);
  EXPECT_NEAR(cmd.yaw_rate, 0.0, 0.05);
}

TEST(RouteFollowerTest, TrackingSimplificationKeepsFinalWaypoint)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 2.0;
  config.max_lookahead_distance_m = 2.0;
  config.lookahead_time_sec = 0.0;
  config.simplify_tolerance_m = 0.05;

  NavigationTask task = makeNoisyStraightTask();
  task.points.back().x = 1.0;
  task.points.back().y = 0.4;

  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.8, 0.0, 0.0), makeProgress(1, 0.9, 0.5),
      0.50, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
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
