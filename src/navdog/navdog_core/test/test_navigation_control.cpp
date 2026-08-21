#include <gtest/gtest.h>

#include "navdog_core/goal_controller.hpp"
#include "navdog_core/route_follower.hpp"
#include "navdog_core/safety_supervisor.hpp"
#include "navdog_core/trajectory_follower.hpp"

#include <cmath>
#include <limits>
#include <memory>

namespace navdog
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
// =============================================================================
// Helpers
// =============================================================================

NavigationTask makeStraightTask(
    std::uint64_t sequence = 1,
    double length = 10.0)
{
  NavigationTask task{};
  task.sequence = sequence;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;

  RoutePoint p0{};
  p0.x = 0.0;
  p0.y = 0.0;
  p0.z = 0.0;
  task.points.push_back(p0);

  RoutePoint p1{};
  p1.x = length;
  p1.y = 0.0;
  p1.z = 0.0;
  task.points.push_back(p1);

  return task;
}

RobotState makeRobot(
    double x = 0.0,
    double y = 0.0,
    double yaw = 0.0,
    double stamp_sec = 0.0)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.z = 0.0;
  robot.yaw = yaw;
  robot.stamp_sec = stamp_sec;
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

LocalTrajectory makeStraightTrajectory(
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence,
    NavigationMode purpose,
    double start_time,
    double duration,
    double start_x,
    double end_x)
{
  LocalTrajectory trajectory{};
  trajectory.task_sequence = task_sequence;
  trajectory.plan_sequence = plan_sequence;
  trajectory.purpose = purpose;
  trajectory.source_stamp_sec = start_time;
  trajectory.duration_sec = duration;
  trajectory.valid = true;

  const std::size_t n = 10;
  for (std::size_t i = 0; i <= n; ++i)
  {
    const double ratio = static_cast<double>(i) / n;
    TimedTrajectoryPoint p{};
    p.time_from_start_sec = ratio * duration;
    p.x = start_x + ratio * (end_x - start_x);
    p.y = 0.0;
    p.z = 0.0;
    p.vx = (end_x - start_x) / duration;
    p.vy = 0.0;
    p.yaw = 0.0;
    p.has_yaw = true;
    trajectory.points.push_back(p);
  }

  return trajectory;
}

class FakeOccupancyQuery : public OccupancyQuery3D
{
public:
  bool ready() const noexcept override
  {
    return true;
  }

  bool isFree(
      double /*x*/,
      double /*y*/,
      double /*z*/,
      double /*yaw*/) const noexcept override
  {
    return free_;
  }

  void setFree(bool free_flag) noexcept
  {
    free_ = free_flag;
  }

private:
  bool free_{true};
};

// =============================================================================
// RouteFollower tests
// =============================================================================

TEST(RouteFollowerTest, OutputsNonZeroVelocity)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_yaw = 1.2;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd =
      follower.update(task, robot, progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NEAR(cmd.yaw_rate, 0.0, 1e-12);
  EXPECT_EQ(cmd.source, CommandSource::PLANNER);
}

TEST(RouteFollowerTest, FollowsSinglePointGoal)
{
  RouteFollowerConfig config{};
  RouteFollower follower(config);

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.4;
  RoutePoint target{};
  target.x = 5.0;
  task.points.push_back(target);

  RouteProgress progress = makeProgress(1, 0.0, 0.0, 0.0);
  progress.total_length_m = 0.0;
  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.0, 0.0, 0.0), progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.source, CommandSource::PLANNER);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
}

TEST(RouteFollowerTest, FollowsRepeatedPointGoal)
{
  RouteFollowerConfig config{};
  RouteFollower follower(config);

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.4;
  RoutePoint target{};
  target.x = 5.0;
  target.y = 5.0;
  task.points = {target, target, target};

  RouteProgress progress = makeProgress(1, 0.0, 0.0, 0.0);
  progress.total_length_m = 0.0;
  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.0, 0.0, 0.0), progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.source, CommandSource::PLANNER);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, RotatesWhenLookaheadBehindRobot)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_yaw = 1.2;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, 2.4);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd =
      follower.update(task, robot, progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, DrivesAndTurnsForForwardHalfPlaneTarget)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_yaw = 1.2;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.0, 0.0, -0.79), progress, 0.5, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, FollowsLookaheadAcrossWaypointWithoutStopping)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.4;
  config.max_vx = 0.5;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.4;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 0.8;
  p2.x = 0.8;
  p2.y = 1.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 0.7, 1.3, 0.0);
  progress.projected_x = 0.7;
  const RobotState robot = makeRobot(0.7, 0.0, 0.0);

  RouteFollower follower(config);
  const VelocityCommand cmd =
      follower.update(task, robot, progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, CosineHeadingScaleSlowsForwardHalfPlaneTarget)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.max_lookahead_distance_m = 1.0;
  config.max_vx = 0.5;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand aligned = follower.update(
      task, makeRobot(0.0, 0.0, 0.0), progress, 0.5, 1.0);
  const VelocityCommand turning = follower.update(
      task, makeRobot(0.0, 0.0, -0.5), progress, 0.5, 1.0);

  EXPECT_GT(aligned.vx, turning.vx);
  EXPECT_GT(turning.vx, 0.0);
  EXPECT_DOUBLE_EQ(turning.vy, 0.0);
}

TEST(RouteFollowerTest, TurnsOnlyWhenLookaheadBehindRobot)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.max_lookahead_distance_m = 1.0;
  config.max_vx = 0.5;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.0, 0.0, 2.4), progress, 0.5, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(RouteFollowerTest, EffectiveSpeedExtendsLookaheadAcrossCorner)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 0.4;
  config.max_lookahead_distance_m = 1.0;
  config.lookahead_time_sec = 1.0;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.5;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 0.8;
  p2.x = 0.8;
  p2.y = 2.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 0.2, 2.8, 0.0);
  RobotState stopped = makeRobot(0.2, 0.0, 0.0);

  RouteFollower follower(config);
  const VelocityCommand slow_cmd = follower.update(
      task, stopped, progress, 0.0, 1.0);
  const VelocityCommand route_cmd = follower.update(
      task, stopped, progress, 0.5, 1.0);

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

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.5;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 0.8;
  p2.x = 0.8;
  p2.y = -2.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 0.2, 2.8, 0.0);
  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, makeRobot(0.2, 0.0, 0.0), progress, 0.5, 1.0);

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

  const auto corner_task = [](double y) {
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
  };
  const auto diagonal_task = [](double y) {
    NavigationTask task{};
    task.sequence = 1;
    task.max_vx = 0.70;
    RoutePoint p0{};
    RoutePoint p1{};
    p1.x = 10.0;
    p1.y = y;
    task.points = {p0, p1};
    return task;
  };

  RouteFollower follower(config);
  const RouteProgress progress = makeProgress(1, 0.2, 2.8, 0.0);
  const RobotState robot = makeRobot(0.2, 0.0, 0.0);

  const VelocityCommand straight =
      follower.update(makeStraightTask(1, 10.0), robot, progress, 0.70, 1.0);
  const VelocityCommand mild =
      follower.update(diagonal_task(3.0), robot, progress, 0.70, 1.0);
  const VelocityCommand sharp =
      follower.update(corner_task(2.0), robot, progress, 0.70, 1.0);
  const VelocityCommand right =
      follower.update(corner_task(-2.0), robot, progress, 0.70, 1.0);

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

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.5;
  RoutePoint p0{};
  RoutePoint p1{};
  RoutePoint p2{};
  p1.x = 0.8;
  p2.x = 0.8;
  p2.y = 2.0;
  task.points = {p0, p1, p2};

  RouteProgress progress = makeProgress(1, 0.0, 3.0, 0.0);
  RobotState robot = makeRobot(0.0, 0.0, 0.0);
  robot.vx = 0.0;
  robot.vy = 0.0;
  RouteFollower follower(config);
  const VelocityCommand cmd = follower.update(
      task, robot, progress, 0.30, 1.0);

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
        3.0 - sample[0] - sample[1], 0.0);
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

TEST(RouteFollowerTest, RejectsProgressRegression)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);

  RouteProgress progress = makeProgress(1, 2.0, 8.0, 0.0);
  progress.projected_x = 2.0;
  progress.projected_y = 0.0;

  RobotState robot = makeRobot(2.0, 0.0, 0.0);
  const VelocityCommand cmd1 =
      follower.update(task, robot, progress, 0.4, 1.0);
  EXPECT_GT(cmd1.vx, 0.0);

  // Negative arc length is rejected.
  progress.arc_length_m = -1.0;
  const VelocityCommand cmd2 =
      follower.update(task, robot, progress, 0.4, 1.0);
  EXPECT_FALSE(cmd2.valid);
}

// =============================================================================
// TrajectoryFollower tests
// =============================================================================

TEST(TrajectoryFollowerTest, AcceptsMatchingTrajectory)
{
  TrajectoryFollowerConfig config{};
  config.time_forward_sec = 0.1;
  config.kp_pos = 0.5;
  config.kp_yaw = 1.0;

  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, StopsWhenNoTrajectory)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const LocalTrajectory trajectory{};

  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      1.0);

  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, RejectsOldTaskSequence)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      2,  // different task sequence
      1.0);

  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, RejectsOldMode)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::ROUTE_FOLLOW,  // expected mode mismatch
      1,
      1.0);

  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, AcceptedTrajectoryRunsBeyondPointThreeSeconds)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 1.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      2.5);  // elapsed 1.5s > duration 1.0s

  EXPECT_TRUE(cmd.valid);
}

TEST(TrajectoryFollowerTest, NinetyDegreeErrorHasEffectivelyZeroTranslation)
{
  TrajectoryFollowerConfig config{};
  config.kp_yaw = 1.0;

  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, kPi / 2.0);
  const VelocityCommand cmd = follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-12);
  EXPECT_NEAR(cmd.vy, 0.0, 1e-12);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(TrajectoryFollowerTest, ModerateHeadingErrorKeepsTranslating)
{
  TrajectoryFollower follower(TrajectoryFollowerConfig{});
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);
  const RobotState robot = makeRobot(0.0, 0.0, 0.3);

  const VelocityCommand cmd = follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

TEST(TrajectoryFollowerTest, TranslationDecreasesContinuouslyWithHeadingError)
{
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const auto velocity_at_yaw = [&trajectory](double yaw) {
    TrajectoryFollower follower(TrajectoryFollowerConfig{});
    return follower.update(
        trajectory, makeRobot(0.0, 0.0, yaw), 0.4, 0.35, 0.65,
        NavigationMode::LOCAL_AVOID, 1, 1.0).vx;
  };

  const double aligned_vx = velocity_at_yaw(0.0);
  const double moderate_vx = velocity_at_yaw(0.3);
  const double larger_vx = velocity_at_yaw(0.6);
  EXPECT_GT(aligned_vx, moderate_vx);
  EXPECT_GT(moderate_vx, larger_vx);
  EXPECT_GT(larger_vx, 0.0);
}

// =============================================================================
// GoalController tests
// =============================================================================

TEST(GoalControllerTest, GoalControllerDoesNotDriveDirectChord)
{
  GoalControllerConfig config{};
  config.near_goal_switch_dist = 0.8;
  config.near_goal_kp_v = 0.3;
  config.near_goal_min_v = 0.10;
  config.near_goal_max_v = 0.25;

  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(9.5, 0.0, 0.0);
  const RouteProgress progress = makeProgress(1, 9.5, 0.5, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_TRUE(result.command.valid);
  EXPECT_EQ(result.command.vx, 0.0);
  EXPECT_EQ(result.command.vy, 0.0);
}

TEST(GoalControllerTest, AlignsYawInPlace)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.finish_yaw_tolerance_deg = 5.0;
  config.finish_yaw_tolerance_rad =
      5.0 * kPi / 180.0;

  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, kPi / 4.0);
  const RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_TRUE(result.command.valid);
  EXPECT_EQ(result.command.vx, 0.0);
  EXPECT_EQ(result.command.vy, 0.0);
  EXPECT_NE(result.command.yaw_rate, 0.0);
}

TEST(GoalControllerTest, FinishesWithZeroSpeed)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.finish_yaw_tolerance_deg = 5.0;
  config.finish_yaw_tolerance_rad =
      5.0 * kPi / 180.0;

  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, 0.0);
  const RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_TRUE(result.finished);
  EXPECT_EQ(result.command.vx, 0.0);
  EXPECT_EQ(result.command.vy, 0.0);
  EXPECT_EQ(result.command.yaw_rate, 0.0);
}

// =============================================================================
// SafetySupervisor tests
// =============================================================================

TEST(SafetySupervisorTest, EmergencyDistanceStops)
{
  NavdogConfig config{};
  config.safety.emergency_stop = 0.45;
  config.safety.slow_down_front = 1.5;

  SafetySupervisor supervisor(
      config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 0.4;
  raw_cmd.vy = 0.0;
  raw_cmd.yaw_rate = 0.0;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.front_min = 0.3;

  supervisor.apply(raw_cmd, context, 0.4, 0.9);
  context.robot.stamp_sec = 1.0;
  context.obstacles.stamp_sec = 1.0;
  context.map_stamp_sec = 1.0;
  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_STOP);
}

TEST(SafetySupervisorTest, FrontObstacleLinearSlowdown)
{
  NavdogConfig config{};
  config.safety.emergency_stop = 0.45;
  config.safety.slow_down_front = 1.5;

  SafetySupervisor supervisor(
      config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 0.4;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = 0.8;
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  supervisor.apply(raw_cmd, context, 0.4, 0.9);
  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_LT(cmd.vx, raw_cmd.vx);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_SLOW);
}

TEST(SafetySupervisorTest, DynamicMaxVxLimit)
{
  NavdogConfig config{};
  config.limits.max_vx = 1.0;
  config.limits.max_vy = 0.35;
  config.limits.max_yaw_rate = 0.65;

  SafetySupervisor supervisor(
      config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 1.5;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  supervisor.apply(raw_cmd, context, 0.4, 1.0);
  context.robot.stamp_sec = 1.1;
  context.obstacles.stamp_sec = 1.1;
  context.map_stamp_sec = 1.1;
  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.1);

  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_LE(cmd.vx, 0.4);
}

TEST(SafetySupervisorTest, NaNCommandStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(
      config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = std::numeric_limits<double>::quiet_NaN();
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);

  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_STOP);
}

TEST(SafetySupervisorTest, InvalidRawCommandStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 0.4;
  raw_cmd.valid = false;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_STOP);
}

TEST(SafetySupervisorTest, InvalidMapStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 0.4;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.map_valid = false;

  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_STOP);
}

TEST(SafetySupervisorTest, AccelerationRiseIsLimited)
{
  NavdogConfig config{};
  config.limits.max_accel_x = 0.5;

  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 1.0;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  VelocityCommand first =
      supervisor.apply(raw_cmd, context, 1.0, 1.0);
  EXPECT_EQ(first.vx, 0.0);

  raw_cmd.vx = 1.0;
  VelocityCommand second =
      supervisor.apply(raw_cmd, context, 1.0, 1.02);
  EXPECT_GT(second.vx, first.vx);
  EXPECT_LE(second.vx - first.vx,
            config.limits.max_accel_x * 0.02 + 1e-9);
}

TEST(SafetySupervisorTest, FirstCommandRampsFromZero)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.0).vx, 0.0);
  EXPECT_GT(supervisor.apply(raw, context, 0.4, 1.1).vx, 0.0);
}

TEST(SafetySupervisorTest, NaNOdomStampStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot();
  context.robot.stamp_sec = std::numeric_limits<double>::quiet_NaN();
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.0).vx, 0.0);
}

TEST(SafetySupervisorTest, NaNObstacleStampStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = std::numeric_limits<double>::quiet_NaN();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.0).vx, 0.0);
}

TEST(SafetySupervisorTest, NaNMapStampStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.map_valid = true;
  context.map_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.0).vx, 0.0);
}

TEST(SafetySupervisorTest, InvalidObstacleSummaryStops)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.0).vx, 0.0);
}

TEST(SafetySupervisorTest, SafetyStopResetsPreviousVelocity)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  VelocityCommand raw{};
  raw.vx = 0.4;
  raw.valid = true;
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  supervisor.apply(raw, context, 0.4, 1.0);
  supervisor.apply(raw, context, 0.4, 1.2);
  context.obstacles.valid = false;
  EXPECT_EQ(supervisor.apply(raw, context, 0.4, 1.3).vx, 0.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.4;
  context.map_stamp_sec = 1.4;
  const VelocityCommand recovered =
      supervisor.apply(raw, context, 0.4, 1.4);
  EXPECT_GT(recovered.vx, 0.0);
  EXPECT_LE(recovered.vx, config.limits.max_accel_x * 0.1 + 1e-9);
}

TEST(SafetySupervisorTest, LateralAccelerationIsLimited)
{
  NavdogConfig config{};
  config.limits.max_accel_y = 0.4;

  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vy = 0.3;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  VelocityCommand first =
      supervisor.apply(raw_cmd, context, 1.0, 1.0);

  raw_cmd.vy = -0.3;
  context.robot.stamp_sec = 1.05;
  context.obstacles.stamp_sec = 1.05;
  context.map_stamp_sec = 1.05;
  VelocityCommand second =
      supervisor.apply(raw_cmd, context, 1.0, 1.05);

  EXPECT_NE(second.vy, 0.0);
  EXPECT_LE(std::abs(second.vy - first.vy),
            config.limits.max_accel_y * 0.05 + 1e-9);
}

TEST(SafetySupervisorTest, YawAccelerationIsLimited)
{
  NavdogConfig config{};
  config.limits.max_accel_yaw = 0.8;

  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.yaw_rate = 0.5;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  VelocityCommand first =
      supervisor.apply(raw_cmd, context, 1.0, 1.0);

  raw_cmd.yaw_rate = -0.5;
  context.robot.stamp_sec = 1.05;
  context.obstacles.stamp_sec = 1.05;
  context.map_stamp_sec = 1.05;
  VelocityCommand second =
      supervisor.apply(raw_cmd, context, 1.0, 1.05);

  EXPECT_NE(second.yaw_rate, 0.0);
  EXPECT_LE(std::abs(second.yaw_rate - first.yaw_rate),
            config.limits.max_accel_yaw * 0.05 + 1e-9);
}

TEST(SafetySupervisorTest, TrackingStopBypassesAccelerationRamp)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  VelocityCommand moving{};
  moving.vx = 0.4;
  moving.valid = true;
  supervisor.apply(moving, context, 0.4, 1.0);
  context.robot.stamp_sec = 1.5;
  context.obstacles.stamp_sec = 1.5;
  context.map_stamp_sec = 1.5;
  supervisor.apply(moving, context, 0.4, 1.5);

  VelocityCommand stop{};
  stop.valid = true;
  stop.source = CommandSource::TRACKING_STOP;
  const VelocityCommand result = supervisor.apply(stop, context, 0.4, 1.5);
  EXPECT_DOUBLE_EQ(result.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_rate, 0.0);
  EXPECT_EQ(result.source, CommandSource::TRACKING_STOP);
}

TEST(SafetySupervisorTest, GoalAlignStopsLinearMotionImmediately)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  VelocityCommand moving{};
  moving.vx = 0.4;
  moving.valid = true;
  supervisor.apply(moving, context, 0.4, 1.0);
  context.robot.stamp_sec = 1.5;
  context.obstacles.stamp_sec = 1.5;
  context.map_stamp_sec = 1.5;
  supervisor.apply(moving, context, 0.4, 1.5);

  VelocityCommand align{};
  align.yaw_rate = 0.22;
  align.valid = true;
  align.source = CommandSource::GOAL_ALIGN;
  const VelocityCommand result = supervisor.apply(align, context, 0.4, 1.5);
  EXPECT_DOUBLE_EQ(result.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.vy, 0.0);
}

TEST(SafetySupervisorTest, GoalAlignYawStillUsesAccelerationLimit)
{
  NavdogConfig config{};
  config.limits.max_accel_yaw = 0.8;
  SafetySupervisor supervisor(config.safety, config.limits);
  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = std::numeric_limits<double>::infinity();
  context.map_valid = true;
  context.map_stamp_sec = 1.0;
  VelocityCommand align{};
  align.yaw_rate = 0.22;
  align.valid = true;
  align.source = CommandSource::GOAL_ALIGN;
  supervisor.apply(align, context, 0.4, 1.0);
  context.robot.stamp_sec = 1.1;
  context.obstacles.stamp_sec = 1.1;
  context.map_stamp_sec = 1.1;
  const VelocityCommand result = supervisor.apply(align, context, 0.4, 1.1);
  EXPECT_GT(result.yaw_rate, 0.0);
  EXPECT_LE(result.yaw_rate, config.limits.max_accel_yaw * 0.1 + 1e-9);
}

TEST(GoalControllerTest, GoalAlignRespectsNearGoalMaxW)
{
  GoalControllerConfig config{};
  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, kPi);
  const RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);
  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);
  EXPECT_LE(std::abs(result.command.yaw_rate), 0.22 + 1e-9);
}

TEST(SafetySupervisorTest, EmergencyStopIsImmediate)
{
  NavdogConfig config{};
  config.limits.max_accel_x = 0.5;
  config.safety.emergency_stop = 0.45;
  config.safety.slow_down_front = 1.5;

  SafetySupervisor supervisor(config.safety, config.limits);

  VelocityCommand raw_cmd{};
  raw_cmd.vx = 0.4;
  raw_cmd.valid = true;

  SafetySupervisor::Context context{};
  context.robot = makeRobot(0.0, 0.0, 0.0, 1.0);
  context.obstacles.valid = true;
  context.obstacles.stamp_sec = 1.0;
  context.obstacles.front_min = 0.8;
  context.map_valid = true;
  context.map_stamp_sec = 1.0;

  supervisor.apply(raw_cmd, context, 0.4, 0.9);
  VelocityCommand first =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);
  EXPECT_GT(first.vx, 0.0);

  context.obstacles.front_min = 0.3;
  VelocityCommand second =
      supervisor.apply(raw_cmd, context, 0.4, 1.02);
  EXPECT_EQ(second.vx, 0.0);
}

// =============================================================================
// TrajectoryFollower time tests
// =============================================================================

TEST(TrajectoryFollowerTimeTest, HeadingCorrectionAdvancesExecutionTime)
{
  TrajectoryFollowerConfig config{};
  config.kp_yaw = 1.0;

  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, kPi / 2.0);

  follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      1.0);

  const double t1 = follower.trajectoryTimeSec();

  follower.update(
      trajectory,
      robot,
      0.4,
      0.35,
      0.65,
      NavigationMode::LOCAL_AVOID,
      1,
      1.1);

  const double t2 = follower.trajectoryTimeSec();

  EXPECT_NEAR(t1, 0.0, 1e-9);
  EXPECT_NEAR(t2, 0.1, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, AlignedMotionAdvancesExecutionTime)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.1);

  EXPECT_NEAR(follower.trajectoryTimeSec(), 0.1, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, NewPlanSequenceRestartsAtZero)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);

  LocalTrajectory trajectory1 = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);
  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.2);

  EXPECT_GT(follower.trajectoryTimeSec(), 0.15);

  LocalTrajectory trajectory2 = makeStraightTrajectory(
      1, 2, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  follower.update(
      trajectory2, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.3);

  EXPECT_NEAR(follower.trajectoryTimeSec(), 0.0, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, PurposeChangeRestartsAtZero)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);

  LocalTrajectory trajectory1 = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);
  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.2);

  EXPECT_GT(follower.trajectoryTimeSec(), 0.15);

  LocalTrajectory trajectory2 = makeStraightTrajectory(
      1, 1, NavigationMode::ROUTE_FOLLOW, 1.0, 2.0, 0.0, 1.0);

  follower.update(
      trajectory2, robot, 0.4, 0.35, 0.65,
      NavigationMode::ROUTE_FOLLOW, 1, 1.3);

  EXPECT_NEAR(follower.trajectoryTimeSec(), 0.0, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, TaskChangeRestartsAtZero)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);

  LocalTrajectory trajectory1 = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);
  follower.update(
      trajectory1, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.2);

  EXPECT_GT(follower.trajectoryTimeSec(), 0.15);

  LocalTrajectory trajectory2 = makeStraightTrajectory(
      2, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  follower.update(
      trajectory2, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 2, 1.3);

  EXPECT_NEAR(follower.trajectoryTimeSec(), 0.0, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, LargeDtDoesNotSkipTrajectory)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.3);  // dt = 0.3 > 0.2

  EXPECT_NEAR(follower.trajectoryTimeSec(), 0.0, 1e-9);
}

TEST(TrajectoryFollowerTimeTest, TimeRegressionStopsTrajectory)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 2.0);

  const VelocityCommand cmd = follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.5);  // regression

  EXPECT_FALSE(cmd.valid);
}

TEST(TrajectoryFollowerTimeTest, TrajectoryRunsLongerThanPointThreeSeconds)
{
  TrajectoryFollowerConfig config{};
  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 2.0, 0.0, 2.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  VelocityCommand cmd = follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);

  cmd = follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.5);  // elapsed 0.5 < duration 2.0

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTimeTest, TrajectoryExpiresByExecutionTime)
{
  TrajectoryFollowerConfig config{};
  // Note: TrajectoryFollower itself does not use expiry margin directly.
  // It allows exec_time up to duration. The coordinator applies the margin.
  // This test verifies exec_time reaches duration with small control steps.

  TrajectoryFollower follower(config);
  const LocalTrajectory trajectory = makeStraightTrajectory(
      1, 1, NavigationMode::LOCAL_AVOID, 1.0, 1.0, 0.0, 1.0);

  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  follower.update(
      trajectory, robot, 0.4, 0.35, 0.65,
      NavigationMode::LOCAL_AVOID, 1, 1.0);

  // Advance in 0.2s steps so the large-dt guard does not freeze time.
  for (double t = 1.2; t <= 2.0 + 1e-9; t += 0.2)
  {
    follower.update(
        trajectory, robot, 0.4, 0.35, 0.65,
        NavigationMode::LOCAL_AVOID, 1, t);
  }

  EXPECT_GE(
      follower.trajectoryTimeSec(), 1.0 - 1e-9);
}

// =============================================================================
// GoalController tests
// =============================================================================

TEST(GoalControllerTest, GoalYawAlignOnlyAfterPositionReached)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.finish_yaw_tolerance_deg = 5.0;
  config.finish_yaw_tolerance_rad = 5.0 * kPi / 180.0;

  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, kPi / 4.0);
  RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_TRUE(result.command.valid);
  EXPECT_EQ(result.command.vx, 0.0);
  EXPECT_EQ(result.command.vy, 0.0);
  EXPECT_NE(result.command.yaw_rate, 0.0);
}

TEST(GoalControllerTest, RequestsRouteReacquisitionAfterPositionDrift)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.goal_align_reacquire_dist = 0.25;
  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(9.70, 0.0, kPi / 4.0);
  const RouteProgress progress = makeProgress(1, 9.70, 0.30, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_TRUE(result.position_lost);
  EXPECT_FALSE(result.finished);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.0);
}

TEST(GoalControllerTest, UsesEffectiveMinimumYawRate)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.finish_yaw_tolerance_rad = 0.01;
  config.goal_align_min_yaw_rate = 0.03;
  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, -0.02);
  const RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);

  EXPECT_FALSE(result.finished);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.03);
}

TEST(GoalControllerTest, FinishesWhenAlignmentTimesOut)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.15;
  config.finish_yaw_tolerance_rad = 0.01;
  config.goal_align_timeout_sec = 2.0;
  GoalController controller(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(10.0, 0.0, -0.2);
  const RouteProgress progress = makeProgress(1, 10.0, 0.0, 0.0);

  EXPECT_FALSE(controller.update(
      task, robot, progress, 0.4, 0.65, 1.0).finished);
  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 3.0);

  EXPECT_TRUE(result.finished);
  EXPECT_TRUE(result.timed_out);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.0);
}

}  // namespace
}  // namespace navdog

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
