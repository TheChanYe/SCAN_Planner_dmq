#include <gtest/gtest.h>

#include "navdog_core/goal_controller.hpp"
#include "navdog_core/rejoin_target_selector.hpp"
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
  config.kp_y = 1.0;
  config.kp_yaw = 1.2;
  config.heading_turn_only_threshold_rad = 0.8;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, 0.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd =
      follower.update(task, robot, progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_GT(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::PLANNER);
}

TEST(RouteFollowerTest, RotatesFirstWhenHeadingErrorLarge)
{
  RouteFollowerConfig config{};
  config.lookahead_distance_m = 1.0;
  config.kp_x = 0.8;
  config.kp_y = 1.0;
  config.kp_yaw = 1.2;
  config.heading_turn_only_threshold_rad = 0.8;
  config.max_vx = 0.8;

  RouteFollower follower(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, kPi / 2.0);
  const RouteProgress progress = makeProgress(1, 0.0, 10.0, 0.0);

  const VelocityCommand cmd =
      follower.update(task, robot, progress, 0.4, 1.0);

  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
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
  config.heading_turn_only_threshold_rad = 0.8;

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
      NavigationMode::ROUTE_REJOIN,  // expected mode mismatch
      1,
      1.0);

  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, StopsWhenExpired)
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

  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.vx, 0.0);
}

TEST(TrajectoryFollowerTest, RotatesOnlyWithLargeHeadingError)
{
  TrajectoryFollowerConfig config{};
  config.heading_turn_only_threshold_rad = 0.3;
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
  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.vy, 0.0);
  EXPECT_NE(cmd.yaw_rate, 0.0);
}

// =============================================================================
// RejoinTargetSelector tests
// =============================================================================

TEST(RejoinTargetSelectorTest, TargetNotBeforeAnchor)
{
  RejoinTargetSelectorConfig config{};
  config.default_forward_distance_m = 2.0;
  config.min_forward_distance_m = 1.0;
  config.max_forward_distance_m = 3.0;

  RejoinTargetSelector selector(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  RouteProgress progress = makeProgress(1, 5.0, 5.0, 0.0);

  NavigationModeStatus mode_status{};
  mode_status.has_rejoin_anchor = true;
  mode_status.rejoin_min_arc_length_m = 4.5;

  const auto result = selector.select(
      task, progress, mode_status, robot, nullptr);

  EXPECT_TRUE(result.valid);
  EXPECT_GE(result.target.x, 4.5);
}

TEST(RejoinTargetSelectorTest, RejectsBehindProgress)
{
  RejoinTargetSelectorConfig config{};
  RejoinTargetSelector selector(config);
  const NavigationTask task = makeStraightTask(1, 10.0);
  const RobotState robot = makeRobot(0.0, 0.0, 0.0);

  RouteProgress progress = makeProgress(1, 5.0, 5.0, 0.0);

  NavigationModeStatus mode_status{};
  mode_status.has_rejoin_anchor = true;
  mode_status.rejoin_min_arc_length_m = 2.0;

  const auto result = selector.select(
      task, progress, mode_status, robot, nullptr);

  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.target.x, 5.0);
}

// =============================================================================
// GoalController tests
// =============================================================================

TEST(GoalControllerTest, SlowsDownNearGoal)
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
  EXPECT_GT(result.command.vx, 0.0);
  EXPECT_LE(result.command.vx, config.near_goal_max_v);
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
  context.robot = makeRobot(0.0, 0.0, 0.0);
  context.obstacles.valid = true;
  context.obstacles.front_min = 0.3;

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
  context.robot = makeRobot(0.0, 0.0, 0.0);
  context.obstacles.valid = true;
  context.obstacles.front_min = 0.8;

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
  context.robot = makeRobot(0.0, 0.0, 0.0);

  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

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
  context.robot = makeRobot(0.0, 0.0, 0.0);

  const VelocityCommand cmd =
      supervisor.apply(raw_cmd, context, 0.4, 1.0);

  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.source, CommandSource::SAFETY_STOP);
}

}  // namespace
}  // namespace navdog

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
