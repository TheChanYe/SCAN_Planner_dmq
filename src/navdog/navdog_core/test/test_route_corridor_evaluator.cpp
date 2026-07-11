#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/route_corridor_evaluator.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helper functions
// =============================================================================

ObstacleField makeObstacleField(double stamp_sec)
{
  ObstacleField field;
  field.stamp_sec = stamp_sec;
  field.valid = true;
  return field;
}

ObstacleCircle makeObstacle(
    double x,
    double y,
    double effective_radius_m)
{
  ObstacleCircle obs;
  obs.x = x;
  obs.y = y;
  obs.effective_radius_m = effective_radius_m;
  return obs;
}

RouteProgress makeRouteProgress(
    std::uint64_t sequence,
    std::size_t segment_index,
    double segment_ratio,
    double arc_length_m,
    double projected_x,
    double projected_y)
{
  RouteProgress progress;
  progress.task_sequence = sequence;
  progress.segment_index = segment_index;
  progress.segment_ratio = segment_ratio;
  progress.arc_length_m = arc_length_m;
  progress.total_length_m = 10.0;
  progress.remaining_distance_m = 10.0 - arc_length_m;
  progress.projected_x = projected_x;
  progress.projected_y = projected_y;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;
  return progress;
}

RobotState makeRobot(double x, double y)
{
  RobotState robot;
  robot.x = x;
  robot.y = y;
  robot.valid = true;
  return robot;
}

NavigationTask makeStraightRoute()
{
  NavigationTask task;
  task.sequence = 1;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  RoutePoint p0;
  p0.x = 0.0;
  p0.y = 0.0;
  task.points.push_back(p0);
  RoutePoint p1;
  p1.x = 10.0;
  p1.y = 0.0;
  task.points.push_back(p1);
  return task;
}

NavigationTask makeMultiPointRoute()
{
  NavigationTask task;
  task.sequence = 1;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  RoutePoint p0;
  p0.x = 0.0;
  p0.y = 0.0;
  task.points.push_back(p0);
  RoutePoint p1;
  p1.x = 3.0;
  p1.y = 0.0;
  task.points.push_back(p1);
  RoutePoint p2;
  p2.x = 6.0;
  p2.y = 0.0;
  task.points.push_back(p2);
  return task;
}

NavigationTask makeSinglePointTask(double x, double y)
{
  NavigationTask task;
  task.sequence = 1;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  RoutePoint p;
  p.x = x;
  p.y = y;
  task.points.push_back(p);
  return task;
}

RouteProgress makeSinglePointProgress(
    std::uint64_t sequence,
    double target_x,
    double target_y)
{
  RouteProgress progress;
  progress.task_sequence = sequence;
  progress.segment_index = 0;
  progress.segment_ratio = 0.0;
  progress.arc_length_m = 0.0;
  progress.total_length_m = 0.0;
  progress.remaining_distance_m = 0.0;
  progress.projected_x = target_x;
  progress.projected_y = target_y;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;
  return progress;
}

// Default test context: route (0,0)->(10,0), progress at x=2,
// robot at (2,0), now=10.0, obstacle field stamp=10.0.
//
// Trimmed first segment: (2,0)->(5,0), length=3.0 (==lookahead).

const double kNow = 10.0;
const double kEps = 1e-9;

// =============================================================================
// 28.1 DefaultOutputIsIdle
// =============================================================================

TEST(RouteCorridorEvaluatorTest, DefaultOutputIsIdle)
{
  RouteCorridorOutput output;
  EXPECT_EQ(output.result, RouteCorridorResult::IDLE);
  EXPECT_FALSE(output.assessment.valid);
}

// =============================================================================
// 28.2 RejectsNonFiniteNow
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsNonFiniteNow)
{
  RouteCorridorEvaluator evaluator;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_TIME);
}

// =============================================================================
// 28.3 RejectsInvalidLookahead
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidLookahead)
{
  {
    NavdogConfig config;
    config.route_corridor.lookahead_distance_m = 0.0;
    RouteCorridorEvaluator evaluator(
        config.route_corridor,
        config.route_progress,
        config.safety);
    RouteCorridorOutput output = evaluator.evaluate(
        makeStraightRoute(),
        makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
        makeRobot(2, 0),
        makeObstacleField(kNow),
        kNow);
    EXPECT_EQ(output.result, RouteCorridorResult::INVALID_CONFIG);
  }
  {
    NavdogConfig config;
    config.route_corridor.lookahead_distance_m =
        std::numeric_limits<double>::quiet_NaN();
    RouteCorridorEvaluator evaluator(
        config.route_corridor,
        config.route_progress,
        config.safety);
    RouteCorridorOutput output = evaluator.evaluate(
        makeStraightRoute(),
        makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
        makeRobot(2, 0),
        makeObstacleField(kNow),
        kNow);
    EXPECT_EQ(output.result, RouteCorridorResult::INVALID_CONFIG);
  }
}

// =============================================================================
// 28.4 RejectsInvalidCorridorRadius
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidCorridorRadius)
{
  NavdogConfig config;
  config.route_corridor.corridor_radius_m = -0.5;
  RouteCorridorEvaluator evaluator(
      config.route_corridor,
      config.route_progress,
      config.safety);
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_CONFIG);
}

// =============================================================================
// 28.5 RejectsInvalidObstacleTimeout
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidObstacleTimeout)
{
  NavdogConfig config;
  config.safety.obstacle_timeout_sec = 0.0;
  RouteCorridorEvaluator evaluator(
      config.route_corridor,
      config.route_progress,
      config.safety);
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_CONFIG);
}

// =============================================================================
// 28.6 RejectsInvalidMinSegmentLength
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidMinSegmentLength)
{
  NavdogConfig config;
  config.route_progress.min_segment_length_m = 0.0;
  RouteCorridorEvaluator evaluator(
      config.route_corridor,
      config.route_progress,
      config.safety);
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_CONFIG);
}

// =============================================================================
// 28.7 RejectsZeroTaskSequence
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsZeroTaskSequence)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task = makeStraightRoute();
  task.sequence = 0;
  RouteCorridorOutput output = evaluator.evaluate(
      task,
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_TASK);
}

// =============================================================================
// 28.8 RejectsEmptyTask
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsEmptyTask)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task;
  task.sequence = 1;
  RouteCorridorOutput output = evaluator.evaluate(
      task,
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_TASK);
}

// =============================================================================
// 28.9 RejectsNonFiniteRoutePoint
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsNonFiniteRoutePoint)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task = makeStraightRoute();
  task.points[1].x = std::numeric_limits<double>::infinity();
  RouteCorridorOutput output = evaluator.evaluate(
      task,
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_TASK);
}

// =============================================================================
// 28.10 RejectsInvalidProgress
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidProgress)
{
  RouteCorridorEvaluator evaluator;
  RouteProgress progress =
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0);
  progress.valid = false;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      progress,
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_PROGRESS);
}

// =============================================================================
// 28.11 RejectsMismatchedProgressSequence
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsMismatchedProgressSequence)
{
  RouteCorridorEvaluator evaluator;
  RouteProgress progress =
      makeRouteProgress(999, 0, 0.2, 2.0, 2.0, 0.0);
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      progress,
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_PROGRESS);
}

// =============================================================================
// 28.12 RejectsOutOfRangeSegmentIndex
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsOutOfRangeSegmentIndex)
{
  RouteCorridorEvaluator evaluator;
  // segment_index=5, but task only has 2 points (index 0 and 1)
  // segment_index + 1 = 6 >= 2 = points.size()
  RouteProgress progress =
      makeRouteProgress(1, 5, 0.0, 2.0, 2.0, 0.0);
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      progress,
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_PROGRESS);
}

// =============================================================================
// 28.13 RejectsInvalidRobot
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsInvalidRobot)
{
  RouteCorridorEvaluator evaluator;
  RobotState robot;
  // robot.valid = false
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      robot,
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::INVALID_ROBOT);
}

// =============================================================================
// 28.14 WaitsForUnavailableObstacleField
// =============================================================================

TEST(RouteCorridorEvaluatorTest, WaitsForUnavailableObstacleField)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field;
  field.valid = false;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::WAITING_FOR_OBSTACLES);
}

// =============================================================================
// 28.15 RejectsNaNObstacleStamp
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsNaNObstacleStamp)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field;
  field.valid = true;
  field.stamp_sec =
      std::numeric_limits<double>::quiet_NaN();
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::INVALID_OBSTACLES);
}

// =============================================================================
// 28.16 RejectsFutureObstacleField
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsFutureObstacleField)
{
  RouteCorridorEvaluator evaluator;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow + 0.1),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::FUTURE_OBSTACLES);
}

// =============================================================================
// 28.17 RejectsStaleObstacleField
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsStaleObstacleField)
{
  RouteCorridorEvaluator evaluator;
  // default obstacle_timeout_sec = 0.5
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow - 0.6),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::STALE_OBSTACLES);
}

// =============================================================================
// 28.18 AcceptsObstacleAtExactTimeoutBoundary
// =============================================================================

TEST(RouteCorridorEvaluatorTest, AcceptsObstacleAtExactTimeoutBoundary)
{
  RouteCorridorEvaluator evaluator;
  // age = now - stamp = 0.5 == timeout → still valid
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow - 0.5),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_TRUE(output.assessment.valid);
}

// =============================================================================
// 28.19 RejectsNaNObstaclePosition
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsNaNObstaclePosition)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(
          std::numeric_limits<double>::quiet_NaN(),
          0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::INVALID_OBSTACLES);
}

// =============================================================================
// 28.20 RejectsNegativeObstacleRadius
// =============================================================================

TEST(RouteCorridorEvaluatorTest, RejectsNegativeObstacleRadius)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, -0.5));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorResult::INVALID_OBSTACLES);
}

// =============================================================================
// 28.21 EmptyValidObstacleFieldIsClear
// =============================================================================

TEST(RouteCorridorEvaluatorTest, EmptyValidObstacleFieldIsClear)
{
  RouteCorridorEvaluator evaluator;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_TRUE(output.assessment.valid);
  EXPECT_FALSE(output.assessment.blocked);
  EXPECT_TRUE(std::isinf(
      output.assessment.minimum_clearance_m));
  EXPECT_NEAR(output.assessment.checked_distance_m,
              3.0, kEps);
}

// =============================================================================
// 28.22 ObstacleOnRouteBlocks
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleOnRouteBlocks)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_TRUE(output.assessment.blocked);
}

// =============================================================================
// 28.23 ObstacleOutsideCorridorIsClear
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleOutsideCorridorIsClear)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(4.0, 1.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_FALSE(output.assessment.blocked);
}

// =============================================================================
// 28.24 ObstacleAtExactBoundaryBlocks
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleAtExactBoundaryBlocks)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // distance = 0.30 == corridor_radius → blocked
  field.obstacles.push_back(
      makeObstacle(4.0, 0.30, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_TRUE(output.assessment.blocked);
}

// =============================================================================
// 28.25 ObstacleRadiusExpandsIntoCorridor
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleRadiusExpandsIntoCorridor)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // distance = 0.50, radius = 0.25, required = 0.55
  // 0.50 <= 0.55 → blocked
  field.obstacles.push_back(
      makeObstacle(4.0, 0.50, 0.25));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
}

// =============================================================================
// 28.26 PointObstacleUsesZeroRadius
// =============================================================================

TEST(RouteCorridorEvaluatorTest, PointObstacleUsesZeroRadius)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
}

// =============================================================================
// 28.27 ObstacleBehindProgressIsIgnored
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleBehindProgressIsIgnored)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=1, behind projected_x=2
  field.obstacles.push_back(
      makeObstacle(1.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_FALSE(output.assessment.blocked);
  // Obstacle was not evaluated → infinite clearance
  EXPECT_TRUE(std::isinf(
      output.assessment.minimum_clearance_m));
}

// =============================================================================
// 28.28 ObstacleBeyondLookaheadIsIgnored
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleBeyondLookaheadIsIgnored)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=6, beyond lookahead boundary x=5
  field.obstacles.push_back(
      makeObstacle(6.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_FALSE(output.assessment.blocked);
  EXPECT_TRUE(std::isinf(
      output.assessment.first_blocked_distance_ahead_m));
}

// =============================================================================
// 28.29 ObstacleAtLookaheadBoundaryIsEvaluated
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ObstacleAtLookaheadBoundaryIsEvaluated)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=5, exactly at lookahead boundary (2+3=5)
  field.obstacles.push_back(
      makeObstacle(5.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      3.0, kEps);
}

// =============================================================================
// 28.30 ChoosesEarliestBlockedObstacle
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ChoosesEarliestBlockedObstacle)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // Obstacle 0 at x=4 (distance_ahead = 2.0)
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));
  // Obstacle 1 at x=3 (distance_ahead = 1.0) → earliest
  field.obstacles.push_back(
      makeObstacle(3.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      1.0, kEps);
}

// =============================================================================
// 28.31 ReportsBlockedDistanceAhead
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsBlockedDistanceAhead)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(3.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      1.0, kEps);
}

// =============================================================================
// 28.32 ReportsBlockedArcLength
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsBlockedArcLength)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(3.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  // arc_length = 2.0 + 1.0 = 3.0
  EXPECT_NEAR(
      output.assessment.first_blocked_arc_length_m,
      3.0, kEps);
}

// =============================================================================
// 28.33 ReportsObstacleIndex
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsObstacleIndex)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // Obstacle 0 farther
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));
  // Obstacle 1 closer → earliest blocker
  field.obstacles.push_back(
      makeObstacle(3.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_EQ(output.assessment.obstacle_index, 1u);
}

// =============================================================================
// 28.34 ReportsPositiveMinimumClearance
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsPositiveMinimumClearance)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // distance = 0.50, required = 0.30 → clearance = 0.20
  field.obstacles.push_back(
      makeObstacle(4.0, 0.50, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_NEAR(
      output.assessment.minimum_clearance_m,
      0.20, kEps);
}

// =============================================================================
// 28.35 ReportsZeroClearanceAtBoundary
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsZeroClearanceAtBoundary)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // distance = 0.30, required = 0.30 → clearance = 0.0
  field.obstacles.push_back(
      makeObstacle(4.0, 0.30, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_NEAR(
      output.assessment.minimum_clearance_m,
      0.0, kEps);
}

// =============================================================================
// 28.36 ReportsNegativeClearanceWhenOverlapping
// =============================================================================

TEST(RouteCorridorEvaluatorTest, ReportsNegativeClearanceWhenOverlapping)
{
  RouteCorridorEvaluator evaluator;
  ObstacleField field = makeObstacleField(kNow);
  // distance = 0.0, required = 0.30 → clearance = -0.30
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      field,
      kNow);
  EXPECT_NEAR(
      output.assessment.minimum_clearance_m,
      -0.30, kEps);
}

// =============================================================================
// 28.37 NoObstaclesKeepsInfiniteClearance
// =============================================================================

TEST(RouteCorridorEvaluatorTest, NoObstaclesKeepsInfiniteClearance)
{
  RouteCorridorEvaluator evaluator;
  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(),
      makeRouteProgress(1, 0, 0.2, 2.0, 2.0, 0.0),
      makeRobot(2, 0),
      makeObstacleField(kNow),
      kNow);
  EXPECT_TRUE(std::isinf(
      output.assessment.minimum_clearance_m));
}

// =============================================================================
// 28.38 DetectsObstacleOnSecondSegment
// =============================================================================

TEST(RouteCorridorEvaluatorTest, DetectsObstacleOnSecondSegment)
{
  RouteCorridorEvaluator evaluator;
  // Route: (0,0)→(3,0)→(6,0), progress at x=1
  // First segment: (1,0)→(3,0), length=2.0
  // Second segment: (3,0)→(4,0), trimmed to 1.0 (lookahead=3.0)
  NavigationTask task = makeMultiPointRoute();
  RouteProgress progress =
      makeRouteProgress(1, 0, 1.0 / 3.0, 1.0, 1.0, 0.0);
  progress.total_length_m = 6.0;
  progress.remaining_distance_m = 5.0;

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle on second segment at x=3.5
  field.obstacles.push_back(
      makeObstacle(3.5, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(1, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  // distance_ahead = 2.0 + 0.5 = 2.5
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      2.5, kEps);
}

// =============================================================================
// 28.39 SkipsDegenerateRouteSegments
// =============================================================================

TEST(RouteCorridorEvaluatorTest, SkipsDegenerateRouteSegments)
{
  RouteCorridorEvaluator evaluator;
  // Route: (0,0)→(0.005,0)→(5,0)
  // First segment degenerate (length=0.005 < 0.01) → skip
  // Second segment: (0.005,0)→(5,0), length≈5.0
  NavigationTask task;
  task.sequence = 1;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  RoutePoint p0; p0.x = 0.0; p0.y = 0.0;
  task.points.push_back(p0);
  RoutePoint p1; p1.x = 0.005; p1.y = 0.0;
  task.points.push_back(p1);
  RoutePoint p2; p2.x = 5.0; p2.y = 0.0;
  task.points.push_back(p2);

  RouteProgress progress =
      makeRouteProgress(1, 0, 0.0, 0.0, 0.0, 0.0);
  progress.total_length_m = 5.0;
  progress.remaining_distance_m = 5.0;

  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(3.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(0, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
}

// =============================================================================
// 28.40 TrimsFinalSegmentToLookahead
// =============================================================================

TEST(RouteCorridorEvaluatorTest, TrimsFinalSegmentToLookahead)
{
  RouteCorridorEvaluator evaluator;
  // Route: (0,0)→(10,0), progress at x=0
  // First segment: (0,0)→(10,0), trimmed to (0,0)→(3,0)
  RouteProgress progress =
      makeRouteProgress(1, 0, 0.0, 0.0, 0.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=2.9 → within trimmed segment → blocked
  field.obstacles.push_back(
      makeObstacle(2.9, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(), progress,
      makeRobot(0, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_NEAR(
      output.assessment.checked_distance_m,
      3.0, kEps);
}

// =============================================================================
// 28.41 CheckedDistanceStopsAtRouteEnd
// =============================================================================

TEST(RouteCorridorEvaluatorTest, CheckedDistanceStopsAtRouteEnd)
{
  RouteCorridorEvaluator evaluator;
  // Route: (0,0)→(2,0), shorter than lookahead=3.0
  NavigationTask task;
  task.sequence = 1;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  RoutePoint p0; p0.x = 0.0; p0.y = 0.0;
  task.points.push_back(p0);
  RoutePoint p1; p1.x = 2.0; p1.y = 0.0;
  task.points.push_back(p1);

  RouteProgress progress =
      makeRouteProgress(1, 0, 0.0, 0.0, 0.0, 0.0);
  progress.total_length_m = 2.0;
  progress.remaining_distance_m = 2.0;

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(0, 0),
      makeObstacleField(kNow), kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_NEAR(
      output.assessment.checked_distance_m,
      2.0, kEps);
}

// =============================================================================
// 28.42 StartsFromProjectedPoint
// =============================================================================

TEST(RouteCorridorEvaluatorTest, StartsFromProjectedPoint)
{
  RouteCorridorEvaluator evaluator;
  // Progress at x=5, so first segment starts at (5,0)
  RouteProgress progress =
      makeRouteProgress(1, 0, 0.5, 5.0, 5.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=6 → within lookahead (5+3=8)
  field.obstacles.push_back(
      makeObstacle(6.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(), progress,
      makeRobot(5, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      1.0, kEps);
}

// =============================================================================
// 28.43 DoesNotCheckAlreadyTravelledRoute
// =============================================================================

TEST(RouteCorridorEvaluatorTest, DoesNotCheckAlreadyTravelledRoute)
{
  RouteCorridorEvaluator evaluator;
  // Progress at x=5
  RouteProgress progress =
      makeRouteProgress(1, 0, 0.5, 5.0, 5.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=4 → behind projected_x=5 → ignored
  field.obstacles.push_back(
      makeObstacle(4.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      makeStraightRoute(), progress,
      makeRobot(5, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_FALSE(output.assessment.blocked);
  EXPECT_TRUE(std::isinf(
      output.assessment.minimum_clearance_m));
}

// =============================================================================
// 28.44 SinglePointUsesRobotToTargetSegment
// =============================================================================

TEST(RouteCorridorEvaluatorTest, SinglePointUsesRobotToTargetSegment)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task =
      makeSinglePointTask(5.0, 0.0);
  RouteProgress progress =
      makeSinglePointProgress(1, 5.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=2, on the robot→target line
  field.obstacles.push_back(
      makeObstacle(2.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(0, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
}

// =============================================================================
// 28.45 SinglePointDetectsBlockingObstacle
// =============================================================================

TEST(RouteCorridorEvaluatorTest, SinglePointDetectsBlockingObstacle)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task =
      makeSinglePointTask(5.0, 0.0);
  RouteProgress progress =
      makeSinglePointProgress(1, 5.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(2.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(0, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::BLOCKED);
  EXPECT_TRUE(output.assessment.blocked);
  // Virtual segment: (0,0)→(3,0) (trimmed to lookahead)
  // Obstacle at x=2 → ratio = 2/3, distance_ahead = 2.0
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      2.0, kEps);
}

// =============================================================================
// 28.46 SinglePointAtGoalIsClear
// =============================================================================

TEST(RouteCorridorEvaluatorTest, SinglePointAtGoalIsClear)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task =
      makeSinglePointTask(5.0, 0.0);
  RouteProgress progress =
      makeSinglePointProgress(1, 5.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  field.obstacles.push_back(
      makeObstacle(5.0, 0.0, 0.0));

  // Robot at target
  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(5, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_NEAR(
      output.assessment.checked_distance_m,
      0.0, kEps);
}

// =============================================================================
// 28.47 SinglePointRespectsLookahead
// =============================================================================

TEST(RouteCorridorEvaluatorTest, SinglePointRespectsLookahead)
{
  RouteCorridorEvaluator evaluator;
  NavigationTask task =
      makeSinglePointTask(10.0, 0.0);
  RouteProgress progress =
      makeSinglePointProgress(1, 10.0, 0.0);

  ObstacleField field = makeObstacleField(kNow);
  // Obstacle at x=5, beyond lookahead=3.0
  // Virtual segment: (0,0)→(3,0), obstacle at 5 → distance=2.0 > 0.30
  field.obstacles.push_back(
      makeObstacle(5.0, 0.0, 0.0));

  RouteCorridorOutput output = evaluator.evaluate(
      task, progress, makeRobot(0, 0), field, kNow);
  EXPECT_EQ(output.result, RouteCorridorResult::CLEAR);
  EXPECT_FALSE(output.assessment.blocked);
  EXPECT_NEAR(
      output.assessment.checked_distance_m,
      3.0, kEps);
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
