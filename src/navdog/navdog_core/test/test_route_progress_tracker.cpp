#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/route_progress_tracker.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helpers
// =============================================================================

RobotState makeRobot(double x, double y)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.valid = true;
  return robot;
}

RobotState makeRobot(double x, double y, double yaw)
{
  RobotState robot = makeRobot(x, y);
  robot.yaw = yaw;
  return robot;
}

NavigationTask makeStraightRoute(
    std::uint64_t sequence,
    double x0, double y0,
    double x1, double y1)
{
  NavigationTask task{};
  task.sequence = sequence;
  task.max_vx = 0.4;

  RoutePoint p0{};
  p0.x = x0;
  p0.y = y0;
  task.points.push_back(p0);

  RoutePoint p1{};
  p1.x = x1;
  p1.y = y1;
  task.points.push_back(p1);

  return task;
}

NavigationTask makeMultiPointRoute(
    std::uint64_t sequence,
    const std::vector<std::pair<double, double>>& pts)
{
  NavigationTask task{};
  task.sequence = sequence;
  task.max_vx = 0.4;

  for (const auto& pt : pts)
  {
    RoutePoint p{};
    p.x = pt.first;
    p.y = pt.second;
    task.points.push_back(p);
  }

  return task;
}

RouteProgressConfig defaultConfig()
{
  return RouteProgressConfig{};
}

}  // namespace

// =============================================================================
// 26.1 DefaultStateIsNotInitialized
// =============================================================================

TEST(RouteProgressTrackerTest, DefaultStateIsNotInitialized)
{
  RouteProgressTracker tracker;
  EXPECT_FALSE(tracker.initialized());
}

// =============================================================================
// 26.2 InvalidTimeIsRejected
// =============================================================================

TEST(RouteProgressTrackerTest, InvalidTimeIsRejected)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(3, 0);

  RouteProgressOutput output =
      tracker.update(task, robot,
          std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(output.result, RouteProgressResult::INVALID_TIME);
  EXPECT_FALSE(tracker.initialized());
}

// =============================================================================
// 26.3 InvalidConfigIsRejected
// =============================================================================

TEST(RouteProgressTrackerTest, InvalidConfigIsRejected)
{
  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(3, 0);

  // min_segment_length_m <= 0
  {
    RouteProgressConfig cfg = defaultConfig();
    cfg.min_segment_length_m = 0.0;
    RouteProgressTracker tracker(cfg);

    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_CONFIG);
  }

  // max_forward_search_m <= 0
  {
    RouteProgressConfig cfg = defaultConfig();
    cfg.max_forward_search_m = 0.0;
    RouteProgressTracker tracker(cfg);

    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_CONFIG);
  }

  // on_route_lateral_tolerance_m <= 0
  {
    RouteProgressConfig cfg = defaultConfig();
    cfg.on_route_lateral_tolerance_m = 0.0;
    RouteProgressTracker tracker(cfg);

    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_CONFIG);
  }

  // NaN config
  {
    RouteProgressConfig cfg = defaultConfig();
    cfg.min_segment_length_m =
        std::numeric_limits<double>::quiet_NaN();
    RouteProgressTracker tracker(cfg);

    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_CONFIG);
  }
}

// =============================================================================
// 26.4 InvalidTaskSequenceIsRejected
// =============================================================================

TEST(RouteProgressTrackerTest, InvalidTaskSequenceIsRejected)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(0, 0, 0, 10, 0);
  RobotState robot = makeRobot(3, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  EXPECT_EQ(output.result, RouteProgressResult::INVALID_TASK);
}

// =============================================================================
// 26.5 EmptyRouteIsRejected
// =============================================================================

TEST(RouteProgressTrackerTest, EmptyRouteIsRejected)
{
  RouteProgressTracker tracker;

  NavigationTask task{};
  task.sequence = 1;
  task.max_vx = 0.4;

  RobotState robot = makeRobot(3, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  EXPECT_EQ(output.result, RouteProgressResult::INVALID_TASK);
}

// =============================================================================
// 26.6 NonFiniteRoutePointIsRejected
// =============================================================================

TEST(RouteProgressTrackerTest, NonFiniteRoutePointIsRejected)
{
  RobotState robot = makeRobot(3, 0);

  // x = NaN
  {
    NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
    task.points[0].x =
        std::numeric_limits<double>::quiet_NaN();

    RouteProgressTracker tracker;
    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_TASK);
  }

  // y = Inf
  {
    NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
    task.points[1].y =
        std::numeric_limits<double>::infinity();

    RouteProgressTracker tracker;
    RouteProgressOutput output =
        tracker.update(task, robot, 1.0);

    EXPECT_EQ(output.result,
              RouteProgressResult::INVALID_TASK);
  }
}

// =============================================================================
// 26.7 WaitsForInvalidRobot
// =============================================================================

TEST(RouteProgressTrackerTest, WaitsForInvalidRobot)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);

  RobotState robot{};
  robot.valid = false;

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  EXPECT_EQ(output.result,
            RouteProgressResult::WAITING_FOR_ROBOT);
}

// =============================================================================
// 26.8 ProjectsOntoStraightSegment
// =============================================================================

TEST(RouteProgressTrackerTest, ProjectsOntoStraightSegment)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(3, 1);

  RouteProgressOutput output =
      tracker.update(task, robot, 5.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);

  const RouteProgress& p = output.progress;

  EXPECT_DOUBLE_EQ(p.projected_x, 3.0);
  EXPECT_DOUBLE_EQ(p.projected_y, 0.0);
  EXPECT_DOUBLE_EQ(p.arc_length_m, 3.0);
  EXPECT_DOUBLE_EQ(p.remaining_distance_m, 7.0);
  EXPECT_DOUBLE_EQ(p.lateral_error_m, 1.0);
  EXPECT_DOUBLE_EQ(p.route_yaw, 0.0);
  EXPECT_TRUE(p.valid);
}

// =============================================================================
// 26.9 ComputesSegmentRatio
// =============================================================================

TEST(RouteProgressTrackerTest, ComputesSegmentRatio)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(3, 1);

  RouteProgressOutput output =
      tracker.update(task, robot, 5.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_NEAR(output.progress.segment_ratio, 0.3, 1e-12);
}

// =============================================================================
// 26.10 UsesOriginalSegmentIndex
// =============================================================================

TEST(RouteProgressTrackerTest, UsesOriginalSegmentIndex)
{
  RouteProgressTracker tracker;

  // (0,0), (0,0), (5,0) — first segment is degenerate
  NavigationTask task = makeMultiPointRoute(
      1, {{0, 0}, {0, 0}, {5, 0}});

  RobotState robot = makeRobot(2, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  // The valid segment starts at original index 1
  EXPECT_EQ(output.progress.segment_index, 1u);
}

// =============================================================================
// 26.11 SkipsDegenerateSegments
// =============================================================================

TEST(RouteProgressTrackerTest, SkipsDegenerateSegments)
{
  RouteProgressTracker tracker;

  // All duplicates — should be single point route
  NavigationTask task = makeMultiPointRoute(
      1, {{3, 3}, {3, 3}, {3, 3}});

  RobotState robot = makeRobot(0, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  // Single point route is still valid
  EXPECT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_TRUE(output.progress.valid);
  // No NaN
  EXPECT_TRUE(std::isfinite(output.progress.lateral_error_m));
  EXPECT_TRUE(std::isfinite(output.progress.projected_x));
  EXPECT_TRUE(std::isfinite(output.progress.projected_y));
}

// =============================================================================
// 26.12 ProgressNeverRegresses
// =============================================================================

TEST(RouteProgressTrackerTest, ProgressNeverRegresses)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);

  // First: robot at x=5
  RobotState robot1 = makeRobot(5, 0);
  RouteProgressOutput out1 =
      tracker.update(task, robot1, 1.0);

  ASSERT_EQ(out1.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out1.progress.arc_length_m, 5.0);

  // Second: robot jitter to x=4
  RobotState robot2 = makeRobot(4, 0);
  RouteProgressOutput out2 =
      tracker.update(task, robot2, 2.0);

  ASSERT_EQ(out2.result, RouteProgressResult::VALID);
  EXPECT_GE(out2.progress.arc_length_m, 5.0);
}

// =============================================================================
// 26.13 ProgressAdvancesForward
// =============================================================================

TEST(RouteProgressTrackerTest, ProgressAdvancesForward)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);

  RobotState robot1 = makeRobot(2, 0);
  RouteProgressOutput out1 =
      tracker.update(task, robot1, 1.0);

  ASSERT_EQ(out1.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out1.progress.arc_length_m, 2.0);

  RobotState robot2 = makeRobot(6, 0);
  RouteProgressOutput out2 =
      tracker.update(task, robot2, 2.0);

  ASSERT_EQ(out2.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out2.progress.arc_length_m, 6.0);
}

// =============================================================================
// 26.14 RemainingDistanceNeverNegative
// =============================================================================

TEST(RouteProgressTrackerTest, RemainingDistanceNeverNegative)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);

  // Robot past the endpoint
  RobotState robot = makeRobot(15, 0);
  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_GE(output.progress.remaining_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(output.progress.remaining_distance_m, 0.0);
}

// =============================================================================
// 26.15 OnRouteInsideTolerance
// =============================================================================

TEST(RouteProgressTrackerTest, OnRouteInsideTolerance)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(5, 0.1);  // lateral 0.1 < 0.30

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_TRUE(output.progress.on_route);
}

// =============================================================================
// 26.16 OffRouteOutsideTolerance
// =============================================================================

TEST(RouteProgressTrackerTest, OffRouteOutsideTolerance)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(5, 0.5);  // lateral 0.5 > 0.30

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_TRUE(output.progress.valid);
  EXPECT_FALSE(output.progress.on_route);
}

// =============================================================================
// 26.17 ExactToleranceCountsAsOnRoute
// =============================================================================

TEST(RouteProgressTrackerTest, ExactToleranceCountsAsOnRoute)
{
  RouteProgressConfig cfg = defaultConfig();
  cfg.on_route_lateral_tolerance_m = 0.5;
  RouteProgressTracker tracker(cfg);

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(5, 0.5);  // exactly at boundary

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_TRUE(output.progress.on_route);
}

// =============================================================================
// 26.18 NewTaskSequenceResetsProgress
// =============================================================================

TEST(RouteProgressTrackerTest, NewTaskSequenceResetsProgress)
{
  RouteProgressTracker tracker;

  // Task 1: (0,0) → (10,0)
  NavigationTask task1 = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot1 = makeRobot(5, 0);

  RouteProgressOutput out1 =
      tracker.update(task1, robot1, 1.0);

  ASSERT_EQ(out1.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out1.progress.arc_length_m, 5.0);
  EXPECT_TRUE(tracker.initialized());

  // Task 2: (0,0) → (20,0)
  NavigationTask task2 = makeStraightRoute(2, 0, 0, 20, 0);
  RobotState robot2 = makeRobot(8, 0);

  RouteProgressOutput out2 =
      tracker.update(task2, robot2, 2.0);

  ASSERT_EQ(out2.result, RouteProgressResult::VALID);
  EXPECT_EQ(out2.progress.task_sequence, 2u);
  EXPECT_DOUBLE_EQ(out2.progress.arc_length_m, 8.0);
  EXPECT_DOUBLE_EQ(out2.progress.total_length_m, 20.0);
}

// =============================================================================
// 26.19 MaxVxChangeDoesNotResetProgress
// =============================================================================

TEST(RouteProgressTrackerTest, MaxVxChangeDoesNotResetProgress)
{
  RouteProgressTracker tracker;

  NavigationTask task1 = makeStraightRoute(1, 0, 0, 10, 0);
  task1.max_vx = 0.4;
  RobotState robot1 = makeRobot(5, 0);

  RouteProgressOutput out1 =
      tracker.update(task1, robot1, 1.0);

  ASSERT_EQ(out1.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out1.progress.arc_length_m, 5.0);

  // Same sequence, different max_vx
  NavigationTask task2 = makeStraightRoute(1, 0, 0, 10, 0);
  task2.max_vx = 0.8;
  RobotState robot2 = makeRobot(7, 0);

  RouteProgressOutput out2 =
      tracker.update(task2, robot2, 2.0);

  ASSERT_EQ(out2.result, RouteProgressResult::VALID);
  EXPECT_DOUBLE_EQ(out2.progress.arc_length_m, 7.0);
}

// =============================================================================
// 26.20 ForwardSearchDoesNotJumpToFarCrossing
// =============================================================================

TEST(RouteProgressTrackerTest, ForwardSearchDoesNotJumpToFarCrossing)
{
  RouteProgressConfig cfg = defaultConfig();
  cfg.max_forward_search_m = 3.0;
  RouteProgressTracker tracker(cfg);

  // Route: (0,0) → (10,0) → (10,10) → (0,10) → (0,0)
  // The last segment returns to near the start.
  NavigationTask task = makeMultiPointRoute(
      1, {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}});

  // Initialize at x=2 on first segment
  RobotState robot1 = makeRobot(2, 0);
  RouteProgressOutput out1 =
      tracker.update(task, robot1, 1.0);

  ASSERT_EQ(out1.result, RouteProgressResult::VALID);
  EXPECT_NEAR(out1.progress.arc_length_m, 2.0, 1e-9);

  // Move forward to x=3 on first segment
  // The last segment (0,10)→(0,0) passes near (0,0) which is
  // closer to robot at (3,0) than (10,0), but the forward search
  // limit prevents jumping there.
  RobotState robot2 = makeRobot(3, 0);
  RouteProgressOutput out2 =
      tracker.update(task, robot2, 2.0);

  ASSERT_EQ(out2.result, RouteProgressResult::VALID);
  // Should still be on first segment, not the last one
  EXPECT_EQ(out2.progress.segment_index, 0u);
  EXPECT_NEAR(out2.progress.arc_length_m, 3.0, 1e-9);
}

// =============================================================================
// 26.21 InitialTieUsesEarlierRoutePosition
// =============================================================================

TEST(RouteProgressTrackerTest, InitialTieUsesEarlierRoutePosition)
{
  // Construct a route where two segments are equidistant
  // from the robot.
  // Route: (0,0) → (5,0) → (5,5) → (0,5) → (0,0)
  // Robot at (2.5, 2.5) is equidistant to segments
  // (0,0)→(5,0) and (0,0)→(0,5) (the last segment).
  // But also equidistant to (5,0)→(5,5) and (0,5)→(0,0)?

  // Simpler: an L-shaped route where robot is at the corner.
  // Route: (0,0) → (5,0) → (5,5)
  // Robot at (5,0) — projects to end of seg 0 and start of seg 1
  // with equal distance (0). Should pick seg 0 (smaller arc).

  RouteProgressTracker tracker;

  NavigationTask task = makeMultiPointRoute(
      1, {{0, 0}, {5, 0}, {5, 5}});

  RobotState robot = makeRobot(5, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  // At the junction, both segments give distance 0.
  // Should prefer the earlier segment (arc_length = 5.0, not 5.0).
  // Both have arc_length = 5.0 since cumulative_start of seg 1 = 5.0.
  // So this test verifies it picks seg 0 (original index 0).
  EXPECT_EQ(output.progress.segment_index, 0u);
  EXPECT_NEAR(output.progress.arc_length_m, 5.0, 1e-9);
}

// =============================================================================
// 26.22 SupportsSinglePointRoute
// =============================================================================

TEST(RouteProgressTrackerTest, SupportsSinglePointRoute)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeMultiPointRoute(1, {{5, 0}});
  RobotState robot = makeRobot(2, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_TRUE(output.progress.valid);
  EXPECT_NEAR(output.progress.remaining_distance_m, 3.0, 1e-9);
  EXPECT_DOUBLE_EQ(output.progress.projected_x, 5.0);
  EXPECT_DOUBLE_EQ(output.progress.projected_y, 0.0);
  EXPECT_DOUBLE_EQ(output.progress.total_length_m, 0.0);
  EXPECT_DOUBLE_EQ(output.progress.arc_length_m, 0.0);
}

// =============================================================================
// 26.23 SinglePointUsesExplicitYaw
// =============================================================================

TEST(RouteProgressTrackerTest, SinglePointUsesExplicitYaw)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeMultiPointRoute(1, {{5, 0}});
  task.points[0].has_yaw = true;
  task.points[0].yaw = 1.5;

  RobotState robot = makeRobot(2, 0);

  RouteProgressOutput output =
      tracker.update(task, robot, 1.0);

  ASSERT_EQ(output.result, RouteProgressResult::VALID);
  EXPECT_NEAR(output.progress.route_yaw, 1.5, 1e-12);
}

// =============================================================================
// 26.24 ResetClearsProgress
// =============================================================================

TEST(RouteProgressTrackerTest, ResetClearsProgress)
{
  RouteProgressTracker tracker;

  NavigationTask task = makeStraightRoute(1, 0, 0, 10, 0);
  RobotState robot = makeRobot(5, 0);

  tracker.update(task, robot, 1.0);
  EXPECT_TRUE(tracker.initialized());

  tracker.reset();

  EXPECT_FALSE(tracker.initialized());
}

}  // namespace navdog

// =============================================================================
// main
// =============================================================================

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
