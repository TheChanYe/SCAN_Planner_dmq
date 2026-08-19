#include <gtest/gtest.h>
#include "navdog_core/route_manager.hpp"

#include <utility>

namespace
{
std::vector<navdog_task::RoutePoint> line()
{
  std::vector<navdog_task::RoutePoint> points(3);
  points[1].x = 1.0;
  points[2].x = 2.0;
  return points;
}
navdog::RobotState robot(double x, double y = 0.0)
{
  navdog::RobotState state{};
  state.x = x; state.y = y; state.valid = true;
  return state;
}
}

TEST(RouteManager, OwnsOneRouteAndRejectsSameSequenceReplacement)
{
  navdog::RouteManager manager;
  ASSERT_TRUE(manager.acceptRoute(1, line()));
  EXPECT_FALSE(manager.acceptRoute(1, line()));
  EXPECT_EQ(3u, manager.route().size());
  ASSERT_TRUE(manager.acceptRoute(2, line()));
  EXPECT_FALSE(manager.progress().valid);
}

TEST(RouteManager, ProgressNeverMovesBackwardAndQueriesForward)
{
  navdog::RouteManager manager;
  ASSERT_TRUE(manager.acceptRoute(1, line()));
  auto forward = manager.updateProgress(robot(1.5), 1.0);
  auto backward = manager.updateProgress(robot(0.2), 2.0);
  EXPECT_GE(backward.progress.arc_length_m, forward.progress.arc_length_m);
  navdog_task::RoutePoint point{};
  ASSERT_TRUE(manager.pointAtArcLength(0.5, point));
  EXPECT_NEAR(0.5, point.x, 1e-9);
  ASSERT_TRUE(manager.forwardTarget(0.5, 1.0, point));
  EXPECT_NEAR(1.5, point.x, 1e-9);
}

TEST(RouteManager, HandlesSingleAndRepeatedPointsAndReset)
{
  navdog::RouteManager manager;
  std::vector<navdog_task::RoutePoint> points(2);
  ASSERT_TRUE(manager.acceptRoute(1, points));
  EXPECT_TRUE(manager.updateProgress(robot(0.0), 1.0).progress.valid);
  manager.reset();
  EXPECT_FALSE(manager.hasRoute());
  EXPECT_EQ(nullptr, manager.goal());
}

TEST(RouteManager, NewSequenceResetsProgress)
{
  navdog::RouteManager manager;
  ASSERT_TRUE(manager.acceptRoute(1, line()));
  EXPECT_GT(manager.updateProgress(robot(1.5), 1.0).progress.arc_length_m, 1.0);
  ASSERT_TRUE(manager.acceptRoute(2, line()));
  EXPECT_NEAR(0.2,
      manager.updateProgress(robot(0.2), 2.0).progress.arc_length_m, 1e-9);
}

TEST(RouteManager, CrossingRouteCannotJumpBackToOldBranch)
{
  navdog::RouteProgressConfig config;
  config.max_forward_search_m = 2.0;
  navdog::RouteManager manager(config);
  std::vector<navdog_task::RoutePoint> points(5);
  points[0].x = -1.0; points[0].y = -1.0;
  points[1].x = 1.0; points[1].y = 1.0;
  points[2].x = 1.0; points[2].y = -1.0;
  points[3].x = -1.0; points[3].y = 1.0;
  points[4].x = -2.0; points[4].y = 1.0;
  ASSERT_TRUE(manager.acceptRoute(1, std::move(points)));
  const double advanced =
      manager.updateProgress(robot(0.8, -0.8), 1.0).progress.arc_length_m;
  const double crossing =
      manager.updateProgress(robot(0.0, 0.0), 2.0).progress.arc_length_m;
  EXPECT_GE(crossing, advanced);
}

TEST(RouteManager, LoopRouteProgressCannotRegress)
{
  navdog::RouteManager manager;
  std::vector<navdog_task::RoutePoint> points(5);
  points[1].x = 1.0;
  points[2].x = 1.0; points[2].y = 1.0;
  points[3].y = 1.0;
  ASSERT_TRUE(manager.acceptRoute(1, points));
  const double late =
      manager.updateProgress(robot(0.0, 0.8), 1.0).progress.arc_length_m;
  const double near_start =
      manager.updateProgress(robot(0.0, 0.0), 2.0).progress.arc_length_m;
  EXPECT_GE(near_start, late);
}

TEST(RouteManager, GoalAndOutOfRangeInterpolationUseLastPoint)
{
  navdog::RouteManager manager;
  ASSERT_TRUE(manager.acceptRoute(1, line()));
  ASSERT_NE(nullptr, manager.goal());
  EXPECT_DOUBLE_EQ(2.0, manager.goal()->x);
  navdog_task::RoutePoint point{};
  ASSERT_TRUE(manager.pointAtArcLength(100.0, point));
  EXPECT_DOUBLE_EQ(2.0, point.x);
  EXPECT_FALSE(manager.forwardTarget(0.0, -1.0, point));
}

TEST(RouteManager, ElevationAssessmentRequiresConsecutiveRawRisingPoints)
{
  const auto assess = [](const std::vector<double>& z_values,
                         double spacing = 0.5) {
    navdog::RouteManager manager;
    std::vector<navdog_task::RoutePoint> points;
    for (std::size_t i = 0; i < z_values.size(); ++i)
    {
      navdog_task::RoutePoint point;
      point.x = static_cast<double>(i) * spacing;
      point.z = z_values[i];
      points.push_back(point);
    }
    EXPECT_TRUE(manager.acceptRoute(1, points));
    navdog::RouteProgress progress;
    progress.valid = true;
    progress.task_sequence = 1;
    progress.segment_index = 0;
    progress.segment_ratio = 0.0;
    progress.arc_length_m = 0.0;
    progress.total_length_m = points.back().x;
    navdog::StairUpConfig config{};
    config.lookahead_distance_m = 2.20;
    config.trigger_rise_m = 0.10;
    config.min_consecutive_rising_points = 4;
    return manager.assessElevation(progress, config);
  };

  EXPECT_FALSE(assess({0.30, 0.30, 0.30, 0.30, 0.30}).ascending);
  EXPECT_FALSE(assess({0.30, 0.30, 0.50, 0.30, 0.30}).ascending);
  EXPECT_FALSE(assess({0.30, 0.33, 0.36, 0.39}).ascending);
  EXPECT_FALSE(assess({0.30, 0.32, 0.34, 0.36, 0.38}).ascending);

  const auto ascent = assess({0.30, 0.33, 0.36, 0.39, 0.42});
  EXPECT_TRUE(ascent.ascending);
  EXPECT_EQ(4, ascent.consecutive_rising_points);
  EXPECT_NEAR(0.12, ascent.rise_m, 1e-9);

  EXPECT_FALSE(assess({0.30, 0.30, 0.30, 0.33, 0.36, 0.39, 0.42},
      0.8).ascending);
}
