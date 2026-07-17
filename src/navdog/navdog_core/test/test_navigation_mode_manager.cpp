#include <gtest/gtest.h>

#include "navdog_core/navigation_mode_manager.hpp"

#include <limits>

namespace navdog
{
namespace
{

NavigationTask task(TaskMode mode = TaskMode::NORMAL_AVOID)
{
  NavigationTask value{};
  value.sequence = 1;
  value.mode = mode;
  value.max_vx = 0.4;
  value.points.resize(2);
  value.points[1].x = 10.0;
  return value;
}

RobotState robot()
{
  RobotState value{};
  value.valid = true;
  return value;
}

RouteProgress progress()
{
  RouteProgress value{};
  value.task_sequence = 1;
  value.arc_length_m = 0.0;
  value.stamp_sec = 1.0;
  value.valid = true;
  return value;
}

RouteCorridorObservationOutput corridor(bool blocked, double distance = 0.5)
{
  RouteCorridorObservationOutput value{};
  value.result = blocked ? RouteCorridorObservationResult::BLOCKED
                         : RouteCorridorObservationResult::CLEAR;
  value.assessment.source = RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  value.assessment.task_sequence = 1;
  value.assessment.blocked = blocked;
  value.assessment.first_blocked_distance_ahead_m = blocked
      ? distance : std::numeric_limits<double>::infinity();
  value.assessment.valid = true;
  return value;
}

TEST(NavigationModeManagerTest, DefaultStateIsNone)
{
  NavigationModeManager manager;
  EXPECT_EQ(manager.status().mode, NavigationMode::NONE);
}

TEST(NavigationModeManagerTest, ClearInitializesRouteFollow)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(), robot(), progress(), corridor(false), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, BlockWithinTwoMetersImmediatelyEntersAvoid)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 2.0), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_TRUE(output.status.transitioned);
}

TEST(NavigationModeManagerTest, BlockBeyondTwoMetersStaysRouteFollow)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 2.01), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, ClearImmediatelyLeavesAvoid)
{
  NavigationModeManager manager;
  manager.update(task(), robot(), progress(), corridor(true), 1.0);
  const auto output = manager.update(
      task(), robot(), progress(), corridor(false), 1.02);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.previous_mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, MissingCorridorNeverLeavesAvoid)
{
  NavigationModeManager manager;
  manager.update(task(), robot(), progress(), corridor(true), 1.0);
  RouteCorridorObservationOutput missing{};
  missing.result = RouteCorridorObservationResult::WAITING_FOR_OBSERVATION;
  const auto output = manager.update(
      task(), robot(), progress(), missing, 1.02);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, RouteOnlyDoesNotEnterAvoid)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(TaskMode::ROUTE_ONLY), robot(), progress(), corridor(true), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, ResetReturnsToNone)
{
  NavigationModeManager manager;
  manager.update(task(), robot(), progress(), corridor(true), 1.0);
  manager.reset();
  EXPECT_EQ(manager.status().mode, NavigationMode::NONE);
}

}  // namespace
}  // namespace navdog

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
