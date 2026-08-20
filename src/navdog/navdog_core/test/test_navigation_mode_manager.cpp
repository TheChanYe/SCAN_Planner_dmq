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

ObstacleSummary obstacles(double front = 3.0, double left = 2.0,
                           double right = 2.0, bool valid = true)
{
  ObstacleSummary value{};
  value.front_min = front;
  value.left_min = left;
  value.right_min = right;
  value.stamp_sec = 1.0;
  value.valid = valid;
  return value;
}

RouteElevationAssessment elevation(bool ascending, double rise,
                                    double checked_until)
{
  RouteElevationAssessment value{};
  value.valid = true;
  value.ascending = ascending;
  value.current_z = 0.30;
  value.baseline_z = 0.30;
  value.baseline_consistent = true;
  value.ascent_end_z = 0.30 + rise;
  value.rise_m = rise;
  value.consecutive_rising_points = ascending ? 4 : 0;
  value.checked_until_arc_m = checked_until;
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
      task(), robot(), progress(), corridor(false), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, BlockBeyondTwoMetersStaysRouteFollow)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 2.01), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, RouteOnlyDoesNotEnterAvoid)
{
  NavigationModeManager manager;
  const auto output = manager.update(
      task(TaskMode::ROUTE_ONLY), robot(), progress(), corridor(true),
      obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, ResetReturnsToNone)
{
  NavigationModeManager manager;
  manager.update(task(), robot(), progress(), corridor(true), obstacles(), 1.0);
  manager.reset();
  EXPECT_EQ(manager.status().mode, NavigationMode::NONE);
}

// --- New hysteresis tests ---

TEST(NavigationModeManagerTest, SingleFrameBlockDoesNotEnterAvoid)
{
  NavigationModeManager manager;
  // Block at 1.5m for 20ms (0.02s < 0.04s confirm).
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.0);
  EXPECT_EQ(manager.status().mode, NavigationMode::ROUTE_FOLLOW);

  // CLEAR at 1.02s (0.02s later).
  const auto output = manager.update(
      task(), robot(), progress(), corridor(false), obstacles(), 1.02);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, ContinuousBlockEntersAvoid)
{
  NavigationModeManager manager;
  // Block at 1.5m starting at t=1.0.
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.0);
  EXPECT_EQ(manager.status().mode, NavigationMode::ROUTE_FOLLOW);

  // Still blocked at 1.03s (< 0.04s).
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.03);
  EXPECT_EQ(manager.status().mode, NavigationMode::ROUTE_FOLLOW);

  // Still blocked at 1.05s (> 0.04s).
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 1.5), obstacles(), 1.05);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_TRUE(output.status.transitioned);
  EXPECT_EQ(
    output.status.reason,
    NavigationModeReason::BLOCK_CONFIRMED);
}

TEST(NavigationModeManagerTest, ImmediateDistanceEntersAvoid)
{
  NavigationModeManager manager;
  // Block at 0.40m — below immediate threshold (0.45m).
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 0.40), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_TRUE(output.status.transitioned);
  EXPECT_EQ(
    output.status.reason,
    NavigationModeReason::BLOCK_IMMEDIATE);
}

TEST(NavigationModeManagerTest, MinHoldKeepsAvoid)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  config.exit_clear_confirm_sec = 0.5;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID via immediate distance.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  // Corridor CLEAR + directional clearance OK at 1.3s,
  // but only 0.3s in LOCAL_AVOID < 1.0s hold.
  const auto output = mgr.update(
      task(), robot(), progress(), corridor(false), obstacles(), 1.3);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, ClearTimeInsufficientKeepsAvoid)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_clear_confirm_sec = 0.5;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  // Corridor CLEAR + clearance OK at 2.0s (hold satisfied), but only 0.3s < 0.5s.
  mgr.update(task(), robot(), progress(), corridor(false), obstacles(), 2.0);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  const auto output = mgr.update(
      task(), robot(), progress(), corridor(false), obstacles(), 2.3);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, ClearSufficientExitsAvoid)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_clear_confirm_sec = 0.5;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  // Corridor CLEAR + directional clearance OK at 2.0s (hold satisfied).
  mgr.update(task(), robot(), progress(), corridor(false), obstacles(), 2.0);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  // Still clear at 2.3s (0.3s < 0.5s).
  mgr.update(task(), robot(), progress(), corridor(false), obstacles(), 2.3);
  EXPECT_EQ(mgr.status().mode, NavigationMode::LOCAL_AVOID);

  // Still clear at 2.5s (0.5s >= 0.5s) — exit.
  const auto output = mgr.update(
      task(), robot(), progress(), corridor(false), obstacles(), 2.5);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.previous_mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, CorridorBlockedPreventsExit)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_clear_confirm_sec = 0.5;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);

  // Directional clearance OK but corridor still BLOCKED.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 2.0);
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 2.3);
  const auto output = mgr.update(
      task(), robot(), progress(), corridor(true, 0.30), obstacles(), 2.6);
  // Should NOT exit because corridor is still BLOCKED.
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, SideBlockedPreventsExit)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_clear_confirm_sec = 0.5;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);

  // front=3.0 OK, left=0.4 BLOCKED — should not exit.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(3.0, 0.4, 1.0), 2.0);
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(3.0, 0.4, 1.0), 2.3);
  const auto output = mgr.update(
      task(), robot(), progress(), corridor(true, 0.30),
      obstacles(3.0, 0.4, 1.0), 2.6);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, InvalidObstaclesPreventExit)
{
  NavigationModeConfig config;
  config.min_local_avoid_hold_sec = 1.0;
  config.exit_clear_confirm_sec = 0.5;
  config.exit_front_clearance_m = 2.5;
  config.exit_left_clearance_m = 0.6;
  config.exit_right_clearance_m = 0.6;
  NavigationModeManager mgr(config);

  // Enter LOCAL_AVOID.
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             obstacles(), 1.0);

  // obstacles.valid=false — should not exit.
  ObstacleSummary invalid_obs;
  invalid_obs.valid = false;

  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             invalid_obs, 2.0);
  mgr.update(task(), robot(), progress(), corridor(true, 0.30),
             invalid_obs, 2.3);
  const auto output = mgr.update(
      task(), robot(), progress(), corridor(true, 0.30),
      invalid_obs, 2.6);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, CandidateTimerResets)
{
  NavigationModeManager manager;
  // Block at 1.5m for 30ms.
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.0);
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.03);
  // CLEAR for one frame — resets timer.
  manager.update(task(), robot(), progress(), corridor(false),
                 obstacles(), 1.04);
  // Block again for 30ms.
  manager.update(task(), robot(), progress(), corridor(true, 1.5),
                 obstacles(), 1.07);
  // Only 30ms of new block, not 60ms total.
  const auto output = manager.update(
      task(), robot(), progress(), corridor(true, 1.5), obstacles(), 1.09);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, RouteAscentEntersAvoidImmediatelyWithPriority)
{
  NavigationModeManager manager;
  auto r = robot();
  r.z = 0.32;
  const auto output = manager.update(task(), r, progress(),
      elevation(true, 0.15, 1.5), corridor(true, 0.2), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason, NavigationModeReason::ROUTE_ASCENDING);
  EXPECT_TRUE(output.status.stair_up_active);
  EXPECT_DOUBLE_EQ(output.status.stair_hold_until_arc_m, 1.5);
}

TEST(NavigationModeManagerTest, RouteAscentRequiresRobotZConsistency)
{
  NavigationModeManager manager;
  auto r = robot();
  r.z = 0.32;
  auto output = manager.update(task(), r, progress(),
      elevation(true, 0.15, 1.5), corridor(false), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason, NavigationModeReason::ROUTE_ASCENDING);
  EXPECT_TRUE(output.status.stair_up_active);

  NavigationModeManager mismatch_manager;
  r.z = 0.05;
  navdog::RouteElevationAssessment route_ascent =
      elevation(true, 0.15, 1.5);
  route_ascent.current_z = -0.15;
  route_ascent.baseline_z = -0.15;
  output = mismatch_manager.update(task(), r, progress(),
      route_ascent, corridor(false), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
  EXPECT_NE(output.status.reason, NavigationModeReason::ROUTE_ASCENDING);
  EXPECT_FALSE(output.status.stair_up_active);

  NavigationModeConfig mode_config;
  mode_config.enter_confirm_sec = 0.0;
  NavigationModeManager blocked_manager(mode_config, StairUpConfig{});
  output = blocked_manager.update(task(), r, progress(),
      route_ascent, corridor(true, 1.0), obstacles(), 1.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_FALSE(output.status.stair_up_active);
  EXPECT_NE(output.status.reason, NavigationModeReason::ROUTE_ASCENDING);
}

TEST(NavigationModeManagerTest, RouteAscentLatchesAfterOrdinaryAvoidEntry)
{
  NavigationModeConfig mode_config;
  mode_config.enter_confirm_sec = 0.0;
  NavigationModeManager manager(mode_config, StairUpConfig{});

  auto r = robot();
  r.z = 0.32;
  auto output = manager.update(task(), r, progress(),
      elevation(false, 0.0, 1.5), corridor(true, 1.0), obstacles(), 1.0);
  ASSERT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  ASSERT_EQ(output.status.reason, NavigationModeReason::BLOCK_CONFIRMED);
  ASSERT_FALSE(output.status.stair_up_active);

  output = manager.update(task(), r, progress(),
      elevation(true, 0.15, 2.0), corridor(true, 0.5), obstacles(), 1.1);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_FALSE(output.status.transitioned);
  EXPECT_EQ(output.status.reason, NavigationModeReason::ROUTE_ASCENDING);
  EXPECT_TRUE(output.status.stair_up_active);
  EXPECT_DOUBLE_EQ(output.status.stair_hold_until_arc_m, 2.0);
  EXPECT_EQ(output.status.avoidance_cycle_count, 1u);
}

TEST(NavigationModeManagerTest, StairAvoidStaysLatchedBeforeTop)
{
  NavigationModeConfig mode_config;
  mode_config.min_local_avoid_hold_sec = 0.0;
  StairUpConfig stair_config;
  stair_config.exit_confirm_sec = 0.5;
  NavigationModeManager manager(mode_config, stair_config);
  RouteProgress p = progress();
  auto r = robot();
  r.z = 0.32;
  manager.update(task(), r, p, elevation(true, 0.15, 1.5),
      corridor(false), obstacles(), 1.0);

  p.arc_length_m = 0.5;
  manager.update(task(), r, p, elevation(true, 0.15, 2.0),
      corridor(false), obstacles(), 1.5);
  EXPECT_DOUBLE_EQ(manager.status().stair_hold_until_arc_m, 2.0);

  p.arc_length_m = 1.0;
  auto output = manager.update(task(), r, p,
      elevation(false, 0.0, 2.5), corridor(false), obstacles(), 2.0);
  EXPECT_EQ(output.status.mode, NavigationMode::LOCAL_AVOID);
  EXPECT_TRUE(output.status.stair_up_active);
}

TEST(NavigationModeManagerTest, StairAvoidExitsOnFlatTopAfterConfirmation)
{
  NavigationModeConfig mode_config;
  mode_config.min_local_avoid_hold_sec = 0.0;
  StairUpConfig stair_config;
  stair_config.exit_confirm_sec = 0.5;
  NavigationModeManager manager(mode_config, stair_config);
  RouteProgress p = progress();
  auto r = robot();
  r.z = 0.32;
  manager.update(task(), r, p, elevation(true, 0.15, 1.5),
      corridor(false), obstacles(), 1.0);

  p.arc_length_m = 1.35;
  manager.update(task(), r, p, elevation(false, 0.02, 2.85),
      corridor(false), obstacles(), 2.0);
  const auto output = manager.update(task(), robot(), p,
      elevation(false, 0.02, 2.85), corridor(false), obstacles(), 2.5);
  EXPECT_EQ(output.status.mode, NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.status.stair_up_active);
}

}  // namespace
}  // namespace navdog

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
