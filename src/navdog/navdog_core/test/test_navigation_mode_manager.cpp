#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/navigation_mode_manager.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helpers
// =============================================================================

NavigationTask makeValidTask(std::uint64_t sequence = 1)
{
  NavigationTask task{};
  task.sequence = sequence;
  task.mode = TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;

  RoutePoint p0{};
  p0.x = 0.0;
  p0.y = 0.0;
  task.points.push_back(p0);

  RoutePoint p1{};
  p1.x = 10.0;
  p1.y = 0.0;
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
  robot.yaw = yaw;
  robot.valid = true;
  return robot;
}

RouteProgress makeProgress(
    std::uint64_t sequence,
    double arc_length,
    double lateral_error = 0.0,
    double route_yaw = 0.0)
{
  RouteProgress progress{};
  progress.task_sequence = sequence;
  progress.arc_length_m = arc_length;
  progress.lateral_error_m = lateral_error;
  progress.route_yaw = route_yaw;
  progress.valid = true;
  return progress;
}

RouteCorridorObservationOutput makeClearCorridor(
    std::uint64_t sequence)
{
  RouteCorridorObservationOutput output{};
  output.result =
      RouteCorridorObservationResult::CLEAR;
  output.assessment.source =
      RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  output.assessment.task_sequence = sequence;
  output.assessment.blocked = false;
  output.assessment.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  output.assessment.valid = true;
  return output;
}

RouteCorridorObservationOutput makeBlockedCorridor(
    std::uint64_t sequence,
    double blocked_distance)
{
  RouteCorridorObservationOutput output =
      makeClearCorridor(sequence);
  output.result =
      RouteCorridorObservationResult::BLOCKED;
  output.assessment.blocked = true;
  output.assessment.first_blocked_distance_ahead_m =
      blocked_distance;
  return output;
}

RouteCorridorObservationOutput makeUnavailableCorridor()
{
  RouteCorridorObservationOutput output{};
  output.result =
      RouteCorridorObservationResult::
          WAITING_FOR_OBSERVATION;
  return output;
}

RouteCorridorObservationOutput makeFatalCorridor()
{
  RouteCorridorObservationOutput output{};
  output.result =
      RouteCorridorObservationResult::INVALID_TIME;
  return output;
}

void enterLocalAvoid(
    NavigationModeManager& m,
    double t = 1.0)
{
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  m.update(task, makeRobot(), progress,
           makeBlockedCorridor(1, 0.5), t);
}

void enterRouteRejoin(
    NavigationModeManager& m,
    double t_avoid = 1.0,
    double t_clear = 1.6,
    double t_rejoin = 2.0)
{
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(m, t_avoid);
  m.update(task, makeRobot(), progress,
           makeClearCorridor(1), t_clear);
  m.update(task, makeRobot(), progress,
           makeClearCorridor(1), t_rejoin);
}

// =============================================================================
// Default and config
// =============================================================================

TEST(NavigationModeManagerTest, DefaultStateIsNone)
{
  NavigationModeManager manager;
  EXPECT_EQ(manager.status().mode,
            NavigationMode::NONE);
  EXPECT_FALSE(manager.status().initialized);
  EXPECT_FALSE(manager.status().valid);
}

TEST(NavigationModeManagerTest, ResetReturnsToNone)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.reset();
  EXPECT_EQ(manager.status().mode,
            NavigationMode::NONE);
  EXPECT_FALSE(manager.status().initialized);
}

TEST(NavigationModeManagerTest, RejectsNonFiniteTime)
{
  NavigationModeManager manager;
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(1, 0.0), makeClearCorridor(1),
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_TIME);
}

TEST(NavigationModeManagerTest, RejectsTimeRegression)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.0);
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_TIME);
}

TEST(NavigationModeManagerTest, AllowsEqualTimestamp)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.0);
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_NE(output.result,
            NavigationModeUpdateResult::INVALID_TIME);
}

TEST(NavigationModeManagerTest, RejectsInvalidConfig)
{
  NavigationModeConfig config;
  config.avoid_enter_distance_m = 0.0;
  NavigationModeManager manager(config);
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(1, 0.0), makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_CONFIG);
}

TEST(NavigationModeManagerTest, RejectsInvalidTask)
{
  NavigationModeManager manager;
  NavigationTask task{};
  NavigationModeOutput output = manager.update(
      task, makeRobot(),
      makeProgress(1, 0.0), makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_TASK);
}

TEST(NavigationModeManagerTest, RejectsInvalidProgress)
{
  NavigationModeManager manager;
  RouteProgress progress{};
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      progress, makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_PROGRESS);
}

TEST(NavigationModeManagerTest, RejectsProgressTaskMismatch)
{
  NavigationModeManager manager;
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(2, 0.0), makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_PROGRESS);
}

// =============================================================================
// Initialization
// =============================================================================

TEST(NavigationModeManagerTest, InitializesToRouteFollow)
{
  NavigationModeManager manager;
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(1, 0.0), makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::UPDATED);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_TRUE(output.status.initialized);
  EXPECT_TRUE(output.status.transitioned);
}

TEST(NavigationModeManagerTest, InitializesReferenceIntentToGlobalRoute)
{
  NavigationModeManager manager;
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(1, 0.0), makeClearCorridor(1), 1.0);
  EXPECT_EQ(output.status.reference_intent,
            ReferenceIntent::GLOBAL_ROUTE);
}

TEST(NavigationModeManagerTest, NewTaskSequenceResetsMode)
{
  NavigationModeManager manager;
  NavigationTask task1 = makeValidTask();
  NavigationTask task2 = makeValidTask(2);
  RouteProgress p1 = makeProgress(1, 1.0);
  RouteProgress p2 = makeProgress(2, 0.0);

  manager.update(task1, makeRobot(), p1,
                 makeBlockedCorridor(1, 0.5), 1.0);
  EXPECT_EQ(manager.status().mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(manager.status().avoidance_cycle_count,
            1u);

  NavigationModeOutput output = manager.update(
      task2, makeRobot(), p2,
      makeClearCorridor(2), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.avoidance_cycle_count, 0u);
  EXPECT_FALSE(output.status.has_rejoin_anchor);
}

TEST(NavigationModeManagerTest, SameSequenceMaxVxChangeDoesNotResetMode)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);

  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  task.max_vx = 0.6;
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.status.transitioned);
}

TEST(NavigationModeManagerTest, FirstImmediateBlockCanEnterAvoidSameCycle)
{
  NavigationModeManager manager;
  NavigationModeOutput output = manager.update(
      makeValidTask(), makeRobot(),
      makeProgress(1, 0.0),
      makeBlockedCorridor(1, 0.5), 1.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_IMMEDIATE);
  EXPECT_EQ(output.status.avoidance_cycle_count, 1u);
}

// =============================================================================
// ROUTE_FOLLOW
// =============================================================================

TEST(NavigationModeManagerTest, ClearKeepsRouteFollow)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.status.transitioned);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::ROUTE_CLEAR);
}

TEST(NavigationModeManagerTest, FarBlockedKeepsRouteFollow)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 2.0), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCKED_FAR_AHEAD);
  EXPECT_TRUE(output.status.route_blocked);
  EXPECT_FALSE(output.status.route_blocked_near);
}

TEST(NavigationModeManagerTest, FarBlockedDoesNotStartConfirmTimer)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 2.0), 2.0);

  // Near blocked at t=2.1 — timer should start fresh.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.1);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
  // At t=2.2, elapsed = 0.1 < 0.2, still confirming.
  output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.2);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, NearBlockedStartsConfirmTimer)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.0);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
}

TEST(NavigationModeManagerTest, NearBlockedBeforeConfirmStaysRouteFollow)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
}

TEST(NavigationModeManagerTest, NearBlockedAtExactConfirmBoundaryEntersAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.0);

  // Exact boundary: elapsed = 0.20 = confirm_sec.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.20);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMED);
}

TEST(NavigationModeManagerTest, NearBlockedAfterConfirmEntersAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.21);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, ImmediateBlockedEntersAvoidWithoutDelay)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_IMMEDIATE);
}

TEST(NavigationModeManagerTest, ClearResetsBlockedTimer)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.1);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.2);

  // Elapsed = 0.1 < 0.2 — timer was reset by CLEAR.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.3);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
}

TEST(NavigationModeManagerTest, MissingCorridorResetsBlockedTimer)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.0);
  manager.update(task, makeRobot(), progress,
                 makeUnavailableCorridor(), 2.1);
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.2);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.3);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
}

TEST(NavigationModeManagerTest, RouteOnlyNeverEntersAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  task.mode = TaskMode::ROUTE_ONLY;
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.status.avoidance_allowed);
}

TEST(NavigationModeManagerTest, RouteOnlyReportsBlockedReason)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  task.mode = TaskMode::ROUTE_ONLY;
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::ROUTE_ONLY_BLOCKED);
  EXPECT_TRUE(output.status.route_blocked);
  EXPECT_TRUE(output.status.route_blocked_near);
}

TEST(NavigationModeManagerTest, ChargingAllowsAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  task.mode = TaskMode::CHARGING;
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_TRUE(output.status.avoidance_allowed);
}

// =============================================================================
// LOCAL_AVOID
// =============================================================================

TEST(NavigationModeManagerTest, BlockedKeepsLocalAvoid)
{
  NavigationModeManager manager;
  enterLocalAvoid(manager, 1.0);

  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::LOCAL_AVOID_ACTIVE);
  EXPECT_FALSE(output.status.transitioned);
}

TEST(NavigationModeManagerTest, BlockedResetsClearTimer)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);

  // CLEAR at t=1.6 (start clear timer).
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);
  // BLOCKED at t=1.9 (reset clear timer).
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 0.5), 1.9);
  // CLEAR at t=2.0 (restart clear timer).
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.0);

  // clear_elapsed = 0.3 < 0.4.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.3);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, ClearBeforeMinHoldKeepsLocalAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);

  // CLEAR at t=1.1, mode_elapsed = 0.1 < 0.5.
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.1);

  // mode_elapsed = 0.4 < 0.5.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 1.4);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::CLEAR_CONFIRMING);
}

TEST(NavigationModeManagerTest, ClearBeforeConfirmKeepsLocalAvoid)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);

  // CLEAR at t=1.6, mode_elapsed = 0.6 >= 0.5.
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);

  // clear_elapsed = 0.2 < 0.4.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 1.8);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::CLEAR_CONFIRMING);
}

TEST(NavigationModeManagerTest, ClearAtExactConfirmBoundaryEntersRejoin)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);

  // CLEAR at t=1.6, mode_elapsed = 0.6 >= 0.5.
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);

  // clear_elapsed = 0.4 = route_clear_confirm_sec.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::CLEAR_CONFIRMED);
}

TEST(NavigationModeManagerTest, ClearAfterMinHoldAndConfirmEntersRejoin)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.01);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
}

TEST(NavigationModeManagerTest, MissingCorridorResetsClearEvidence)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);
  // Unavailable resets clear timer.
  manager.update(task, makeRobot(), progress,
                 makeUnavailableCorridor(), 1.7);
  // CLEAR restarts timer.
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.8);

  // clear_elapsed = 0.2 < 0.4.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, EnteringRejoinStoresCurrentArcAnchor)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 2.5);
  enterLocalAvoid(manager, 1.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_TRUE(output.status.has_rejoin_anchor);
  EXPECT_DOUBLE_EQ(
      output.status.rejoin_min_arc_length_m, 2.5);
}

TEST(NavigationModeManagerTest, AvoidanceCountIncrementsOnlyOnEntry)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);
  EXPECT_EQ(manager.status().avoidance_cycle_count, 1u);

  // Stay in LOCAL_AVOID.
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 0.5), 2.0);
  EXPECT_EQ(manager.status().avoidance_cycle_count, 1u);
}

// =============================================================================
// ROUTE_REJOIN
// =============================================================================

TEST(NavigationModeManagerTest, ClearButFarFromRouteKeepsRejoin)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress =
      makeProgress(1, 1.0, 0.5, 0.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::ROUTE_REJOIN_ACTIVE);
}

TEST(NavigationModeManagerTest, ClearButHeadingMisalignedKeepsRejoin)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  // Lateral ok (0.0), heading misaligned.
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);
  RobotState robot = makeRobot(0, 0, 1.0);

  NavigationModeOutput output = manager.update(
      task, robot, progress,
      makeClearCorridor(1), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::ROUTE_REJOIN_ACTIVE);
}

TEST(NavigationModeManagerTest, ClearAndAlignedStartsConfirm)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(0, 0, 0), progress,
      makeClearCorridor(1), 2.1);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::REJOIN_CONFIRMING);
}

TEST(NavigationModeManagerTest, AlignedAtExactBoundaryReturnsRouteFollow)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);
  manager.update(task, makeRobot(0, 0, 0), progress,
                 makeClearCorridor(1), 2.0);

  // rejoin_confirm_sec = 0.30, exact boundary.
  NavigationModeOutput output = manager.update(
      task, makeRobot(0, 0, 0), progress,
      makeClearCorridor(1), 2.30);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::REJOIN_COMPLETE);
}

TEST(NavigationModeManagerTest, AlignmentLossResetsRejoinTimer)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress_ok =
      makeProgress(1, 1.0, 0.0, 0.0);
  RouteProgress progress_bad =
      makeProgress(1, 1.0, 0.5, 0.0);

  // Aligned at t=2.0, start timer.
  manager.update(task, makeRobot(0, 0, 0), progress_ok,
                 makeClearCorridor(1), 2.0);
  // Misaligned at t=2.1, reset timer.
  manager.update(task, makeRobot(0, 0, 0), progress_bad,
                 makeClearCorridor(1), 2.1);
  // Aligned again at t=2.2, restart timer.
  manager.update(task, makeRobot(0, 0, 0), progress_ok,
                 makeClearCorridor(1), 2.2);

  // Elapsed = 0.1 < 0.3, not confirmed.
  NavigationModeOutput output = manager.update(
      task, makeRobot(0, 0, 0), progress_ok,
      makeClearCorridor(1), 2.3);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::REJOIN_CONFIRMING);
}

TEST(NavigationModeManagerTest, InvalidRobotWaitsWithoutFailure)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);
  RobotState robot{};
  // robot.valid = false.

  NavigationModeOutput output = manager.update(
      task, robot, progress,
      makeClearCorridor(1), 2.1);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::WAITING_FOR_ROBOT);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
}

TEST(NavigationModeManagerTest, NonFiniteValidRobotIsRejected)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  RobotState robot = makeRobot();
  robot.x = std::numeric_limits<double>::quiet_NaN();

  NavigationModeOutput output = manager.update(
      task, robot, progress,
      makeClearCorridor(1), 2.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::INVALID_ROBOT);
}

TEST(NavigationModeManagerTest, FarBlockedKeepsRejoin)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 2.0), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::BLOCKED_FAR_AHEAD);
}

TEST(NavigationModeManagerTest, ImmediateBlockReturnsToLocalAvoid)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::REJOIN_BLOCKED);
}

TEST(NavigationModeManagerTest, ConfirmedBlockReturnsToLocalAvoid)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);

  // Near blocked, start confirm timer.
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 1.0), 2.1);

  // Elapsed = 0.20 = confirm_sec.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 1.0), 2.30);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.reason,
            NavigationModeReason::REJOIN_BLOCKED);
}

TEST(NavigationModeManagerTest, SecondAvoidRoundIncrementsCycleCount)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);

  // First avoid round.
  enterLocalAvoid(manager, 1.0);
  EXPECT_EQ(manager.status().avoidance_cycle_count, 1u);

  // Enter rejoin.
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.0);
  EXPECT_EQ(manager.status().mode,
            NavigationMode::ROUTE_REJOIN);

  // Immediate block → second avoid round.
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.status.avoidance_cycle_count, 2u);
}

TEST(NavigationModeManagerTest, RejoinCompleteClearsAnchor)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);
  manager.update(task, makeRobot(0, 0, 0), progress,
                 makeClearCorridor(1), 2.0);
  EXPECT_TRUE(manager.status().has_rejoin_anchor);

  NavigationModeOutput output = manager.update(
      task, makeRobot(0, 0, 0), progress,
      makeClearCorridor(1), 2.30);
  EXPECT_EQ(output.status.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.status.has_rejoin_anchor);
}

// =============================================================================
// Multi-round and jitter
// =============================================================================

TEST(NavigationModeManagerTest, BlockedClearAlternationDoesNotChatter)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  for (int i = 0; i < 5; ++i)
  {
    double t = 2.0 + i * 0.2;
    manager.update(task, makeRobot(), progress,
                   makeBlockedCorridor(1, 1.0), t);
    manager.update(task, makeRobot(), progress,
                   makeClearCorridor(1), t + 0.1);
  }

  EXPECT_EQ(manager.status().mode,
            NavigationMode::ROUTE_FOLLOW);
}

TEST(NavigationModeManagerTest, ClearBlockedDuringAvoidDoesNotEnterRejoin)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);
  enterLocalAvoid(manager, 1.0);

  // CLEAR 0.3s (not enough).
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.9);

  // BLOCKED resets clear timer.
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 0.5), 2.0);

  // CLEAR 0.3s again (not enough).
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.1);
  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeClearCorridor(1), 2.4);

  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, RejoinBlockedReturnsToAvoid)
{
  NavigationModeManager manager;
  enterRouteRejoin(manager);
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0, 0.0, 0.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeBlockedCorridor(1, 0.5), 2.1);
  EXPECT_EQ(output.status.mode,
            NavigationMode::LOCAL_AVOID);
}

TEST(NavigationModeManagerTest, MultipleAvoidRoundsKeepTaskSequence)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 1.0);

  enterLocalAvoid(manager, 1.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.6);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 2.0);

  // Second avoid round.
  manager.update(task, makeRobot(), progress,
                 makeBlockedCorridor(1, 0.5), 2.1);

  EXPECT_EQ(manager.status().task_sequence, 1u);
}

TEST(NavigationModeManagerTest, ModeNeverReturnsToNoneWhileTaskActive)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  // Various corridor inputs.
  manager.update(task, makeRobot(), progress,
                 makeUnavailableCorridor(), 2.0);
  EXPECT_NE(manager.status().mode,
            NavigationMode::NONE);

  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 3.0);
  EXPECT_NE(manager.status().mode,
            NavigationMode::NONE);
}

// =============================================================================
// Fatal corridor
// =============================================================================

TEST(NavigationModeManagerTest, FatalCorridorReturnsInvalidResult)
{
  NavigationModeManager manager;
  NavigationTask task = makeValidTask();
  RouteProgress progress = makeProgress(1, 0.0);
  manager.update(task, makeRobot(), progress,
                 makeClearCorridor(1), 1.0);

  NavigationModeOutput output = manager.update(
      task, makeRobot(), progress,
      makeFatalCorridor(), 2.0);
  EXPECT_EQ(output.result,
            NavigationModeUpdateResult::
                INVALID_CORRIDOR_RESULT);
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
