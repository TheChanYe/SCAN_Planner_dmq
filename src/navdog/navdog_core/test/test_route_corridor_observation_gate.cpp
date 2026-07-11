#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/route_corridor_observation_gate.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helper functions
// =============================================================================

RouteProgress makeProgress(
    std::uint64_t sequence,
    double arc_length_m)
{
  RouteProgress progress;
  progress.task_sequence = sequence;
  progress.segment_index = 0;
  progress.segment_ratio = 0.0;
  progress.arc_length_m = arc_length_m;
  progress.total_length_m = 10.0;
  progress.remaining_distance_m = 10.0 - arc_length_m;
  progress.projected_x = arc_length_m;
  progress.projected_y = 0.0;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;
  return progress;
}

RouteCorridorAssessment makeClearObservation(
    std::uint64_t sequence,
    double now_sec)
{
  RouteCorridorAssessment obs;
  obs.source = RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  obs.task_sequence = sequence;
  obs.blocked = false;
  obs.evaluated_from_arc_length_m = 2.0;
  obs.checked_distance_m = 3.0;
  obs.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  obs.first_blocked_arc_length_m =
      std::numeric_limits<double>::infinity();
  obs.map_resolution_m = 0.10;
  obs.sample_step_m = 0.05;
  obs.query_z_m = 0.4;
  obs.samples_checked = 60;
  obs.out_of_map = false;
  obs.map_stamp_sec = now_sec;
  obs.evaluation_stamp_sec = now_sec;
  obs.valid = true;
  return obs;
}

RouteCorridorAssessment makeBlockedObservation(
    std::uint64_t sequence,
    double now_sec)
{
  RouteCorridorAssessment obs =
      makeClearObservation(sequence, now_sec);
  obs.blocked = true;
  obs.first_blocked_distance_ahead_m = 1.5;
  obs.first_blocked_arc_length_m = 3.5;
  return obs;
}

const double kNow = 10.0;

// =============================================================================
// DefaultOutputIsIdle
// =============================================================================

TEST(RouteCorridorObservationGateTest, DefaultOutputIsIdle)
{
  RouteCorridorObservationOutput output;
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::IDLE);
  EXPECT_FALSE(output.assessment.valid);
}

// =============================================================================
// RejectsNonFiniteNow
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsNonFiniteNow)
{
  RouteCorridorObservationGate gate;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      makeClearObservation(1, kNow),
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::INVALID_TIME);
}

// =============================================================================
// RejectsInvalidConfig
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsInvalidConfig)
{
  RouteCorridorObservationConfig config;
  config.map_timeout_sec = 0.0;
  RouteCorridorObservationGate gate(config);
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      makeClearObservation(1, kNow),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::INVALID_CONFIG);
}

// =============================================================================
// RejectsInvalidProgress
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsInvalidProgress)
{
  RouteCorridorObservationGate gate;
  RouteProgress progress = makeProgress(1, 2.0);
  progress.valid = false;
  RouteCorridorObservationOutput output = gate.evaluate(
      progress,
      makeClearObservation(1, kNow),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::INVALID_PROGRESS);
}

// =============================================================================
// WaitsForMissingObservation
// =============================================================================

TEST(RouteCorridorObservationGateTest, WaitsForMissingObservation)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs;
  // obs.valid defaults to false
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                WAITING_FOR_OBSERVATION);
}

// =============================================================================
// AcceptsClearScanObservation
// =============================================================================

TEST(RouteCorridorObservationGateTest, AcceptsClearScanObservation)
{
  RouteCorridorObservationGate gate;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      makeClearObservation(1, kNow),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::CLEAR);
  EXPECT_TRUE(output.assessment.valid);
  EXPECT_FALSE(output.assessment.blocked);
}

// =============================================================================
// AcceptsBlockedScanObservation
// =============================================================================

TEST(RouteCorridorObservationGateTest, AcceptsBlockedScanObservation)
{
  RouteCorridorObservationGate gate;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      makeBlockedObservation(1, kNow),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::BLOCKED);
  EXPECT_TRUE(output.assessment.blocked);
}

// =============================================================================
// RejectsWrongSource
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsWrongSource)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.source = RouteCorridorSource::NONE;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// RejectsTaskMismatch
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsTaskMismatch)
{
  RouteCorridorObservationGate gate;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      makeClearObservation(999, kNow),
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::TASK_MISMATCH);
}

// =============================================================================
// RejectsStaleMap
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsStaleMap)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.map_stamp_sec = kNow - 0.31;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::STALE_MAP);
}

// =============================================================================
// RejectsFutureMap
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsFutureMap)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.map_stamp_sec = kNow + 0.1;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::FUTURE_MAP);
}

// =============================================================================
// AcceptsMapAtExactTimeout
// =============================================================================

TEST(RouteCorridorObservationGateTest, AcceptsMapAtExactTimeout)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  // map_timeout_sec default = 0.30
  obs.map_stamp_sec = kNow - 0.30;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::CLEAR);
}

// =============================================================================
// RejectsStaleProgress
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsStaleProgress)
{
  RouteCorridorObservationGate gate;
  // max_progress_lag_m default = 0.50
  // progress.arc_length = 5.0, obs.evaluated_from = 2.0
  // lag = 3.0 > 0.50
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.evaluated_from_arc_length_m = 2.0;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 5.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::STALE_PROGRESS);
}

// =============================================================================
// RejectsFutureProgress
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsFutureProgress)
{
  RouteCorridorObservationGate gate;
  // progress.arc_length = 1.0, obs.evaluated_from = 2.0
  // lag = -1.0 < -1e-9
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.evaluated_from_arc_length_m = 2.0;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 1.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                FUTURE_PROGRESS);
}

// =============================================================================
// RejectsOutOfMap
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsOutOfMap)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.out_of_map = true;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::OUT_OF_MAP);
}

// =============================================================================
// RejectsNaNFields
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsNaNFields)
{
  RouteCorridorObservationGate gate;

  {
    RouteCorridorAssessment obs =
        makeClearObservation(1, kNow);
    obs.map_stamp_sec =
        std::numeric_limits<double>::quiet_NaN();
    RouteCorridorObservationOutput output = gate.evaluate(
        makeProgress(1, 2.0), obs, kNow);
    EXPECT_EQ(output.result,
              RouteCorridorObservationResult::
                  INVALID_OBSERVATION);
  }

  {
    RouteCorridorAssessment obs =
        makeClearObservation(1, kNow);
    obs.evaluation_stamp_sec =
        std::numeric_limits<double>::quiet_NaN();
    RouteCorridorObservationOutput output = gate.evaluate(
        makeProgress(1, 2.0), obs, kNow);
    EXPECT_EQ(output.result,
              RouteCorridorObservationResult::
                  INVALID_OBSERVATION);
  }

  {
    RouteCorridorAssessment obs =
        makeClearObservation(1, kNow);
    obs.checked_distance_m =
        std::numeric_limits<double>::quiet_NaN();
    RouteCorridorObservationOutput output = gate.evaluate(
        makeProgress(1, 2.0), obs, kNow);
    EXPECT_EQ(output.result,
              RouteCorridorObservationResult::
                  INVALID_OBSERVATION);
  }
}

// =============================================================================
// RejectsInvalidResolution
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsInvalidResolution)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.map_resolution_m = 0.0;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// RejectsInvalidSampleStep
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsInvalidSampleStep)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.sample_step_m = -0.1;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// RejectsInvalidQueryHeight
// =============================================================================

TEST(RouteCorridorObservationGateTest, RejectsInvalidQueryHeight)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.query_z_m =
      std::numeric_limits<double>::quiet_NaN();
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// BlockedRequiresFiniteBlockedDistances
// =============================================================================

TEST(RouteCorridorObservationGateTest, BlockedRequiresFiniteBlockedDistances)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeBlockedObservation(1, kNow);
  // Reset to infinity despite blocked=true
  obs.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// ClearRequiresInfiniteBlockedDistances
// =============================================================================

TEST(RouteCorridorObservationGateTest, ClearRequiresInfiniteBlockedDistances)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  // Set finite despite blocked=false
  obs.first_blocked_distance_ahead_m = 1.0;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::
                INVALID_OBSERVATION);
}

// =============================================================================
// AcceptsProgressAtExactLagBoundary
// =============================================================================

TEST(RouteCorridorObservationGateTest, AcceptsProgressAtExactLagBoundary)
{
  RouteCorridorObservationGate gate;
  // max_progress_lag_m = 0.50
  // progress.arc_length = 2.5, obs.evaluated_from = 2.0
  // lag = 0.50 == boundary → valid
  RouteCorridorAssessment obs =
      makeClearObservation(1, kNow);
  obs.evaluated_from_arc_length_m = 2.0;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.5),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::CLEAR);
}

// =============================================================================
// CopiesAssessmentOnSuccess
// =============================================================================

TEST(RouteCorridorObservationGateTest, CopiesAssessmentOnSuccess)
{
  RouteCorridorObservationGate gate;
  RouteCorridorAssessment obs =
      makeBlockedObservation(1, kNow);
  obs.first_blocked_distance_ahead_m = 1.5;
  obs.first_blocked_arc_length_m = 3.5;
  obs.samples_checked = 42;
  RouteCorridorObservationOutput output = gate.evaluate(
      makeProgress(1, 2.0),
      obs,
      kNow);
  EXPECT_EQ(output.result,
            RouteCorridorObservationResult::BLOCKED);
  EXPECT_EQ(output.assessment.samples_checked, 42u);
  EXPECT_NEAR(
      output.assessment.first_blocked_distance_ahead_m,
      1.5, 1e-9);
  EXPECT_NEAR(
      output.assessment.first_blocked_arc_length_m,
      3.5, 1e-9);
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
