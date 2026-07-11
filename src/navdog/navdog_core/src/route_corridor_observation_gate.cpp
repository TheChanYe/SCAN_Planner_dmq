#include "navdog_core/route_corridor_observation_gate.hpp"

#include <cmath>
#include <limits>

namespace navdog
{

// =============================================================================
// Constructor
// =============================================================================

RouteCorridorObservationGate::RouteCorridorObservationGate(
    const RouteCorridorObservationConfig& config)
    : config_(config)
{
}

// =============================================================================
// isConfigValid
// =============================================================================

bool RouteCorridorObservationGate::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.map_timeout_sec) ||
      config_.map_timeout_sec <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(config_.max_progress_lag_m) ||
      config_.max_progress_lag_m < 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isProgressValid
// =============================================================================

bool RouteCorridorObservationGate::isProgressValid(
    const RouteProgress& progress) const noexcept
{
  if (!progress.valid)
  {
    return false;
  }

  if (!std::isfinite(progress.arc_length_m) ||
      progress.arc_length_m < 0.0)
  {
    return false;
  }

  if (progress.task_sequence == 0)
  {
    return false;
  }

  if (!std::isfinite(progress.stamp_sec))
  {
    return false;
  }

  return true;
}

// =============================================================================
// isObservationNumericValid
// =============================================================================

bool RouteCorridorObservationGate::isObservationNumericValid(
    const RouteCorridorAssessment& observation) const noexcept
{
  if (!std::isfinite(observation.map_stamp_sec))
  {
    return false;
  }

  if (!std::isfinite(observation.evaluation_stamp_sec))
  {
    return false;
  }

  if (!std::isfinite(observation.map_resolution_m) ||
      observation.map_resolution_m <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(observation.sample_step_m) ||
      observation.sample_step_m <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(observation.query_z_m))
  {
    return false;
  }

  if (!std::isfinite(observation.checked_distance_m) ||
      observation.checked_distance_m < 0.0)
  {
    return false;
  }

  if (!std::isfinite(observation.evaluated_from_arc_length_m) ||
      observation.evaluated_from_arc_length_m < 0.0)
  {
    return false;
  }

  // BLOCKED requires finite, non-negative blocked distances.
  if (observation.blocked)
  {
    if (!std::isfinite(
            observation.first_blocked_distance_ahead_m) ||
        observation.first_blocked_distance_ahead_m < 0.0)
    {
      return false;
    }

    if (!std::isfinite(
            observation.first_blocked_arc_length_m) ||
        observation.first_blocked_arc_length_m < 0.0)
    {
      return false;
    }
  }
  else
  {
    // CLEAR requires positive infinity for both blocked-distance fields.
    // Do not use only std::isinf(), because negative infinity is invalid.
    const double positive_infinity =
        std::numeric_limits<double>::infinity();

    if (observation.first_blocked_distance_ahead_m !=
            positive_infinity ||
        observation.first_blocked_arc_length_m !=
            positive_infinity)
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// evaluate
// =============================================================================

RouteCorridorObservationOutput
RouteCorridorObservationGate::evaluate(
    const RouteProgress& current_progress,
    const RouteCorridorAssessment& observation,
    double now_sec) const
{
  RouteCorridorObservationOutput output;

  // Step 1: now_sec
  if (!std::isfinite(now_sec))
  {
    output.result =
        RouteCorridorObservationResult::INVALID_TIME;
    return output;
  }

  // Step 2: config
  if (!isConfigValid())
  {
    output.result =
        RouteCorridorObservationResult::INVALID_CONFIG;
    return output;
  }

  // Step 3: current_progress
  if (!isProgressValid(current_progress))
  {
    output.result =
        RouteCorridorObservationResult::INVALID_PROGRESS;
    return output;
  }

  // Step 4: observation.valid
  if (!observation.valid)
  {
    output.result =
        RouteCorridorObservationResult::
            WAITING_FOR_OBSERVATION;
    return output;
  }

  // Step 5: source
  if (observation.source !=
      RouteCorridorSource::SCAN_INFLATED_GRID_3D)
  {
    output.result =
        RouteCorridorObservationResult::
            INVALID_OBSERVATION;
    return output;
  }

  // Step 6: task_sequence
  if (observation.task_sequence !=
      current_progress.task_sequence)
  {
    output.result =
        RouteCorridorObservationResult::TASK_MISMATCH;
    return output;
  }

  // Step 7: numeric validation
  if (!isObservationNumericValid(observation))
  {
    output.result =
        RouteCorridorObservationResult::
            INVALID_OBSERVATION;
    return output;
  }

  // Step 8: map timestamp
  if (observation.map_stamp_sec > now_sec)
  {
    output.result =
        RouteCorridorObservationResult::FUTURE_MAP;
    return output;
  }

  const double map_age_sec =
      now_sec - observation.map_stamp_sec;

  // Floating-point subtraction can make an exact timeout boundary
  // slightly larger than the configured value, for example:
  // 10.0 - 9.7 == 0.3000000000000007.
  //
  // Preserve the intended rule:
  // age == timeout is valid;
  // age meaningfully greater than timeout is stale.
  constexpr double kTimeComparisonEpsilonSec = 1e-9;

  if (map_age_sec - config_.map_timeout_sec >
      kTimeComparisonEpsilonSec)
  {
    output.result =
        RouteCorridorObservationResult::STALE_MAP;
    return output;
  }

  // Step 9: progress correlation
  const double progress_lag =
      current_progress.arc_length_m -
      observation.evaluated_from_arc_length_m;

  if (progress_lag < -1e-9)
  {
    output.result =
        RouteCorridorObservationResult::
            FUTURE_PROGRESS;
    return output;
  }

  if (progress_lag > config_.max_progress_lag_m)
  {
    output.result =
        RouteCorridorObservationResult::STALE_PROGRESS;
    return output;
  }

  // Step 10: out_of_map
  if (observation.out_of_map)
  {
    output.result =
        RouteCorridorObservationResult::OUT_OF_MAP;
    return output;
  }

  // Step 11: CLEAR / BLOCKED
  output.assessment = observation;

  output.result = observation.blocked
      ? RouteCorridorObservationResult::BLOCKED
      : RouteCorridorObservationResult::CLEAR;

  return output;
}

}  // namespace navdog
