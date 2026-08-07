#include "navdog_core/route_corridor_observation_gate.hpp"

#include <cmath>
#include <limits>

namespace navdog
{

// =============================================================================
// Constructor
// 构造函数：保存门控配置。
// =============================================================================

RouteCorridorObservationGate::RouteCorridorObservationGate(
    const RouteCorridorObservationConfig& config)
    : config_(config)
{
}

// =============================================================================
// isConfigValid
// 校验配置本身合法性：地图超时阈值必须为正数，最大进度滞后距离必须为非负有限数。
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
// 校验路线进度合法性：valid标志为真、弧长为非负有限数、任务sequence非零、时间戳有限。
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
// 校验走廊观测中所有时间戳/分辨率/采样步长/高度/已检查距离/评估起始弧长等字段
// 是否为合理有限数，并根据 blocked 标志分派校验阻塞距离字段：
// 阻塞时必须非负有限，畅通时必须为正无穷（不能用std::isinf判断，因为负无穷也会命中）。
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
// 依次执行11步合法性检查（任一不通过则直接返回对应失败原因）：
//   1.now_sec有限；2.门控配置合法；3.进度本身合法；4.观测.valid为真；
//   5.观测来源为 SCAN_INFLATED_GRID_3D；6.观测与当前进度的任务sequence匹配；
//   7.观测数值字段合法；8.地图时间戳不超前且未超时（带浮点误差容差）；
//   9.进度弧长与观测评估起始弧长的偏差在允许范围内（既不能超前也不能滞后过多）；
//   10.不能超出地图范围；11.全部通过后根据blocked标志输出CLEAR或BLOCKED并携带详情。
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
