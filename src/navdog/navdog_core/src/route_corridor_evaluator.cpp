#include "navdog_core/route_corridor_evaluator.hpp"

#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kDistanceTieEpsilon = 1e-12;

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

RouteCorridorEvaluator::RouteCorridorEvaluator(
    const RouteCorridorConfig& corridor_config,
    const RouteProgressConfig& progress_config,
    const SafetyConfig& safety_config)
    : corridor_config_(corridor_config),
      progress_config_(progress_config),
      safety_config_(safety_config)
{
}

// =============================================================================
// clamp
// =============================================================================

double RouteCorridorEvaluator::clamp(
    double value,
    double lower,
    double upper) noexcept
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

// =============================================================================
// isConfigValid
// =============================================================================

bool RouteCorridorEvaluator::isConfigValid() const noexcept
{
  if (!std::isfinite(corridor_config_.lookahead_distance_m) ||
      corridor_config_.lookahead_distance_m <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(corridor_config_.corridor_radius_m) ||
      corridor_config_.corridor_radius_m <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(safety_config_.obstacle_timeout_sec) ||
      safety_config_.obstacle_timeout_sec <= 0.0)
  {
    return false;
  }

  if (!std::isfinite(progress_config_.min_segment_length_m) ||
      progress_config_.min_segment_length_m <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isTaskValid
// =============================================================================

bool RouteCorridorEvaluator::isTaskValid(
    const NavigationTask& task) const noexcept
{
  if (task.sequence == 0)
  {
    return false;
  }

  if (task.points.empty())
  {
    return false;
  }

  for (const RoutePoint& p : task.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// isProgressValid
// =============================================================================

bool RouteCorridorEvaluator::isProgressValid(
    const NavigationTask& task,
    const RouteProgress& progress) const noexcept
{
  if (!progress.valid)
  {
    return false;
  }

  if (progress.task_sequence != task.sequence)
  {
    return false;
  }

  if (!std::isfinite(progress.segment_ratio) ||
      progress.segment_ratio < 0.0 ||
      progress.segment_ratio > 1.0)
  {
    return false;
  }

  if (!std::isfinite(progress.arc_length_m) ||
      progress.arc_length_m < 0.0)
  {
    return false;
  }

  if (!std::isfinite(progress.total_length_m) ||
      progress.total_length_m < 0.0)
  {
    return false;
  }

  if (!std::isfinite(progress.remaining_distance_m) ||
      progress.remaining_distance_m < 0.0)
  {
    return false;
  }

  if (!std::isfinite(progress.projected_x) ||
      !std::isfinite(progress.projected_y))
  {
    return false;
  }

  if (!std::isfinite(progress.route_yaw))
  {
    return false;
  }

  if (!std::isfinite(progress.lateral_error_m))
  {
    return false;
  }

  if (!std::isfinite(progress.stamp_sec))
  {
    return false;
  }

  // Multi-point task additionally requires a valid next point.
  if (task.points.size() >= 2)
  {
    if (progress.segment_index + 1 >= task.points.size())
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// isRobotValid
// =============================================================================

bool RouteCorridorEvaluator::isRobotValid(
    const RobotState& robot) const noexcept
{
  if (!robot.valid)
  {
    return false;
  }

  if (!std::isfinite(robot.x) || !std::isfinite(robot.y))
  {
    return false;
  }

  return true;
}

// =============================================================================
// isObstacleFieldNumericValid
// =============================================================================

bool RouteCorridorEvaluator::isObstacleFieldNumericValid(
    const ObstacleField& obstacle_field) const noexcept
{
  for (const ObstacleCircle& obs : obstacle_field.obstacles)
  {
    if (!std::isfinite(obs.x) || !std::isfinite(obs.y))
    {
      return false;
    }

    if (!std::isfinite(obs.effective_radius_m) ||
        obs.effective_radius_m < 0.0)
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// pointToSegmentDistance
// =============================================================================

RouteCorridorEvaluator::DistanceResult
RouteCorridorEvaluator::pointToSegmentDistance(
    double point_x,
    double point_y,
    const SegmentPiece& segment) const noexcept
{
  const double length_sq =
      segment.dx * segment.dx +
      segment.dy * segment.dy;

  // The caller guarantees the segment is non-degenerate
  // (length >= min_segment_length_m > 0), so length_sq > 0.
  double unclamped_ratio =
      ((point_x - segment.x0) * segment.dx +
       (point_y - segment.y0) * segment.dy) /
      length_sq;

  const double ratio = clamp(unclamped_ratio, 0.0, 1.0);

  const double closest_x =
      segment.x0 + ratio * segment.dx;
  const double closest_y =
      segment.y0 + ratio * segment.dy;

  const double diff_x = point_x - closest_x;
  const double diff_y = point_y - closest_y;

  DistanceResult result;
  result.distance_sq = diff_x * diff_x + diff_y * diff_y;
  result.ratio = ratio;
  result.unclamped_ratio = unclamped_ratio;
  return result;
}

// =============================================================================
// evaluateSegment
// =============================================================================

void RouteCorridorEvaluator::evaluateSegment(
    const SegmentPiece& segment,
    const ObstacleField& obstacle_field,
    RouteCorridorAssessment& assessment) const noexcept
{
  const double corridor_radius =
      corridor_config_.corridor_radius_m;

  for (std::size_t i = 0;
       i < obstacle_field.obstacles.size();
       ++i)
  {
    const ObstacleCircle& obs =
        obstacle_field.obstacles[i];

    const DistanceResult dist =
        pointToSegmentDistance(
            obs.x, obs.y, segment);

    // Skip obstacles that project behind the start of this
    // segment.  On the first segment this corresponds to
    // obstacles behind the current route progress — they
    // must not participate in the corridor evaluation.
    // On subsequent segments such obstacles were already
    // evaluated on a previous segment (or are behind the
    // progress), so skipping them is harmless.
    if (dist.unclamped_ratio < 0.0)
    {
      continue;
    }

    const double required_distance =
        corridor_radius + obs.effective_radius_m;
    const double required_distance_sq =
        required_distance * required_distance;

    const double actual_distance =
        std::sqrt(dist.distance_sq);
    const double clearance =
        actual_distance - required_distance;

    if (clearance < assessment.minimum_clearance_m)
    {
      assessment.minimum_clearance_m = clearance;
    }

    if (dist.distance_sq <= required_distance_sq)
    {
      const double blocked_distance_ahead =
          segment.distance_from_progress_m +
          dist.ratio * segment.length;

      const bool strictly_closer =
          blocked_distance_ahead <
          assessment.first_blocked_distance_ahead_m -
              kDistanceTieEpsilon;

      const bool same_distance =
          !strictly_closer &&
          blocked_distance_ahead <=
          assessment.first_blocked_distance_ahead_m +
              kDistanceTieEpsilon;

      if (strictly_closer)
      {
        assessment.first_blocked_distance_ahead_m =
            blocked_distance_ahead;
        assessment.obstacle_index = i;
        assessment.blocked = true;
      }
      else if (same_distance &&
               i < assessment.obstacle_index)
      {
        assessment.obstacle_index = i;
        assessment.blocked = true;
      }
    }
  }
}

// =============================================================================
// evaluatePolylineRoute
// =============================================================================

RouteCorridorOutput RouteCorridorEvaluator::evaluatePolylineRoute(
    const NavigationTask& task,
    const RouteProgress& progress,
    const ObstacleField& obstacle_field,
    double now_sec) const
{
  RouteCorridorOutput output;
  RouteCorridorAssessment& assessment = output.assessment;

  assessment.task_sequence = task.sequence;
  assessment.stamp_sec = now_sec;
  assessment.valid = true;

  double checked_distance = 0.0;
  const double lookahead =
      corridor_config_.lookahead_distance_m;

  // --- First segment: projected point → next route point ---
  {
    const RoutePoint& next_pt =
        task.points[progress.segment_index + 1];

    SegmentPiece segment;
    segment.x0 = progress.projected_x;
    segment.y0 = progress.projected_y;
    segment.x1 = next_pt.x;
    segment.y1 = next_pt.y;
    segment.dx = segment.x1 - segment.x0;
    segment.dy = segment.y1 - segment.y0;
    segment.length = std::hypot(segment.dx, segment.dy);
    segment.distance_from_progress_m = 0.0;

    if (segment.length >=
        progress_config_.min_segment_length_m)
    {
      const double remaining =
          lookahead - checked_distance;

      if (remaining > 0.0)
      {
        if (segment.length > remaining)
        {
          const double trim_ratio =
              remaining / segment.length;
          segment.x1 =
              segment.x0 + trim_ratio * segment.dx;
          segment.y1 =
              segment.y0 + trim_ratio * segment.dy;
          segment.dx = segment.x1 - segment.x0;
          segment.dy = segment.y1 - segment.y0;
          segment.length = remaining;
        }

        evaluateSegment(
            segment, obstacle_field, assessment);
        checked_distance += segment.length;
      }
    }
  }

  // --- Remaining segments: points[i] → points[i+1] ---
  for (std::size_t i = progress.segment_index + 1;
       i + 1 < task.points.size() &&
       checked_distance < lookahead;
       ++i)
  {
    SegmentPiece segment;
    segment.x0 = task.points[i].x;
    segment.y0 = task.points[i].y;
    segment.x1 = task.points[i + 1].x;
    segment.y1 = task.points[i + 1].y;
    segment.dx = segment.x1 - segment.x0;
    segment.dy = segment.y1 - segment.y0;
    segment.length = std::hypot(segment.dx, segment.dy);
    segment.distance_from_progress_m = checked_distance;

    if (segment.length <
        progress_config_.min_segment_length_m)
    {
      continue;
    }

    const double remaining =
        lookahead - checked_distance;

    if (remaining <= 0.0)
    {
      break;
    }

    if (segment.length > remaining)
    {
      const double trim_ratio =
          remaining / segment.length;
      segment.x1 =
          segment.x0 + trim_ratio * segment.dx;
      segment.y1 =
          segment.y0 + trim_ratio * segment.dy;
      segment.dx = segment.x1 - segment.x0;
      segment.dy = segment.y1 - segment.y0;
      segment.length = remaining;
    }

    evaluateSegment(
        segment, obstacle_field, assessment);
    checked_distance += segment.length;
  }

  assessment.checked_distance_m = checked_distance;

  if (assessment.blocked)
  {
    assessment.first_blocked_arc_length_m =
        progress.arc_length_m +
        assessment.first_blocked_distance_ahead_m;
  }

  output.result = assessment.blocked
      ? RouteCorridorResult::BLOCKED
      : RouteCorridorResult::CLEAR;

  return output;
}

// =============================================================================
// evaluateSinglePointRoute
// =============================================================================

RouteCorridorOutput
RouteCorridorEvaluator::evaluateSinglePointRoute(
    const NavigationTask& task,
    const RouteProgress& progress,
    const RobotState& robot,
    const ObstacleField& obstacle_field,
    double now_sec) const
{
  RouteCorridorOutput output;
  RouteCorridorAssessment& assessment = output.assessment;

  assessment.task_sequence = task.sequence;
  assessment.stamp_sec = now_sec;
  assessment.valid = true;

  const RoutePoint& target = task.points.back();

  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double robot_to_target =
      std::hypot(dx, dy);

  if (robot_to_target <
      progress_config_.min_segment_length_m)
  {
    assessment.checked_distance_m = 0.0;
    output.result = RouteCorridorResult::CLEAR;
    return output;
  }

  const double lookahead =
      corridor_config_.lookahead_distance_m;
  const double check_distance =
      std::min(robot_to_target, lookahead);

  SegmentPiece segment;
  segment.x0 = robot.x;
  segment.y0 = robot.y;
  segment.x1 = robot.x + (dx / robot_to_target) * check_distance;
  segment.y1 = robot.y + (dy / robot_to_target) * check_distance;
  segment.dx = segment.x1 - segment.x0;
  segment.dy = segment.y1 - segment.y0;
  segment.length = check_distance;
  segment.distance_from_progress_m = 0.0;

  evaluateSegment(
      segment, obstacle_field, assessment);

  assessment.checked_distance_m = check_distance;

  if (assessment.blocked)
  {
    assessment.first_blocked_arc_length_m =
        progress.arc_length_m +
        assessment.first_blocked_distance_ahead_m;
  }

  output.result = assessment.blocked
      ? RouteCorridorResult::BLOCKED
      : RouteCorridorResult::CLEAR;

  return output;
}

// =============================================================================
// evaluate
// =============================================================================

RouteCorridorOutput RouteCorridorEvaluator::evaluate(
    const NavigationTask& task,
    const RouteProgress& progress,
    const RobotState& robot,
    const ObstacleField& obstacle_field,
    double now_sec) const
{
  // Step 1: now_sec
  if (!std::isfinite(now_sec))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_TIME;
    return output;
  }

  // Step 2: config
  if (!isConfigValid())
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_CONFIG;
    return output;
  }

  // Step 3: task
  if (!isTaskValid(task))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_TASK;
    return output;
  }

  // Step 4: progress
  if (!isProgressValid(task, progress))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_PROGRESS;
    return output;
  }

  // Step 5: robot
  if (!isRobotValid(robot))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_ROBOT;
    return output;
  }

  // Step 6: obstacle_field.valid
  if (!obstacle_field.valid)
  {
    RouteCorridorOutput output;
    output.result =
        RouteCorridorResult::WAITING_FOR_OBSTACLES;
    return output;
  }

  // Step 7: obstacle_field numeric
  if (!isObstacleFieldNumericValid(obstacle_field))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_OBSTACLES;
    return output;
  }

  // Step 8: obstacle timestamp
  if (!std::isfinite(obstacle_field.stamp_sec))
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::INVALID_OBSTACLES;
    return output;
  }

  if (obstacle_field.stamp_sec > now_sec)
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::FUTURE_OBSTACLES;
    return output;
  }

  const double age_sec =
      now_sec - obstacle_field.stamp_sec;

  if (age_sec > safety_config_.obstacle_timeout_sec)
  {
    RouteCorridorOutput output;
    output.result = RouteCorridorResult::STALE_OBSTACLES;
    return output;
  }

  // Step 9: geometric evaluation
  //
  // Determine whether the route is effectively single-point.
  // A route is single-point if it has only one point, or if
  // all segments from the current progress forward are
  // degenerate (shorter than min_segment_length_m).
  //
  if (task.points.size() == 1)
  {
    return evaluateSinglePointRoute(
        task, progress, robot, obstacle_field, now_sec);
  }

  // Check if there is at least one valid segment ahead.
  bool has_valid_segment = false;

  // First segment: projected → points[segment_index + 1]
  {
    const RoutePoint& next_pt =
        task.points[progress.segment_index + 1];
    const double seg_dx = next_pt.x - progress.projected_x;
    const double seg_dy = next_pt.y - progress.projected_y;
    if (std::hypot(seg_dx, seg_dy) >=
        progress_config_.min_segment_length_m)
    {
      has_valid_segment = true;
    }
  }

  // Remaining segments
  if (!has_valid_segment)
  {
    for (std::size_t i = progress.segment_index + 1;
         i + 1 < task.points.size();
         ++i)
    {
      const double seg_dx =
          task.points[i + 1].x - task.points[i].x;
      const double seg_dy =
          task.points[i + 1].y - task.points[i].y;
      if (std::hypot(seg_dx, seg_dy) >=
          progress_config_.min_segment_length_m)
      {
        has_valid_segment = true;
        break;
      }
    }
  }

  if (!has_valid_segment)
  {
    return evaluateSinglePointRoute(
        task, progress, robot, obstacle_field, now_sec);
  }

  return evaluatePolylineRoute(
      task, progress, obstacle_field, now_sec);
}

}  // namespace navdog
