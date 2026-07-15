#include "navdog_core/route_progress_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

namespace
{
constexpr double kDistanceTieEpsilon = 1e-12;
}  // namespace

// =============================================================================
// Constructor
// =============================================================================

RouteProgressTracker::RouteProgressTracker(
    const RouteProgressConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

void RouteProgressTracker::reset() noexcept
{
  active_task_sequence_ = 0;
  segments_.clear();

  initialized_ = false;
  single_point_route_ = false;

  total_length_m_ = 0.0;
  current_arc_length_m_ = 0.0;
  current_segment_vector_index_ = 0;

  last_progress_ = RouteProgress{};
}

// =============================================================================
// initialized
// =============================================================================

bool RouteProgressTracker::initialized() const noexcept
{
  return initialized_;
}

// =============================================================================
// isConfigValid
// =============================================================================

bool RouteProgressTracker::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.min_segment_length_m) ||
      !std::isfinite(config_.max_forward_search_m) ||
      !std::isfinite(config_.on_route_lateral_tolerance_m))
  {
    return false;
  }

  if (config_.min_segment_length_m <= 0.0 ||
      config_.max_forward_search_m <= 0.0 ||
      config_.on_route_lateral_tolerance_m <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isTaskUsable
// =============================================================================

bool RouteProgressTracker::isTaskUsable(
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
// isRobotUsable
// =============================================================================

bool RouteProgressTracker::isRobotUsable(
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
// rebuildRoute
// =============================================================================

bool RouteProgressTracker::rebuildRoute(
    const NavigationTask& task)
{
  segments_.clear();
  total_length_m_ = 0.0;
  single_point_route_ = false;

  double cumulative = 0.0;

  for (std::size_t i = 0; i + 1 < task.points.size(); ++i)
  {
    const double x0 = task.points[i].x;
    const double y0 = task.points[i].y;
    const double x1 = task.points[i + 1].x;
    const double y1 = task.points[i + 1].y;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double length = std::hypot(dx, dy);

    if (length < config_.min_segment_length_m)
    {
      continue;
    }

    Segment seg{};
    seg.original_index = i;
    seg.x0 = x0;
    seg.y0 = y0;
    seg.x1 = x1;
    seg.y1 = y1;
    seg.dx = dx;
    seg.dy = dy;
    seg.length = length;
    seg.cumulative_start_m = cumulative;

    segments_.push_back(seg);
    cumulative += length;
  }

  if (segments_.empty())
  {
    if (task.points.empty())
    {
      return false;
    }

    single_point_route_ = true;
    total_length_m_ = 0.0;
  }
  else
  {
    single_point_route_ = false;
    total_length_m_ = cumulative;
  }

  active_task_sequence_ = task.sequence;
  initialized_ = false;
  current_arc_length_m_ = 0.0;
  current_segment_vector_index_ = 0;
  last_progress_ = RouteProgress{};

  return true;
}

// =============================================================================
// clamp
// =============================================================================

double RouteProgressTracker::clamp(
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
// projectToSegment
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::projectToSegment(
    const Segment& segment,
    std::size_t segment_vector_index,
    const RobotState& robot,
    double minimum_arc_length_m) const noexcept
{
  ProjectionCandidate candidate{};
  candidate.valid = false;

  const double length_sq =
      segment.dx * segment.dx +
      segment.dy * segment.dy;

  if (length_sq < kDistanceTieEpsilon)
  {
    return candidate;
  }

  double ratio =
      ((robot.x - segment.x0) * segment.dx +
       (robot.y - segment.y0) * segment.dy) /
      length_sq;

  ratio = clamp(ratio, 0.0, 1.0);

  double arc_length_m =
      segment.cumulative_start_m +
      ratio * segment.length;

  // Enforce monotonic progress: do not allow arc length
  // below the minimum for this update.
  if (arc_length_m < minimum_arc_length_m)
  {
    if (minimum_arc_length_m >
        segment.cumulative_start_m + segment.length)
    {
      // The minimum is beyond this segment; clamp to end.
      arc_length_m =
          segment.cumulative_start_m + segment.length;
      ratio = 1.0;
    }
    else
    {
      arc_length_m = minimum_arc_length_m;
      if (segment.length > 0.0)
      {
        ratio = clamp(
            (arc_length_m -
             segment.cumulative_start_m) /
                segment.length,
            0.0,
            1.0);
      }
      else
      {
        ratio = 0.0;
      }
    }
  }

  const double projected_x =
      segment.x0 + ratio * segment.dx;
  const double projected_y =
      segment.y0 + ratio * segment.dy;

  const double ddx = robot.x - projected_x;
  const double ddy = robot.y - projected_y;
  const double distance_sq = ddx * ddx + ddy * ddy;

  const double route_yaw =
      std::atan2(segment.dy, segment.dx);

  candidate.valid = true;
  candidate.segment_vector_index = segment_vector_index;
  candidate.original_segment_index = segment.original_index;
  candidate.ratio = ratio;
  candidate.arc_length_m = arc_length_m;
  candidate.projected_x = projected_x;
  candidate.projected_y = projected_y;
  candidate.distance_sq = distance_sq;
  candidate.route_yaw = route_yaw;

  return candidate;
}

// =============================================================================
// isBetterCandidate
// =============================================================================

bool RouteProgressTracker::isBetterCandidate(
    const ProjectionCandidate& candidate,
    const ProjectionCandidate& best) const noexcept
{
  if (!best.valid)
  {
    return true;
  }

  if (!candidate.valid)
  {
    return false;
  }

  // Prefer closer projection.
  if (candidate.distance_sq <
      best.distance_sq - kDistanceTieEpsilon)
  {
    return true;
  }

  if (candidate.distance_sq >
      best.distance_sq + kDistanceTieEpsilon)
  {
    return false;
  }

  // Tie: prefer smaller arc length (earlier route position).
  return candidate.arc_length_m < best.arc_length_m;
}

// =============================================================================
// findInitialProjection
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::findInitialProjection(
    const RobotState& robot) const noexcept
{
  ProjectionCandidate best{};
  best.valid = false;

  for (std::size_t i = 0; i < segments_.size(); ++i)
  {
    ProjectionCandidate candidate =
        projectToSegment(
            segments_[i],
            i,
            robot,
            0.0);

    if (isBetterCandidate(candidate, best))
    {
      best = candidate;
    }
  }

  return best;
}

// =============================================================================
// findForwardProjection
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::findForwardProjection(
    const RobotState& robot) const noexcept
{
  ProjectionCandidate best{};
  best.valid = false;

  const double max_arc =
      current_arc_length_m_ +
      config_.max_forward_search_m;

  for (std::size_t i = current_segment_vector_index_;
       i < segments_.size();
       ++i)
  {
    const Segment& seg = segments_[i];

    // Skip segments beyond the forward search limit.
    if (seg.cumulative_start_m > max_arc)
    {
      break;
    }

    // Compute minimum arc length for this segment.
    double minimum_arc_length_m = current_arc_length_m_;

    // If this is the current segment, enforce ratio lower
    // bound based on current progress.
    if (i == current_segment_vector_index_ &&
        seg.length > 0.0)
    {
      const double min_ratio =
          (current_arc_length_m_ -
           seg.cumulative_start_m) /
          seg.length;

      if (min_ratio > 0.0)
      {
        minimum_arc_length_m =
            seg.cumulative_start_m +
            min_ratio * seg.length;
      }
    }

    ProjectionCandidate candidate =
        projectToSegment(
            seg,
            i,
            robot,
            minimum_arc_length_m);

    if (isBetterCandidate(candidate, best))
    {
      best = candidate;
    }
  }

  // If no candidate found in forward search, keep current.
  if (!best.valid &&
      current_segment_vector_index_ < segments_.size())
  {
    const Segment& seg =
        segments_[current_segment_vector_index_];

    double minimum_arc_length_m = current_arc_length_m_;

    if (seg.length > 0.0)
    {
      const double min_ratio =
          (current_arc_length_m_ -
           seg.cumulative_start_m) /
          seg.length;

      if (min_ratio > 0.0)
      {
        minimum_arc_length_m =
            seg.cumulative_start_m +
            min_ratio * seg.length;
      }
    }

    best = projectToSegment(
        seg,
        current_segment_vector_index_,
        robot,
        minimum_arc_length_m);
  }

  return best;
}

// =============================================================================
// makeProgress
// =============================================================================

RouteProgress RouteProgressTracker::makeProgress(
    const ProjectionCandidate& candidate,
    const RobotState& /*robot*/,
    double now_sec) const noexcept
{
  RouteProgress progress{};

  progress.task_sequence = active_task_sequence_;

  progress.segment_index =
      candidate.original_segment_index;

  progress.segment_ratio = candidate.ratio;

  progress.arc_length_m = candidate.arc_length_m;

  progress.total_length_m = total_length_m_;

  progress.remaining_distance_m =
      std::max(
          0.0,
          total_length_m_ -
          candidate.arc_length_m);

  progress.projected_x = candidate.projected_x;
  progress.projected_y = candidate.projected_y;

  progress.route_yaw = candidate.route_yaw;

  progress.lateral_error_m =
      std::sqrt(candidate.distance_sq);

  progress.on_route =
      progress.lateral_error_m <=
      config_.on_route_lateral_tolerance_m;

  progress.stamp_sec = now_sec;

  progress.valid = true;

  return progress;
}

// =============================================================================
// makeSinglePointProgress
// =============================================================================

RouteProgress RouteProgressTracker::makeSinglePointProgress(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec) const noexcept
{
  RouteProgress progress{};

  const RoutePoint& target = task.points.back();

  progress.task_sequence = task.sequence;

  progress.segment_index = 0;
  progress.segment_ratio = 0.0;

  progress.arc_length_m = 0.0;
  progress.total_length_m = 0.0;

  progress.projected_x = target.x;
  progress.projected_y = target.y;

  progress.lateral_error_m =
      std::hypot(
          robot.x - target.x,
          robot.y - target.y);

  progress.remaining_distance_m =
      progress.lateral_error_m;

  // Route yaw
  if (target.has_yaw && std::isfinite(target.yaw))
  {
    progress.route_yaw = target.yaw;
  }
  else if (progress.lateral_error_m >
           config_.min_segment_length_m)
  {
    progress.route_yaw =
        std::atan2(
            target.y - robot.y,
            target.x - robot.x);
  }
  else
  {
    progress.route_yaw =
        std::isfinite(robot.yaw) ? robot.yaw : 0.0;
  }

  progress.on_route =
      progress.lateral_error_m <=
      config_.on_route_lateral_tolerance_m;

  progress.stamp_sec = now_sec;

  progress.valid = true;

  return progress;
}

// =============================================================================
// update
// =============================================================================

RouteProgressOutput RouteProgressTracker::update(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec)
{
  RouteProgressOutput output{};

  // 1. Check time
  if (!std::isfinite(now_sec))
  {
    output.result = RouteProgressResult::INVALID_TIME;
    return output;
  }

  // 2. Check config
  if (!isConfigValid())
  {
    output.result = RouteProgressResult::INVALID_CONFIG;
    return output;
  }

  // 3. Check task
  if (!isTaskUsable(task))
  {
    output.result = RouteProgressResult::INVALID_TASK;
    return output;
  }

  // 4. Rebuild route if sequence changed
  if (task.sequence != active_task_sequence_)
  {
    if (!rebuildRoute(task))
    {
      output.result = RouteProgressResult::INVALID_TASK;
      return output;
    }
  }

  // 5. Check robot
  if (!isRobotUsable(robot))
  {
    output.result =
        RouteProgressResult::WAITING_FOR_ROBOT;
    return output;
  }

  // 6. Single point route
  if (single_point_route_)
  {
    RouteProgress progress =
        makeSinglePointProgress(
            task,
            robot,
            now_sec);

    initialized_ = true;
    last_progress_ = progress;

    output.result = RouteProgressResult::VALID;
    output.progress = progress;
    return output;
  }

  // 7. Projection
  ProjectionCandidate candidate{};

  if (!initialized_)
  {
    candidate = findInitialProjection(robot);
  }
  else
  {
    candidate = findForwardProjection(robot);
  }

  if (!candidate.valid)
  {
    output.result = RouteProgressResult::INVALID_TASK;
    return output;
  }

  // 8. Enforce monotonic progress
  // Update internal state. This explicit clamp is the central invariant:
  // closest-point changes at crossings or loops can never move progress back.
  current_arc_length_m_ =
      std::max(current_arc_length_m_, candidate.arc_length_m);
  candidate.arc_length_m = current_arc_length_m_;
  current_segment_vector_index_ =
      candidate.segment_vector_index;
  initialized_ = true;

  // 9. Generate RouteProgress
  RouteProgress progress =
      makeProgress(candidate, robot, now_sec);

  last_progress_ = progress;

  output.result = RouteProgressResult::VALID;
  output.progress = progress;

  return output;
}

}  // namespace navdog
