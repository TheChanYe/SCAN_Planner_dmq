#include "navdog_scan_adapter/scan_route_corridor_evaluator_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog_scan_adapter
{

namespace
{

constexpr double kEpsilon = 1e-9;

}  // namespace

ScanRouteCorridorEvaluator3D::ScanRouteCorridorEvaluator3D(
    const navdog::RouteCorridorConfig& config,
    const std::shared_ptr<InflatedGridQuery3D>& grid)
    : config_(config),
      grid_(grid)
{
}

navdog::RouteCorridorAssessment
ScanRouteCorridorEvaluator3D::evaluate(
    const navdog::NavigationTask& task,
    const navdog::RouteProgress& progress,
    const navdog::RobotState& robot,
    double now_sec) const
{
  navdog::RouteCorridorAssessment assessment;

  // --- Grid readiness ---
  if (!grid_ || !grid_->ready())
    return assessment;

  const double resolution = grid_->resolutionM();
  if (!std::isfinite(resolution) || resolution <= 0.0)
    return assessment;

  const double sample_step = resolution * 0.5;
  if (!std::isfinite(sample_step) || sample_step <= 0.0)
    return assessment;

  // --- Robot validity ---
  if (!robot.valid)
    return assessment;
  if (!std::isfinite(robot.z))
    return assessment;

  // --- Progress validity ---
  if (!progress.valid)
    return assessment;
  if (progress.task_sequence != task.sequence)
    return assessment;

  // --- Configuration ---
  if (!std::isfinite(config_.lookahead_distance_m) ||
      config_.lookahead_distance_m <= 0.0)
    return assessment;

  // --- Initialize assessment ---
  assessment.source =
      navdog::RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  assessment.task_sequence = task.sequence;
  assessment.evaluated_from_arc_length_m =
      progress.arc_length_m;
  assessment.map_resolution_m = resolution;
  assessment.sample_step_m = sample_step;
  assessment.query_z_m = robot.z;
  assessment.map_stamp_sec = grid_->mapStampSec();
  assessment.evaluation_stamp_sec = now_sec;
  assessment.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  assessment.first_blocked_arc_length_m =
      std::numeric_limits<double>::infinity();

  // --- Determine check distance ---
  const double check_distance = std::min(
      config_.lookahead_distance_m,
      progress.remaining_distance_m);

  if (check_distance <= 0.0)
  {
    // Already at goal
    assessment.checked_distance_m = 0.0;
    assessment.samples_checked = 0;
    assessment.blocked = false;
    assessment.valid = true;
    return assessment;
  }

  // --- Build the list of segment endpoints to sample ---
  // We walk from progress.projected_x/y forward through route points.
  //
  // Segments:
  //   [0] projected → points[segment_index + 1]
  //   [1] points[segment_index + 1] → points[segment_index + 2]
  //   ...
  //
  // Each segment is trimmed to the remaining lookahead budget.

  double remaining_budget = check_distance;
  double cumulative_distance = 0.0;  // from progress start

  // Current segment start point
  double cur_x = progress.projected_x;
  double cur_y = progress.projected_y;

  // Helper lambda to query a single point.
  // Returns true if evaluation should continue, false if stopped.
  auto queryPoint = [&](
      double px, double py,
      double seg_yaw,
      double dist_from_start) -> bool
  {
    assessment.samples_checked++;

    InflatedGridQueryResult result =
        grid_->query(px, py, robot.z, seg_yaw);

    switch (result)
    {
      case InflatedGridQueryResult::FREE:
        return true;  // continue

      case InflatedGridQueryResult::OCCUPIED:
        assessment.blocked = true;
        assessment.first_blocked_distance_ahead_m =
            dist_from_start;
        assessment.first_blocked_arc_length_m =
            progress.arc_length_m + dist_from_start;
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = true;
        assessment.out_of_map = false;
        return false;  // stop

      case InflatedGridQueryResult::OUT_OF_MAP:
        assessment.out_of_map = true;
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = true;
        return false;  // stop

      case InflatedGridQueryResult::INVALID:
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = false;
        return false;  // stop
    }

    return false;
  };

  // Process a segment from (sx, sy) to (ex, ey) with a yaw.
  // Samples: start point, step points, end point (trimmed).
  // Returns true if should continue, false if stopped.
  auto processSegment = [&](
      double sx, double sy,
      double ex, double ey,
      double budget) -> bool
  {
    const double dx = ex - sx;
    const double dy = ey - sy;
    const double seg_len = std::hypot(dx, dy);

    if (seg_len < kEpsilon)
      return true;  // degenerate, skip

    const double seg_yaw = std::atan2(dy, dx);
    const double usable_len = std::min(seg_len, budget);

    // --- Query start point ---
    {
      if (!queryPoint(sx, sy, seg_yaw, cumulative_distance))
        return false;
    }

    // --- Query step points ---
    if (sample_step < usable_len)
    {
      const int num_steps =
          static_cast<int>(std::floor(usable_len / sample_step));

      for (int i = 1; i <= num_steps; ++i)
      {
        const double t = (i * sample_step) / seg_len;
        // Don't sample beyond usable_len
        if (t * seg_len > usable_len + kEpsilon)
          break;

        const double px = sx + t * dx;
        const double py = sy + t * dy;
        const double dist = cumulative_distance + t * seg_len;

        if (!queryPoint(px, py, seg_yaw, dist))
          return false;
      }
    }

    // --- Query end point (trimmed to budget) ---
    {
      const double t = usable_len / seg_len;
      const double px = sx + t * dx;
      const double py = sy + t * dy;
      const double dist = cumulative_distance + usable_len;

      if (!queryPoint(px, py, seg_yaw, dist))
        return false;
    }

    cumulative_distance += usable_len;
    return true;
  };

  // --- Single point route ---
  if (task.points.size() == 1)
  {
    const navdog::RoutePoint& target = task.points.back();
    const double dx = target.x - robot.x;
    const double dy = target.y - robot.y;
    const double robot_to_target = std::hypot(dx, dy);

    if (robot_to_target < kEpsilon)
    {
      // Robot at goal
      assessment.checked_distance_m = 0.0;
      assessment.samples_checked = 0;
      assessment.blocked = false;
      assessment.valid = true;
      return assessment;
    }

    const double seg_yaw = std::atan2(dy, dx);
    const double usable_len =
        std::min(robot_to_target, check_distance);

    // Query start (robot position)
    if (!queryPoint(robot.x, robot.y, seg_yaw, 0.0))
    {
      return assessment;
    }

    // Query step points
    if (sample_step < usable_len)
    {
      const int num_steps =
          static_cast<int>(std::floor(usable_len / sample_step));

      for (int i = 1; i <= num_steps; ++i)
      {
        const double t = (i * sample_step) / robot_to_target;
        if (t * robot_to_target > usable_len + kEpsilon)
          break;

        const double px = robot.x + t * dx;
        const double py = robot.y + t * dy;
        const double dist = t * robot_to_target;

        if (!queryPoint(px, py, seg_yaw, dist))
        {
          return assessment;
        }
      }
    }

    // Query end point
    {
      const double t = usable_len / robot_to_target;
      const double px = robot.x + t * dx;
      const double py = robot.y + t * dy;
      const double dist = usable_len;

      if (!queryPoint(px, py, seg_yaw, dist))
      {
        return assessment;
      }
    }

    cumulative_distance = usable_len;
    assessment.checked_distance_m = cumulative_distance;
    assessment.blocked = false;
    assessment.valid = true;
    return assessment;
  }

  // --- Multi-point route ---
  // First segment: projected → points[segment_index + 1]
  if (progress.segment_index + 1 < task.points.size())
  {
    const navdog::RoutePoint& next_pt =
        task.points[progress.segment_index + 1];

    if (!processSegment(
            cur_x, cur_y,
            next_pt.x, next_pt.y,
            remaining_budget))
    {
      return assessment;
    }

    remaining_budget = check_distance - cumulative_distance;

    cur_x = next_pt.x;
    cur_y = next_pt.y;
  }

  // Remaining segments
  for (std::size_t i = progress.segment_index + 2;
       i < task.points.size() && remaining_budget > kEpsilon;
       ++i)
  {
    const navdog::RoutePoint& seg_start = task.points[i - 1];
    const navdog::RoutePoint& seg_end = task.points[i];

    if (!processSegment(
            seg_start.x, seg_start.y,
            seg_end.x, seg_end.y,
            remaining_budget))
    {
      return assessment;
    }

    remaining_budget = check_distance - cumulative_distance;
    cur_x = seg_end.x;
    cur_y = seg_end.y;
  }

  // All segments processed without finding obstacle
  assessment.checked_distance_m = cumulative_distance;
  assessment.blocked = false;
  assessment.valid = true;
  return assessment;
}

}  // namespace navdog_scan_adapter
