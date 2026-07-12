#include "navdog_core/rejoin_target_selector.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kSearchStepM = 0.20;

double normalizeAngle(double angle) noexcept
{
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

RejoinTargetSelector::RejoinTargetSelector(
    const RejoinTargetSelectorConfig& config)
    : config_(config)
{
}

// =============================================================================
// evaluateTarget
// =============================================================================

bool RejoinTargetSelector::evaluateTarget(
    const RoutePoint& target,
    const OccupancyQuery3D* occupancy) const noexcept
{
  if (!std::isfinite(target.x) ||
      !std::isfinite(target.y) ||
      !std::isfinite(target.z))
  {
    return false;
  }

  if (!std::isfinite(target.yaw))
    return false;

  if (occupancy)
  {
    if (!occupancy->ready())
      return false;

    const double yaw = target.has_yaw ? target.yaw : 0.0;
    if (!occupancy->isFree(target.x, target.y, target.z, yaw))
      return false;
  }

  return true;
}

// =============================================================================
// interpolateRoutePoint
// =============================================================================

bool RejoinTargetSelector::interpolateRoutePoint(
    const NavigationTask& task,
    double target_arc_length_m,
    RoutePoint& out) const noexcept
{
  out = RoutePoint{};

  const auto& points = task.points;
  if (points.size() < 2)
    return false;

  if (target_arc_length_m <= 0.0)
  {
    out = points.front();
    return true;
  }

  double accumulated = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double seg_len = std::hypot(dx, dy);

    if (seg_len < kEpsilon)
      continue;

    if (accumulated + seg_len >= target_arc_length_m)
    {
      const double ratio =
          (target_arc_length_m - accumulated) / seg_len;
      out.x = points[i - 1].x + ratio * dx;
      out.y = points[i - 1].y + ratio * dy;
      out.z = points[i - 1].z + ratio *
          (points[i].z - points[i - 1].z);

      if (points[i - 1].has_yaw && points[i].has_yaw)
      {
        const double yaw_diff = normalizeAngle(
            points[i].yaw - points[i - 1].yaw);
        out.yaw = normalizeAngle(
            points[i - 1].yaw + ratio * yaw_diff);
        out.has_yaw = true;
      }
      else if (points[i].has_yaw)
      {
        out.yaw = points[i].yaw;
        out.has_yaw = true;
      }
      else if (points[i - 1].has_yaw)
      {
        out.yaw = points[i - 1].yaw;
        out.has_yaw = true;
      }
      else
      {
        out.yaw = std::atan2(dy, dx);
        out.has_yaw = false;
      }

      return true;
    }

    accumulated += seg_len;
  }

  out = points.back();
  return true;
}

// =============================================================================
// select
// =============================================================================

RejoinTargetSelector::Result RejoinTargetSelector::select(
    const NavigationTask& task,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    const RobotState& robot,
    const OccupancyQuery3D* occupancy) const
{
  (void)robot;

  Result result{};

  if (task.points.size() < 2 ||
      !progress.valid ||
      !std::isfinite(progress.arc_length_m) ||
      !std::isfinite(progress.total_length_m) ||
      !mode_status.has_rejoin_anchor ||
      !std::isfinite(mode_status.rejoin_min_arc_length_m))
  {
    return result;
  }

  const double min_arc = std::max(
      mode_status.rejoin_min_arc_length_m,
      progress.arc_length_m + kEpsilon);

  const double search_start = std::max(
      min_arc,
      progress.arc_length_m + config_.min_forward_distance_m);

  const double search_end = std::min(
      progress.total_length_m,
      progress.arc_length_m + config_.max_forward_distance_m);

  if (search_start > search_end)
    return result;

  // Prefer default distance, then search forward/backward in steps.
  const double preferred_arc = std::max(
      min_arc,
      std::min(
          progress.arc_length_m +
              config_.default_forward_distance_m,
          progress.total_length_m));

  // Build candidate arcs: start from preferred, then forward, then backward.
  std::vector<double> candidate_arcs;
  candidate_arcs.reserve(64);

  // Forward sweep from preferred to search_end.
  double arc = preferred_arc;
  while (arc <= search_end + kEpsilon)
  {
    candidate_arcs.push_back(arc);
    arc += kSearchStepM;
  }

  // Backward sweep from preferred - step down to search_start.
  arc = preferred_arc - kSearchStepM;
  while (arc >= search_start - kEpsilon)
  {
    candidate_arcs.push_back(arc);
    arc -= kSearchStepM;
  }

  // Ensure search_start and search_end are included.
  candidate_arcs.push_back(search_start);
  if (search_end > search_start + kEpsilon)
    candidate_arcs.push_back(search_end);

  for (double candidate_arc : candidate_arcs)
  {
    if (candidate_arc <= progress.arc_length_m)
      continue;
    if (candidate_arc < min_arc - kEpsilon)
      continue;
    if (candidate_arc > progress.total_length_m + kEpsilon)
      continue;

    RoutePoint target{};
    if (!interpolateRoutePoint(task, candidate_arc, target))
      continue;

    if (!evaluateTarget(target, occupancy))
      continue;

    result.target = target;
    result.target_arc_length_m = candidate_arc;
    result.valid = true;
    return result;
  }

  return result;
}

}  // namespace navdog
