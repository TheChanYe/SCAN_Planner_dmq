#include "navdog_core/rejoin_target_selector.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

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
// isRouteYawAcceptable
// =============================================================================

bool RejoinTargetSelector::isRouteYawAcceptable(
    double route_yaw,
    double target_yaw) const noexcept
{
  if (!std::isfinite(route_yaw) || !std::isfinite(target_yaw))
    return false;

  const double diff = std::abs(normalizeAngle(target_yaw - route_yaw));
  return diff <= config_.route_yaw_tolerance_rad;
}

// =============================================================================
// evaluateTarget
// =============================================================================

bool RejoinTargetSelector::evaluateTarget(
    const RoutePoint& target,
    const RobotState& robot,
    const OccupancyQuery3D* occupancy) const noexcept
{
  if (!std::isfinite(target.x) ||
      !std::isfinite(target.y) ||
      !std::isfinite(target.z))
  {
    return false;
  }

  if (occupancy && occupancy->ready())
  {
    const double yaw = target.has_yaw ? target.yaw : robot.yaw;
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
  Result result{};

  if (task.points.size() < 2 ||
      !progress.valid ||
      !std::isfinite(progress.arc_length_m) ||
      !mode_status.has_rejoin_anchor ||
      !std::isfinite(mode_status.rejoin_min_arc_length_m))
  {
    return result;
  }

  const double min_arc = std::max(
      mode_status.rejoin_min_arc_length_m,
      progress.arc_length_m + kEpsilon);

  const double preferred_arc =
      progress.arc_length_m +
      config_.default_forward_distance_m;

  const double target_arc = std::max(
      min_arc,
      std::min(
          preferred_arc,
          progress.arc_length_m +
              config_.max_forward_distance_m));

  RoutePoint target{};
  if (!interpolateRoutePoint(task, target_arc, target))
    return result;

  if (!isRouteYawAcceptable(progress.route_yaw, target.yaw))
  {
    // Try to adjust by searching forward for a yaw-consistent point.
    bool adjusted = false;
    for (double delta = config_.min_forward_distance_m;
         delta <= config_.max_forward_distance_m;
         delta += 0.2)
    {
      const double candidate_arc =
          progress.arc_length_m + delta;
      if (candidate_arc <= min_arc)
        continue;

      RoutePoint candidate{};
      if (!interpolateRoutePoint(task, candidate_arc, candidate))
        break;

      if (isRouteYawAcceptable(progress.route_yaw, candidate.yaw) &&
          evaluateTarget(candidate, robot, occupancy))
      {
        target = candidate;
        adjusted = true;
        break;
      }
    }

    if (!adjusted)
      return result;
  }

  if (!evaluateTarget(target, robot, occupancy))
    return result;

  result.target = target;
  result.valid = true;
  return result;
}

}  // namespace navdog
