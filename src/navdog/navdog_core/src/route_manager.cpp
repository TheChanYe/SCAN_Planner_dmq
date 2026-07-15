#include "navdog_core/route_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navdog
{

RouteManager::RouteManager(const RouteProgressConfig& config)
    : progress_tracker_(config) {}

void RouteManager::reset() noexcept
{
  task_view_ = NavigationTask{};
  progress_tracker_.reset();
  last_progress_ = RouteProgress{};
}

bool RouteManager::acceptRoute(std::uint64_t sequence,
    const std::vector<navdog_task::RoutePoint>& points)
{
  if (!canAccept(sequence, points)) return false;
  reset();
  task_view_.sequence = sequence;
  task_view_.points = points;
  return true;
}

bool RouteManager::acceptRoute(std::uint64_t sequence,
    std::vector<navdog_task::RoutePoint>&& points)
{
  if (!canAccept(sequence, points)) return false;
  reset();
  task_view_.sequence = sequence;
  task_view_.points = std::move(points);
  return true;
}

bool RouteManager::canAccept(std::uint64_t sequence,
    const std::vector<navdog_task::RoutePoint>& points) const noexcept
{
  if (sequence == 0 || points.empty()) return false;
  for (const auto& point : points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) ||
        (point.has_yaw && !std::isfinite(point.yaw))) return false;
  }
  if (hasRoute() && sequence == task_view_.sequence) return false;
  if (hasRoute() && sequence < task_view_.sequence) return false;
  return true;
}

bool RouteManager::hasRoute() const noexcept
{ return task_view_.sequence != 0 && !task_view_.points.empty(); }

std::uint64_t RouteManager::taskSequence() const noexcept
{ return task_view_.sequence; }

RouteProgressOutput RouteManager::updateProgress(
    const RobotState& robot, double now_sec)
{
  if (!hasRoute()) return RouteProgressOutput{};
  RouteProgressOutput output =
      progress_tracker_.update(task_view_, robot, now_sec);
  if (output.progress.valid)
  {
    // This explicit clamp is the invariant protecting crossings and loops.
    output.progress.arc_length_m = std::max(
        last_progress_.valid ? last_progress_.arc_length_m : 0.0,
        output.progress.arc_length_m);
    output.progress.remaining_distance_m = std::max(
        0.0, output.progress.total_length_m - output.progress.arc_length_m);
    last_progress_ = output.progress;
  }
  return output;
}

bool RouteManager::pointAtArcLength(
    double arc, navdog_task::RoutePoint& output) const noexcept
{
  output = navdog_task::RoutePoint{};
  if (!hasRoute() || !std::isfinite(arc)) return false;
  if (task_view_.points.size() == 1)
  {
    output = task_view_.points.front();
    return true;
  }
  double traversed = 0.0;
  const double target = std::max(0.0, arc);
  for (std::size_t i = 1; i < task_view_.points.size(); ++i)
  {
    const auto& a = task_view_.points[i - 1];
    const auto& b = task_view_.points[i];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length <= 1e-12) continue;
    if (target <= traversed + length)
    {
      const double ratio = std::max(0.0,
          std::min(1.0, (target - traversed) / length));
      output.x = a.x + ratio * (b.x - a.x);
      output.y = a.y + ratio * (b.y - a.y);
      output.z = a.z + ratio * (b.z - a.z);
      output.yaw = std::atan2(b.y - a.y, b.x - a.x);
      output.has_yaw = true;
      if (ratio >= 1.0 && b.has_yaw) output = b;
      return true;
    }
    traversed += length;
  }
  output = task_view_.points.back();
  return true;
}

bool RouteManager::forwardTarget(double from, double distance,
    navdog_task::RoutePoint& output) const noexcept
{
  if (!std::isfinite(from) || !std::isfinite(distance) || distance < 0.0)
    return false;
  return pointAtArcLength(from + distance, output);
}

const navdog_task::RoutePoint* RouteManager::goal() const noexcept
{ return hasRoute() ? &task_view_.points.back() : nullptr; }

const std::vector<navdog_task::RoutePoint>& RouteManager::route() const noexcept
{ return task_view_.points; }

const RouteProgress& RouteManager::progress() const noexcept
{ return last_progress_; }

const NavigationTask& RouteManager::taskView() const noexcept
{ return task_view_; }

}  // namespace navdog
