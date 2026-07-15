#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/route_progress_tracker.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>
#include <vector>

namespace navdog
{

class RouteManager
{
public:
  explicit RouteManager(const RouteProgressConfig& config = RouteProgressConfig{});

  void reset() noexcept;
  bool acceptRoute(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points);
  bool acceptRoute(std::uint64_t task_sequence,
      std::vector<navdog_task::RoutePoint>&& points);
  bool hasRoute() const noexcept;
  std::uint64_t taskSequence() const noexcept;
  RouteProgressOutput updateProgress(const RobotState& robot, double now_sec);
  bool pointAtArcLength(double arc_length_m,
      navdog_task::RoutePoint& output) const noexcept;
  bool forwardTarget(double from_arc_length_m, double forward_distance_m,
      navdog_task::RoutePoint& output) const noexcept;
  const navdog_task::RoutePoint* goal() const noexcept;
  const std::vector<navdog_task::RoutePoint>& route() const noexcept;
  const RouteProgress& progress() const noexcept;
  // Read-only compatibility view; the vector storage remains owned here.
  const NavigationTask& taskView() const noexcept;

private:
  bool canAccept(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points) const noexcept;
  NavigationTask task_view_{};
  RouteProgressTracker progress_tracker_{};
  RouteProgress last_progress_{};
};

}  // namespace navdog
