#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// RejoinTargetSelector
//
// 在原始路线前方选择安全的接回目标点。
// =============================================================================

class RejoinTargetSelector
{
public:
  explicit RejoinTargetSelector(
      const RejoinTargetSelectorConfig& config);

  struct Result
  {
    RoutePoint target{};
    double target_arc_length_m{0.0};
    bool valid{false};
  };

  Result select(
      const NavigationTask& task,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      const RobotState& robot,
      const OccupancyQuery3D* occupancy) const;

private:
  bool evaluateTarget(
      const RoutePoint& target,
      const OccupancyQuery3D* occupancy) const noexcept;

  bool interpolateRoutePoint(
      const NavigationTask& task,
      double target_arc_length_m,
      RoutePoint& out) const noexcept;

  RejoinTargetSelectorConfig config_{};
};

}  // namespace navdog
