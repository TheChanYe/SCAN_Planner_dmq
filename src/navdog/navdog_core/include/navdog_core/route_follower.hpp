#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// RouteFollower
//
// 沿着原始 NavigationTask 路线生成速度命令。
// 全向机器人支持 vy。
// =============================================================================

class RouteFollower
{
public:
  explicit RouteFollower(
      const RouteFollowerConfig& config);

  VelocityCommand update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double now_sec);

private:
  VelocityCommand updatePointGoal(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double now_sec) const;

  bool interpolateRoutePoint(
      const NavigationTask& task,
      double target_arc_length_m,
      double& out_x,
      double& out_y,
      double& out_yaw) const noexcept;

  bool isYawAligned(double heading_error) const noexcept;

  RouteFollowerConfig config_{};
};

}  // namespace navdog
