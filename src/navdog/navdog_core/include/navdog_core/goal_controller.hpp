#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// GoalController
//
// 近终点减速、到位对齐、完成判定。
// =============================================================================

class GoalController
{
public:
  explicit GoalController(
      const GoalControllerConfig& config);

  void reset() noexcept;

  struct Result
  {
    VelocityCommand command{};
    bool finished{false};
  };

  Result update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double max_yaw_rate,
      double now_sec);

  bool isNearGoal(
      const RouteProgress& progress) const noexcept;

private:
  RoutePoint finalGoal() const noexcept;

  GoalControllerConfig config_{};
};

}  // namespace navdog
