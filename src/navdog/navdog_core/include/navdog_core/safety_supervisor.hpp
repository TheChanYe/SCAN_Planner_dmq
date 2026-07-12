#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// SafetySupervisor
//
// 所有速度命令必须经过此层。
// 实现前障减速/停止、动态限速、超时保护、NaN 保护。
// =============================================================================

class SafetySupervisor
{
public:
  explicit SafetySupervisor(
      const SafetyConfig& config,
      const LimitConfig& limits);

  struct Context
  {
    RobotState robot{};
    ObstacleSummary obstacles{};
    RouteCorridorAssessment corridor{};
    LocalTrajectory trajectory{};
    double map_stamp_sec{0.0};
    bool map_valid{false};
  };

  VelocityCommand apply(
      const VelocityCommand& raw_cmd,
      const Context& context,
      double max_vx,
      double now_sec);

private:
  bool checkTimeouts(
      const Context& context,
      double now_sec) const noexcept;

  double computeFrontSpeedLimit(
      const ObstacleSummary& obstacles) const noexcept;

  double computeYawRateSpeedPenalty(
      double yaw_rate_cmd,
      double max_vx) const noexcept;

  SafetyConfig safety_config_{};
  LimitConfig limit_config_{};
};

}  // namespace navdog
