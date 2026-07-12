#include "navdog_core/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

SafetySupervisor::SafetySupervisor(
    const SafetyConfig& config,
    const LimitConfig& limits)
    : safety_config_(config),
      limit_config_(limits)
{
}

// =============================================================================
// checkTimeouts
// =============================================================================

bool SafetySupervisor::checkTimeouts(
    const Context& context,
    double now_sec) const noexcept
{
  if (!std::isfinite(now_sec))
    return false;

  if (!context.robot.valid ||
      !std::isfinite(context.robot.stamp_sec))
  {
    return false;
  }

  if (now_sec - context.robot.stamp_sec >
      safety_config_.odom_timeout_sec)
  {
    return false;
  }

  if (context.obstacles.valid &&
      std::isfinite(context.obstacles.stamp_sec) &&
      now_sec - context.obstacles.stamp_sec >
          safety_config_.obstacle_timeout_sec)
  {
    return false;
  }

  if (context.trajectory.valid)
  {
    if (!std::isfinite(context.trajectory.source_stamp_sec))
      return false;

    if (now_sec - context.trajectory.source_stamp_sec >
        safety_config_.planner_cmd_timeout_sec)
    {
      return false;
    }
  }

  if (context.map_valid &&
      std::isfinite(context.map_stamp_sec) &&
      now_sec - context.map_stamp_sec >
          safety_config_.obstacle_timeout_sec)
  {
    return false;
  }

  return true;
}

// =============================================================================
// computeFrontSpeedLimit
// =============================================================================

double SafetySupervisor::computeFrontSpeedLimit(
    const ObstacleSummary& obstacles) const noexcept
{
  if (!obstacles.valid ||
      !std::isfinite(obstacles.front_min))
  {
    return std::numeric_limits<double>::infinity();
  }

  const double d = obstacles.front_min;

  if (d <= safety_config_.emergency_stop)
    return 0.0;

  if (d >= safety_config_.slow_down_front)
    return std::numeric_limits<double>::infinity();

  const double range =
      safety_config_.slow_down_front -
      safety_config_.emergency_stop;

  if (range < kEpsilon)
    return 0.0;

  return (d - safety_config_.emergency_stop) / range;
}

// =============================================================================
// computeYawRateSpeedPenalty
// =============================================================================

double SafetySupervisor::computeYawRateSpeedPenalty(
    double yaw_rate_cmd,
    double max_vx) const noexcept
{
  if (!std::isfinite(yaw_rate_cmd) ||
      !std::isfinite(max_vx) ||
      max_vx < kEpsilon ||
      limit_config_.max_yaw_rate < kEpsilon)
  {
    return 1.0;
  }

  const double ratio =
      std::abs(yaw_rate_cmd) / limit_config_.max_yaw_rate;

  return 1.0 - ratio;
}

// =============================================================================
// apply
// =============================================================================

VelocityCommand SafetySupervisor::apply(
    const VelocityCommand& raw_cmd,
    const Context& context,
    double max_vx,
    double now_sec)
{
  VelocityCommand cmd{};
  cmd.stamp_sec = now_sec;

  // NaN/Inf guard.
  const bool raw_finite =
      std::isfinite(raw_cmd.vx) &&
      std::isfinite(raw_cmd.vy) &&
      std::isfinite(raw_cmd.yaw_rate);

  const bool timeouts_ok = checkTimeouts(context, now_sec);

  if (!raw_finite || !timeouts_ok)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.valid = true;
    cmd.source = CommandSource::SAFETY_STOP;
    return cmd;
  }

  double limited_vx = raw_cmd.vx;
  double limited_vy = raw_cmd.vy;
  double limited_w = raw_cmd.yaw_rate;

  // Dynamic max_vx cap.
  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, limit_config_.max_vx));

  // Front obstacle slowdown.
  const double front_limit =
      computeFrontSpeedLimit(context.obstacles);

  if (std::isfinite(front_limit) && front_limit < 1.0)
  {
    limited_vx = std::min(limited_vx, front_limit * effective_max_vx);
    limited_vx = std::max(0.0, limited_vx);
  }

  // Yaw rate penalty on forward speed.
  const double yaw_penalty =
      computeYawRateSpeedPenalty(limited_w, effective_max_vx);

  limited_vx = std::min(
      limited_vx, yaw_penalty * effective_max_vx);
  limited_vx = std::max(0.0, limited_vx);

  // Lateral limit.
  const double effective_max_vy =
      std::max(0.0, limit_config_.max_vy);
  limited_vy = clamp(limited_vy, -effective_max_vy, effective_max_vy);

  // Yaw rate limit.
  const double effective_max_w =
      std::max(0.0,
          std::min(limit_config_.max_yaw_rate, 0.65));
  limited_w = clamp(limited_w, -effective_max_w, effective_max_w);

  // Hard forward stop when emergency distance breached.
  if (context.obstacles.valid &&
      std::isfinite(context.obstacles.front_min) &&
      context.obstacles.front_min <=
          safety_config_.emergency_stop)
  {
    limited_vx = 0.0;
  }

  cmd.vx = limited_vx;
  cmd.vy = limited_vy;
  cmd.yaw_rate = limited_w;
  cmd.valid = true;

  if (cmd.vx < kEpsilon)
    cmd.vx = 0.0;
  if (std::abs(cmd.vy) < kEpsilon)
    cmd.vy = 0.0;
  if (std::abs(cmd.yaw_rate) < kEpsilon)
    cmd.yaw_rate = 0.0;

  cmd.source =
      (cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.yaw_rate == 0.0)
          ? CommandSource::SAFETY_STOP
          : CommandSource::SAFETY_SLOW;

  return cmd;
}

}  // namespace navdog
