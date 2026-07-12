#include "navdog_core/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
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
  // Initialize previous output so the first call applies acceleration
  // limits relative to zero rather than allowing an instant jump.
  previous_output_ = VelocityCommand{};
  previous_output_.valid = true;
  previous_stamp_sec_ = 0.0;
  has_previous_output_ = true;
}

// =============================================================================
// reset
// =============================================================================

void SafetySupervisor::reset() noexcept
{
  previous_output_ = VelocityCommand{};
  previous_stamp_sec_ = 0.0;
  has_previous_output_ = false;
}

// =============================================================================
// safetyStop
// =============================================================================

VelocityCommand SafetySupervisor::safetyStop(
    double now_sec,
    CommandSource source) const noexcept
{
  VelocityCommand cmd{};
  cmd.vx = 0.0;
  cmd.vy = 0.0;
  cmd.yaw_rate = 0.0;
  cmd.stamp_sec = std::isfinite(now_sec) ? now_sec : 0.0;
  cmd.valid = true;
  cmd.source = source;
  return cmd;
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

  if (!context.robot.valid)
  {
    return false;
  }

  if (std::isfinite(context.robot.stamp_sec) &&
      now_sec - context.robot.stamp_sec >
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
// checkTrajectoryIdentity
// =============================================================================

bool SafetySupervisor::checkTrajectoryIdentity(
    const Context& context) const noexcept
{
  if (!context.trajectory.valid)
    return true;  // No trajectory to validate.

  if (!std::isfinite(context.trajectory.duration_sec) ||
      context.trajectory.duration_sec <= 0.0)
  {
    return false;
  }

  // If the caller provided a trajectory, it must match the active identity.
  // When not tracking a local trajectory, callers should leave trajectory.valid
  // as false.
  if (context.trajectory.purpose == NavigationMode::NONE ||
      context.trajectory.task_sequence == 0)
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
// shouldApplyAccelerationLimit
// =============================================================================

bool SafetySupervisor::shouldApplyAccelerationLimit(
    const VelocityCommand& raw_cmd,
    const Context& context) const noexcept
{
  // Do not limit deceleration during safety stops or invalid commands.
  if (!raw_cmd.valid)
    return false;

  if (raw_cmd.source == CommandSource::SAFETY_STOP ||
      raw_cmd.source == CommandSource::FAILED_STOP ||
      raw_cmd.source == CommandSource::PAUSE_STOP ||
      raw_cmd.source == CommandSource::CANCEL_STOP)
  {
    return false;
  }

  if (!context.map_valid)
    return false;

  return true;
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

  // Invalid raw command -> stop.
  if (!raw_cmd.valid)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // NaN/Inf guard.
  const bool raw_finite =
      std::isfinite(raw_cmd.vx) &&
      std::isfinite(raw_cmd.vy) &&
      std::isfinite(raw_cmd.yaw_rate);

  if (!raw_finite)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // Timeout / map validity guard.
  const bool timeouts_ok = checkTimeouts(context, now_sec);
  if (!timeouts_ok)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  if (!context.map_valid)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // Local trajectory identity guard.
  if (!checkTrajectoryIdentity(context))
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
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
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // Acceleration limits.
  if (shouldApplyAccelerationLimit(raw_cmd, context) &&
      has_previous_output_ &&
      std::isfinite(previous_stamp_sec_))
  {
    const double dt = now_sec - previous_stamp_sec_;
    if (dt > 0.0 && std::isfinite(dt))
    {
      const double max_dvx = limit_config_.max_accel_x * dt;
      const double max_dvy = limit_config_.max_accel_y * dt;
      const double max_dw = limit_config_.max_accel_yaw * dt;

      limited_vx = clamp(
          limited_vx,
          previous_output_.vx - max_dvx,
          previous_output_.vx + max_dvx);
      limited_vy = clamp(
          limited_vy,
          previous_output_.vy - max_dvy,
          previous_output_.vy + max_dvy);
      limited_w = clamp(
          limited_w,
          previous_output_.yaw_rate - max_dw,
          previous_output_.yaw_rate + max_dw);
    }
  }

  VelocityCommand cmd{};
  cmd.vx = limited_vx;
  cmd.vy = limited_vy;
  cmd.yaw_rate = limited_w;
  cmd.stamp_sec = std::isfinite(now_sec) ? now_sec : 0.0;
  cmd.valid = true;

  if (cmd.vx < kEpsilon)
    cmd.vx = 0.0;
  if (std::abs(cmd.vy) < kEpsilon)
    cmd.vy = 0.0;
  if (std::abs(cmd.yaw_rate) < kEpsilon)
    cmd.yaw_rate = 0.0;

  // Preserve the original source when no safety intervention occurred.
  // Mark as safety-slow only when the command was actually reduced, and
  // safety-stop only when the final command is zero while the raw command
  // requested non-zero motion.
  const bool all_zero =
      cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.yaw_rate == 0.0;
  const bool raw_all_zero =
      raw_cmd.vx == 0.0 && raw_cmd.vy == 0.0 &&
      raw_cmd.yaw_rate == 0.0;

  if (all_zero && !raw_all_zero)
  {
    cmd.source = CommandSource::SAFETY_STOP;
  }
  else if (std::abs(cmd.vx - raw_cmd.vx) > kEpsilon ||
           std::abs(cmd.vy - raw_cmd.vy) > kEpsilon ||
           std::abs(cmd.yaw_rate - raw_cmd.yaw_rate) > kEpsilon)
  {
    cmd.source = CommandSource::SAFETY_SLOW;
  }
  else
  {
    cmd.source = raw_cmd.source;
  }

  previous_output_ = cmd;
  previous_stamp_sec_ = cmd.stamp_sec;
  has_previous_output_ = true;

  return cmd;
}

}  // namespace navdog
