#include "navdog_core/start_align_controller.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

// =============================================================================
// Constructor
// =============================================================================

StartAlignController::StartAlignController(
    const StartAlignConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

void StartAlignController::reset() noexcept
{
  phase_started_ = false;
  target_resolved_ = false;
  rotation_required_ = false;
  phase_started_sec_ = 0.0;
  target_yaw_ = 0.0;
}

// =============================================================================
// active
// =============================================================================

bool StartAlignController::active() const noexcept
{
  return phase_started_;
}

// =============================================================================
// normalizeAngle
// =============================================================================

double StartAlignController::normalizeAngle(
    double angle) noexcept
{
  return std::atan2(
      std::sin(angle),
      std::cos(angle));
}

// =============================================================================
// degreesToRadians
// =============================================================================

double StartAlignController::degreesToRadians(
    double degrees) noexcept
{
  return degrees * 3.14159265358979323846 / 180.0;
}

// =============================================================================
// isConfigValid
// =============================================================================

bool StartAlignController::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.enter_deg) ||
      !std::isfinite(config_.exit_deg) ||
      !std::isfinite(config_.max_hold_sec) ||
      !std::isfinite(config_.kp_yaw) ||
      !std::isfinite(config_.max_yaw_rate) ||
      !std::isfinite(config_.target_min_dist_m))
  {
    return false;
  }

  if (config_.enter_deg < 0.0 ||
      config_.exit_deg < 0.0)
  {
    return false;
  }

  if (config_.max_hold_sec <= 0.0 ||
      config_.kp_yaw <= 0.0 ||
      config_.max_yaw_rate <= 0.0 ||
      config_.target_min_dist_m <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isRobotUsable
// =============================================================================

bool StartAlignController::isRobotUsable(
    const RobotState& robot) const noexcept
{
  if (!robot.valid)
  {
    return false;
  }

  if (!std::isfinite(robot.x) ||
      !std::isfinite(robot.y) ||
      !std::isfinite(robot.yaw))
  {
    return false;
  }

  return true;
}

// =============================================================================
// resolveTargetYaw
// =============================================================================

bool StartAlignController::resolveTargetYaw(
    const NavigationTask& task,
    const RobotState& robot,
    double& target_yaw) const noexcept
{
  const double min_dist =
      std::fabs(config_.target_min_dist_m);

  // --- Priority 1: first valid route segment ---
  if (task.points.size() >= 2)
  {
    for (std::size_t i = 0;
         i + 1 < task.points.size();
         ++i)
    {
      const RoutePoint& current = task.points[i];
      const RoutePoint& next = task.points[i + 1];

      const double dx = next.x - current.x;
      const double dy = next.y - current.y;
      const double distance = std::hypot(dx, dy);

      if (distance >= min_dist)
      {
        target_yaw = std::atan2(dy, dx);
        return true;
      }
    }
  }

  // --- Priority 2: explicit yaw on a route point ---
  for (const auto& point : task.points)
  {
    if (point.has_yaw &&
        std::isfinite(point.yaw))
    {
      target_yaw = normalizeAngle(point.yaw);
      return true;
    }
  }

  // --- Priority 3: robot pointing toward a target point ---
  if (isRobotUsable(robot))
  {
    for (const auto& point : task.points)
    {
      const double dx = point.x - robot.x;
      const double dy = point.y - robot.y;
      const double distance = std::hypot(dx, dy);

      if (distance >= min_dist)
      {
        target_yaw = std::atan2(dy, dx);
        return true;
      }
    }
  }

  return false;
}

// =============================================================================
// update
// =============================================================================

StartAlignOutput StartAlignController::update(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec)
{
  // --- Safe default output ---
  StartAlignOutput output{};

  output.command.vx = 0.0;
  output.command.vy = 0.0;
  output.command.yaw_rate = 0.0;
  output.command.valid = true;
  output.command.source = CommandSource::START_ALIGN;
  output.command.stamp_sec =
      std::isfinite(now_sec) ? now_sec : 0.0;

  // --- 12.1 Check time ---
  if (!std::isfinite(now_sec))
  {
    output.result = StartAlignResult::INVALID_TIME;
    return output;
  }

  // --- 12.2 Check config ---
  if (!isConfigValid())
  {
    output.result = StartAlignResult::INVALID_CONFIG;
    return output;
  }

  // --- 12.3 Start alignment phase ---
  if (!phase_started_)
  {
    phase_started_ = true;
    phase_started_sec_ = now_sec;
  }

  // --- 12.4 Resolve target direction ---
  if (!target_resolved_)
  {
    double resolved_yaw = 0.0;

    if (resolveTargetYaw(task, robot, resolved_yaw))
    {
      target_yaw_ = normalizeAngle(resolved_yaw);
      target_resolved_ = true;
    }
    else
    {
      // Cannot resolve direction
      if (!isRobotUsable(robot))
      {
        // Robot not available yet; wait but check timeout
        output.result = StartAlignResult::WAITING_FOR_ROBOT;

        const double elapsed =
            now_sec - phase_started_sec_;
        const double max_hold =
            std::fabs(config_.max_hold_sec);

        if (elapsed > max_hold)
        {
          output.result = StartAlignResult::TIMED_OUT;
        }

        return output;
      }

      // Robot valid but direction cannot be resolved
      output.result = StartAlignResult::INVALID_TASK;
      return output;
    }
  }

  // --- 12.5 Wait for robot state ---
  if (!isRobotUsable(robot))
  {
    output.result = StartAlignResult::WAITING_FOR_ROBOT;

    // Check timeout even while waiting
    const double elapsed =
        now_sec - phase_started_sec_;
    const double max_hold =
        std::fabs(config_.max_hold_sec);

    if (elapsed > max_hold)
    {
      output.result = StartAlignResult::TIMED_OUT;
    }

    return output;
  }

  // --- 12.6 Compute angle error ---
  const double yaw_error =
      normalizeAngle(target_yaw_ - robot.yaw);

  output.target_yaw = target_yaw_;
  output.yaw_error = yaw_error;
  output.has_target = true;

  // --- Hysteresis thresholds ---
  const double enter_rad =
      degreesToRadians(
          std::max(
              std::fabs(config_.enter_deg),
              std::fabs(config_.exit_deg)));

  const double exit_rad =
      degreesToRadians(
          std::min(
              std::fabs(config_.enter_deg),
              std::fabs(config_.exit_deg)));

  // --- Not yet rotating ---
  if (!rotation_required_)
  {
    if (std::fabs(yaw_error) <= enter_rad)
    {
      output.result = StartAlignResult::ALIGNED;
      return output;
    }

    rotation_required_ = true;
  }

  // --- Already rotating: check exit threshold ---
  if (std::fabs(yaw_error) <= exit_rad)
  {
    output.result = StartAlignResult::ALIGNED;
    return output;
  }

  // --- Alignment in progress: compute yaw_rate ---
  double yaw_rate =
      config_.kp_yaw * yaw_error;

  const double max_yaw_rate =
      std::fabs(config_.max_yaw_rate);

  yaw_rate =
      std::max(
          -max_yaw_rate,
          std::min(max_yaw_rate, yaw_rate));

  output.command.yaw_rate = yaw_rate;
  output.result = StartAlignResult::ALIGNING;

  // --- Check timeout (ALIGNED takes priority) ---
  const double elapsed =
      now_sec - phase_started_sec_;
  const double max_hold =
      std::fabs(config_.max_hold_sec);

  if (elapsed > max_hold)
  {
    output.result = StartAlignResult::TIMED_OUT;
    output.command.yaw_rate = 0.0;
  }

  return output;
}

}  // namespace navdog
