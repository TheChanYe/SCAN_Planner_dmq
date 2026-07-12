#include "navdog_core/goal_controller.hpp"

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

GoalController::GoalController(
    const GoalControllerConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

void GoalController::reset() noexcept
{
}

// =============================================================================
// finalGoal
// =============================================================================

RoutePoint GoalController::finalGoal() const noexcept
{
  RoutePoint goal{};
  return goal;
}

// =============================================================================
// isNearGoal
// =============================================================================

bool GoalController::isNearGoal(
    const RouteProgress& progress) const noexcept
{
  if (!progress.valid ||
      !std::isfinite(progress.remaining_distance_m))
  {
    return false;
  }

  return progress.remaining_distance_m <=
         config_.near_goal_switch_dist;
}

// =============================================================================
// update
// =============================================================================

GoalController::Result GoalController::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double max_yaw_rate,
    double now_sec)
{
  Result result{};
  result.command.stamp_sec = now_sec;
  result.command.source = CommandSource::GOAL_ALIGN;

  if (task.points.empty() ||
      !progress.valid ||
      !robot.valid)
  {
    result.command.valid = false;
    return result;
  }

  const RoutePoint goal = task.points.back();

  if (!std::isfinite(goal.x) ||
      !std::isfinite(goal.y) ||
      !std::isfinite(robot.x) ||
      !std::isfinite(robot.y))
  {
    result.command.valid = false;
    return result;
  }

  const double dx = goal.x - robot.x;
  const double dy = goal.y - robot.y;
  const double dist = std::hypot(dx, dy);

  const double goal_yaw = goal.has_yaw ? goal.yaw : progress.route_yaw;
  const double yaw_error =
      std::isfinite(goal_yaw)
          ? normalizeAngle(goal_yaw - robot.yaw)
          : 0.0;

  const bool position_reached = dist <= config_.finish_dist;
  const bool yaw_reached =
      std::abs(yaw_error) <=
      config_.finish_yaw_tolerance_rad;

  if (position_reached && yaw_reached)
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate = 0.0;
    result.command.valid = true;
    result.finished = true;
    return result;
  }

  if (position_reached)
  {
    // In-place yaw alignment.
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate =
        std::max(-max_yaw_rate,
            std::min(max_yaw_rate,
                config_.near_goal_kp_w * yaw_error));
    if (std::abs(result.command.yaw_rate) < kEpsilon)
      result.command.yaw_rate = 0.0;
    result.command.valid = true;
    return result;
  }

  // Position not reached: drive slowly toward goal.
  const double desired_v =
      std::max(config_.near_goal_min_v,
          std::min(config_.near_goal_max_v,
              config_.near_goal_kp_v * dist));

  const double effective_max_vx =
      std::min(max_vx, desired_v);

  const double heading_to_goal = std::atan2(dy, dx);
  const double heading_error_to_goal =
      normalizeAngle(heading_to_goal - robot.yaw);

  if (std::abs(heading_error_to_goal) >
      config_.near_goal_turn_only_rad)
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate =
        std::max(-config_.near_goal_max_w,
            std::min(config_.near_goal_max_w,
                config_.near_goal_kp_w *
                    heading_error_to_goal));
    result.command.valid = true;
    return result;
  }

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  double vx_world = effective_max_vx * std::cos(heading_to_goal);
  double vy_world = effective_max_vx * std::sin(heading_to_goal);

  result.command.vx = c * vx_world + s * vy_world;
  result.command.vy = -s * vx_world + c * vy_world;
  result.command.yaw_rate =
      std::max(-config_.near_goal_max_w,
          std::min(config_.near_goal_max_w,
              config_.near_goal_kp_w * yaw_error));

  if (!std::isfinite(result.command.vx))
    result.command.vx = 0.0;
  if (!std::isfinite(result.command.vy))
    result.command.vy = 0.0;
  if (!std::isfinite(result.command.yaw_rate))
    result.command.yaw_rate = 0.0;

  result.command.valid = true;
  return result;
}

}  // namespace navdog
