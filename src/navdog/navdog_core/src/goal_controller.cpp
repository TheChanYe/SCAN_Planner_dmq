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
  align_started_sec_ = 0.0;
  align_timer_active_ = false;
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

  if (dist > std::max(config_.finish_dist,
                      config_.goal_align_reacquire_dist))
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate = 0.0;
    result.command.valid = true;
    result.position_lost = true;
    reset();
    return result;
  }

  if (!align_timer_active_ && std::isfinite(now_sec))
  {
    align_started_sec_ = now_sec;
    align_timer_active_ = true;
  }

  const double align_elapsed = now_sec - align_started_sec_;
  const bool align_timed_out = align_timer_active_ &&
      std::isfinite(align_elapsed) &&
      config_.goal_align_timeout_sec > 0.0 &&
      align_elapsed >= config_.goal_align_timeout_sec;

  if ((position_reached && yaw_reached) || align_timed_out)
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate = 0.0;
    result.command.valid = true;
    result.finished = true;
    result.timed_out = align_timed_out;
    return result;
  }

  (void)max_vx;
  (void)position_reached;
  const double effective_max_w =
      std::max(0.0,
          std::min(max_yaw_rate, config_.near_goal_max_w));
  result.command.vx = 0.0;
  result.command.vy = 0.0;
  result.command.yaw_rate =
      std::max(-effective_max_w,
          std::min(effective_max_w,
              config_.near_goal_kp_w * yaw_error));

  const double minimum_yaw_rate = std::min(
      effective_max_w, std::max(0.0, config_.goal_align_min_yaw_rate));
  if (!yaw_reached && minimum_yaw_rate > 0.0 &&
      std::abs(result.command.yaw_rate) < minimum_yaw_rate)
  {
    result.command.yaw_rate = std::copysign(minimum_yaw_rate, yaw_error);
  }

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
