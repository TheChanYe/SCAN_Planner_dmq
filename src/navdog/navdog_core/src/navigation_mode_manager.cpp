#include "navdog_core/navigation_mode_manager.hpp"

#include <cmath>

namespace navdog
{

namespace
{
constexpr double kTimeEpsilonSec = 1e-9;
constexpr double kAvoidLookaheadM = 2.0;
}

NavigationModeManager::NavigationModeManager(
    const NavigationModeConfig& config)
    : config_(config)
{
}

void NavigationModeManager::reset() noexcept
{
  status_ = NavigationModeStatus{};
  active_task_sequence_ = 0;
  last_update_stamp_sec_ = 0.0;
  has_last_update_stamp_ = false;
}

bool NavigationModeManager::isConfigValid() const noexcept
{
  return true;
}

bool NavigationModeManager::isTaskValid(
    const NavigationTask& task) const noexcept
{
  if (task.sequence == 0)
    return false;
  return task.mode == TaskMode::NORMAL_AVOID ||
         task.mode == TaskMode::ROUTE_ONLY ||
         task.mode == TaskMode::CHARGING;
}

bool NavigationModeManager::isProgressValid(
    const NavigationTask& task,
    const RouteProgress& progress) const noexcept
{
  return progress.valid &&
      progress.task_sequence == task.sequence &&
      std::isfinite(progress.arc_length_m) &&
      progress.arc_length_m >= 0.0 &&
      std::isfinite(progress.stamp_sec);
}

bool NavigationModeManager::isRobotNumericValid(
    const RobotState& robot) const noexcept
{
  return std::isfinite(robot.x) && std::isfinite(robot.y) &&
         std::isfinite(robot.yaw);
}

bool NavigationModeManager::taskAllowsAvoidance(
    TaskMode task_mode) const noexcept
{
  return task_mode == TaskMode::NORMAL_AVOID ||
         task_mode == TaskMode::CHARGING;
}

void NavigationModeManager::initializeForTask(
    const NavigationTask& task,
    double now_sec)
{
  const bool was_initialized = status_.initialized;
  status_ = NavigationModeStatus{};
  status_.mode = NavigationMode::ROUTE_FOLLOW;
  status_.previous_mode = NavigationMode::NONE;
  status_.reference_intent = ReferenceIntent::GLOBAL_ROUTE;
  status_.reason = was_initialized
      ? NavigationModeReason::TASK_CHANGED
      : NavigationModeReason::INITIALIZED;
  status_.task_sequence = task.sequence;
  status_.initialized = true;
  status_.transitioned = true;
  status_.mode_enter_stamp_sec = now_sec;
  status_.transition_stamp_sec = now_sec;
  active_task_sequence_ = task.sequence;
}

void NavigationModeManager::transitionTo(
    NavigationMode new_mode,
    NavigationModeReason reason,
    const RouteProgress& progress,
    double now_sec)
{
  (void)progress;
  status_.previous_mode = status_.mode;
  status_.mode = new_mode;
  status_.reason = reason;
  status_.transitioned = true;
  status_.mode_enter_stamp_sec = now_sec;
  status_.transition_stamp_sec = now_sec;
  status_.reference_intent = new_mode == NavigationMode::LOCAL_AVOID
      ? ReferenceIntent::LOCAL_AVOIDANCE
      : ReferenceIntent::GLOBAL_ROUTE;
  if (new_mode == NavigationMode::LOCAL_AVOID)
    ++status_.avoidance_cycle_count;
}

NavigationModeOutput NavigationModeManager::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const RouteCorridorObservationOutput& corridor,
    double now_sec)
{
  NavigationModeOutput output{};
  status_.transitioned = false;

  if (!std::isfinite(now_sec) ||
      (has_last_update_stamp_ &&
       now_sec + kTimeEpsilonSec < last_update_stamp_sec_))
  {
    output.result = NavigationModeUpdateResult::INVALID_TIME;
    output.status = status_;
    return output;
  }
  last_update_stamp_sec_ = now_sec;
  has_last_update_stamp_ = true;

  if (!isConfigValid())
  {
    output.result = NavigationModeUpdateResult::INVALID_CONFIG;
    output.status = status_;
    return output;
  }
  if (!isTaskValid(task))
  {
    output.result = NavigationModeUpdateResult::INVALID_TASK;
    output.status = status_;
    return output;
  }

  const bool initialized_this_update = !status_.initialized ||
      task.sequence != active_task_sequence_;
  if (initialized_this_update)
    initializeForTask(task, now_sec);
  const NavigationModeReason initialization_reason = status_.reason;

  status_.task_sequence = task.sequence;
  status_.avoidance_allowed = taskAllowsAvoidance(task.mode);
  if (!isProgressValid(task, progress))
  {
    output.result = NavigationModeUpdateResult::INVALID_PROGRESS;
    output.status = status_;
    return output;
  }
  if (robot.valid && !isRobotNumericValid(robot))
  {
    output.result = NavigationModeUpdateResult::INVALID_ROBOT;
    output.status = status_;
    return output;
  }

  const bool is_clear =
      corridor.result == RouteCorridorObservationResult::CLEAR;
  const bool is_blocked =
      corridor.result == RouteCorridorObservationResult::BLOCKED;
  if (!is_clear && !is_blocked)
  {
    if (corridor.result == RouteCorridorObservationResult::INVALID_TIME ||
        corridor.result == RouteCorridorObservationResult::INVALID_CONFIG ||
        corridor.result == RouteCorridorObservationResult::INVALID_PROGRESS)
    {
      output.result = NavigationModeUpdateResult::INVALID_CORRIDOR_RESULT;
    }
    else
    {
      status_.corridor_available = false;
      status_.route_blocked = false;
      status_.route_blocked_near = false;
      status_.reason = NavigationModeReason::WAITING_FOR_CORRIDOR;
      output.result = NavigationModeUpdateResult::WAITING_FOR_CORRIDOR;
    }
    status_.stamp_sec = now_sec;
    status_.valid = status_.initialized;
    output.status = status_;
    return output;
  }

  if (!corridor.assessment.valid ||
      corridor.assessment.task_sequence != task.sequence)
  {
    output.result = NavigationModeUpdateResult::INVALID_CORRIDOR_RESULT;
    output.status = status_;
    return output;
  }

  status_.corridor_available = true;
  status_.route_blocked = is_blocked;
  status_.route_blocked_near = is_blocked;

  const bool blocked_in_avoid_range = is_blocked &&
      std::isfinite(corridor.assessment.first_blocked_distance_ahead_m) &&
      corridor.assessment.first_blocked_distance_ahead_m <=
          kAvoidLookaheadM + kTimeEpsilonSec;
  status_.route_blocked_near = blocked_in_avoid_range;

  if (status_.mode == NavigationMode::ROUTE_FOLLOW &&
      blocked_in_avoid_range)
  {
    if (status_.avoidance_allowed)
      transitionTo(NavigationMode::LOCAL_AVOID,
          NavigationModeReason::BLOCK_IMMEDIATE, progress, now_sec);
    else
      status_.reason = NavigationModeReason::ROUTE_ONLY_BLOCKED;
  }
  else if (status_.mode == NavigationMode::LOCAL_AVOID && is_clear)
  {
    transitionTo(NavigationMode::ROUTE_FOLLOW,
        NavigationModeReason::ROUTE_CLEAR, progress, now_sec);
  }
  else
  {
    status_.reason = status_.mode == NavigationMode::LOCAL_AVOID
        ? NavigationModeReason::LOCAL_AVOID_ACTIVE
        : NavigationModeReason::ROUTE_CLEAR;
  }

  if (initialized_this_update && !status_.transitioned)
    status_.reason = initialization_reason;
  status_.stamp_sec = now_sec;
  status_.valid = true;
  output.result = NavigationModeUpdateResult::UPDATED;
  output.status = status_;
  return output;
}

const NavigationModeStatus&
NavigationModeManager::status() const noexcept
{
  return status_;
}

}  // namespace navdog
