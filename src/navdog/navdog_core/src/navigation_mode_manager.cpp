#include "navdog_core/navigation_mode_manager.hpp"

#include <cmath>
#include <cstdio>

namespace navdog
{

namespace
{
constexpr double kTimeEpsilonSec = 1e-9;
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

  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
}

bool NavigationModeManager::isConfigValid() const noexcept
{
  if (config_.enter_confirm_sec < 0.0)
    return false;
  if (config_.exit_clear_confirm_sec < 0.0)
    return false;
  if (config_.immediate_enter_distance_m <= 0.0)
    return false;
  if (config_.immediate_enter_distance_m >
      config_.enter_blocked_distance_m)
    return false;
  if (config_.exit_front_clearance_m < 0.0)
    return false;
  if (config_.exit_left_clearance_m < 0.0)
    return false;
  if (config_.exit_right_clearance_m < 0.0)
    return false;
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

  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
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

  // Clear candidate timers on any transition.
  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
}

NavigationModeOutput NavigationModeManager::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const RouteCorridorObservationOutput& corridor,
    const ObstacleSummary& obstacles,
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
          config_.enter_blocked_distance_m + kTimeEpsilonSec;
  status_.route_blocked_near = blocked_in_avoid_range;

  status_.blocked_confirm_elapsed_sec =
      blocked_candidate_active_ ? (now_sec - blocked_candidate_start_sec_) : 0.0;
  status_.clear_confirm_elapsed_sec =
      clear_candidate_active_ ? (now_sec - clear_candidate_start_sec_) : 0.0;

  // --- ROUTE_FOLLOW → LOCAL_AVOID ---
  if (status_.mode == NavigationMode::ROUTE_FOLLOW &&
      blocked_in_avoid_range)
  {
    if (!status_.avoidance_allowed)
    {
      status_.reason = NavigationModeReason::ROUTE_ONLY_BLOCKED;
    }
    else
    {
      const double blocked_dist =
          corridor.assessment.first_blocked_distance_ahead_m;

      // Immediate enter for very close obstacles.
      if (blocked_dist <= config_.immediate_enter_distance_m)
      {
        blocked_candidate_active_ = false;
        blocked_candidate_start_sec_ = 0.0;

        std::printf("NAV_MODE ROUTE_FOLLOW -> LOCAL_AVOID "
                    "blocked_forward=%.3f confirm_hold=0.000 immediate=true\n",
                    blocked_dist);
        std::fflush(stdout);

        transitionTo(NavigationMode::LOCAL_AVOID,
            NavigationModeReason::BLOCK_IMMEDIATE, progress, now_sec);
      }
      else
      {
        // Start or continue confirmation timer.
        if (!blocked_candidate_active_)
        {
          blocked_candidate_active_ = true;
          blocked_candidate_start_sec_ = now_sec;
        }

        const double held = now_sec - blocked_candidate_start_sec_;
        if (held >= config_.enter_confirm_sec)
        {
          blocked_candidate_active_ = false;
          blocked_candidate_start_sec_ = 0.0;

          std::printf("NAV_MODE ROUTE_FOLLOW -> LOCAL_AVOID "
                      "blocked_forward=%.3f confirm_hold=%.3f immediate=false\n",
                      blocked_dist, held);
          std::fflush(stdout);

          transitionTo(NavigationMode::LOCAL_AVOID,
              NavigationModeReason::BLOCK_IMMEDIATE, progress, now_sec);
        }
      }
    }
  }
  // --- LOCAL_AVOID → ROUTE_FOLLOW ---
  else if (status_.mode == NavigationMode::LOCAL_AVOID)
  {
    const double mode_hold_sec =
        now_sec - status_.mode_enter_stamp_sec;
    const bool minimum_hold_satisfied =
        mode_hold_sec >= config_.min_local_avoid_hold_sec;

    // Check directional clearance from obstacles.
    const double front_min = obstacles.valid
        ? obstacles.front_min
        : std::numeric_limits<double>::infinity();
    const double left_min = obstacles.valid
        ? obstacles.left_min
        : std::numeric_limits<double>::infinity();
    const double right_min = obstacles.valid
        ? obstacles.right_min
        : std::numeric_limits<double>::infinity();

    const bool clearance_satisfied =
        obstacles.valid &&
        front_min >= config_.exit_front_clearance_m &&
        left_min >= config_.exit_left_clearance_m &&
        right_min >= config_.exit_right_clearance_m;

    // Exit LOCAL_AVOID when:
    //   1. Minimum hold time satisfied
    //   2. Directional obstacle clearance satisfied (from ObstacleSummary)
    // Note: corridor CLEAR is NOT required here because during active
    // avoidance the corridor at the robot's current position may still
    // report BLOCKED even though the robot has found a safe path.
    const bool all_exit_conditions =
        minimum_hold_satisfied &&
        clearance_satisfied;

    if (all_exit_conditions)
    {
      if (!clear_candidate_active_)
      {
        clear_candidate_active_ = true;
        clear_candidate_start_sec_ = now_sec;
      }

      const double clear_held = now_sec - clear_candidate_start_sec_;
      if (clear_held >= config_.exit_clear_confirm_sec)
      {
        std::printf("NAV_MODE LOCAL_AVOID -> ROUTE_FOLLOW "
                    "front_min=%.3f left_min=%.3f right_min=%.3f "
                    "corridor_clear_hold=%.3f mode_hold=%.3f\n",
                    front_min, left_min, right_min,
                    clear_held, mode_hold_sec);
        std::fflush(stdout);

        clear_candidate_active_ = false;
        clear_candidate_start_sec_ = 0.0;

        transitionTo(NavigationMode::ROUTE_FOLLOW,
            NavigationModeReason::ROUTE_CLEAR, progress, now_sec);
      }
    }
    else
    {
      // Any exit condition not met — reset clear candidate timer.
      clear_candidate_active_ = false;
      clear_candidate_start_sec_ = 0.0;
    }

    if (!status_.transitioned)
      status_.reason = NavigationModeReason::LOCAL_AVOID_ACTIVE;
  }
  else
  {
    // ROUTE_FOLLOW and not blocked.
    blocked_candidate_active_ = false;
    blocked_candidate_start_sec_ = 0.0;

    status_.reason = NavigationModeReason::ROUTE_CLEAR;
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
