#include "navdog_core/navigation_mode_manager.hpp"

#include <cmath>

namespace navdog
{

namespace
{

constexpr double kTimeEpsilonSec = 1e-9;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

NavigationModeManager::NavigationModeManager(
    const NavigationModeConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

void NavigationModeManager::reset() noexcept
{
  status_ = NavigationModeStatus{};
  active_task_sequence_ = 0;
  has_last_update_stamp_ = false;
  last_update_stamp_sec_ = 0.0;
  resetBlockedEvidence();
  resetClearEvidence();
  resetRejoinEvidence();
}

// =============================================================================
// isConfigValid
// =============================================================================

bool NavigationModeManager::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.avoid_enter_distance_m) ||
      config_.avoid_enter_distance_m <= 0.0)
    return false;

  if (!std::isfinite(config_.avoid_immediate_distance_m) ||
      config_.avoid_immediate_distance_m <= 0.0 ||
      config_.avoid_immediate_distance_m >
          config_.avoid_enter_distance_m)
    return false;

  if (!std::isfinite(config_.avoid_block_confirm_sec) ||
      config_.avoid_block_confirm_sec < 0.0)
    return false;

  if (!std::isfinite(config_.local_avoid_min_hold_sec) ||
      config_.local_avoid_min_hold_sec < 0.0)
    return false;

  if (!std::isfinite(config_.route_clear_confirm_sec) ||
      config_.route_clear_confirm_sec < 0.0)
    return false;

  if (!std::isfinite(config_.rejoin_lateral_tolerance_m) ||
      config_.rejoin_lateral_tolerance_m < 0.0)
    return false;

  if (!std::isfinite(config_.rejoin_heading_tolerance_rad) ||
      config_.rejoin_heading_tolerance_rad < 0.0 ||
      config_.rejoin_heading_tolerance_rad > kPi)
    return false;

  if (!std::isfinite(config_.rejoin_confirm_sec) ||
      config_.rejoin_confirm_sec < 0.0)
    return false;

  return true;
}

// =============================================================================
// isTaskValid
// =============================================================================

bool NavigationModeManager::isTaskValid(
    const NavigationTask& task) const noexcept
{
  if (task.sequence == 0)
    return false;

  switch (task.mode)
  {
    case TaskMode::NORMAL_AVOID:
    case TaskMode::ROUTE_ONLY:
    case TaskMode::CHARGING:
      break;
    default:
      return false;
  }

  return true;
}

// =============================================================================
// isProgressValid
// =============================================================================

bool NavigationModeManager::isProgressValid(
    const NavigationTask& task,
    const RouteProgress& progress) const noexcept
{
  if (!progress.valid)
    return false;

  if (progress.task_sequence != task.sequence)
    return false;

  if (!std::isfinite(progress.arc_length_m) ||
      progress.arc_length_m < 0.0)
    return false;

  if (!std::isfinite(progress.lateral_error_m))
    return false;

  if (!std::isfinite(progress.route_yaw))
    return false;

  if (!std::isfinite(progress.stamp_sec))
    return false;

  return true;
}

// =============================================================================
// isRobotNumericValid
// =============================================================================

bool NavigationModeManager::isRobotNumericValid(
    const RobotState& robot) const noexcept
{
  if (!std::isfinite(robot.x) ||
      !std::isfinite(robot.y) ||
      !std::isfinite(robot.yaw))
    return false;

  return true;
}

// =============================================================================
// taskAllowsAvoidance
// =============================================================================

bool NavigationModeManager::taskAllowsAvoidance(
    TaskMode task_mode) const noexcept
{
  switch (task_mode)
  {
    case TaskMode::NORMAL_AVOID:
    case TaskMode::CHARGING:
      return true;
    case TaskMode::ROUTE_ONLY:
      return false;
    default:
      return false;
  }
}

// =============================================================================
// initializeForTask
// =============================================================================

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
  status_.avoidance_cycle_count = 0;

  active_task_sequence_ = task.sequence;

  resetBlockedEvidence();
  resetClearEvidence();
  resetRejoinEvidence();
}

// =============================================================================
// transitionTo
// =============================================================================

void NavigationModeManager::transitionTo(
    NavigationMode new_mode,
    NavigationModeReason reason,
    const RouteProgress& progress,
    double now_sec)
{
  status_.previous_mode = status_.mode;
  status_.mode = new_mode;
  status_.transitioned = true;
  status_.reason = reason;
  status_.mode_enter_stamp_sec = now_sec;
  status_.transition_stamp_sec = now_sec;

  switch (new_mode)
  {
    case NavigationMode::ROUTE_FOLLOW:
      status_.reference_intent =
          ReferenceIntent::GLOBAL_ROUTE;
      resetBlockedEvidence();
      resetClearEvidence();
      resetRejoinEvidence();
      status_.has_rejoin_anchor = false;
      status_.rejoin_min_arc_length_m = 0.0;
      break;

    case NavigationMode::LOCAL_AVOID:
      status_.reference_intent =
          ReferenceIntent::LOCAL_AVOIDANCE;
      resetBlockedEvidence();
      resetClearEvidence();
      resetRejoinEvidence();
      status_.avoidance_cycle_count += 1;
      status_.has_rejoin_anchor = false;
      status_.rejoin_min_arc_length_m = 0.0;
      break;

    case NavigationMode::ROUTE_REJOIN:
      status_.reference_intent =
          ReferenceIntent::FORWARD_ROUTE_REJOIN;
      resetBlockedEvidence();
      resetClearEvidence();
      resetRejoinEvidence();
      status_.rejoin_min_arc_length_m =
          progress.arc_length_m;
      status_.has_rejoin_anchor = true;
      break;

    default:
      break;
  }
}

// =============================================================================
// resetBlockedEvidence
// =============================================================================

void NavigationModeManager::resetBlockedEvidence() noexcept
{
  blocked_timer_active_ = false;
  blocked_since_sec_ = 0.0;
  status_.blocked_confirm_elapsed_sec = 0.0;
}

// =============================================================================
// resetClearEvidence
// =============================================================================

void NavigationModeManager::resetClearEvidence() noexcept
{
  clear_timer_active_ = false;
  clear_since_sec_ = 0.0;
  status_.clear_confirm_elapsed_sec = 0.0;
}

// =============================================================================
// resetRejoinEvidence
// =============================================================================

void NavigationModeManager::resetRejoinEvidence() noexcept
{
  rejoin_timer_active_ = false;
  rejoin_since_sec_ = 0.0;
  status_.rejoin_confirm_elapsed_sec = 0.0;
}

// =============================================================================
// shortestAngularDistance
// =============================================================================

double NavigationModeManager::shortestAngularDistance(
    double from,
    double to) const noexcept
{
  double error = to - from;

  while (error > kPi)
    error -= 2.0 * kPi;

  while (error < -kPi)
    error += 2.0 * kPi;

  return error;
}

// =============================================================================
// update
// =============================================================================

NavigationModeOutput NavigationModeManager::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const RouteCorridorObservationOutput& corridor,
    double now_sec)
{
  NavigationModeOutput output{};

  // Reset transitioned flag at start of each update.
  status_.transitioned = false;

  // --- 1. Time validation ---

  if (!std::isfinite(now_sec))
  {
    output.result =
        NavigationModeUpdateResult::INVALID_TIME;
    output.status = status_;
    return output;
  }

  if (has_last_update_stamp_ &&
      now_sec + kTimeEpsilonSec <
          last_update_stamp_sec_)
  {
    output.result =
        NavigationModeUpdateResult::INVALID_TIME;
    output.status = status_;
    return output;
  }

  last_update_stamp_sec_ = now_sec;
  has_last_update_stamp_ = true;

  // --- 2. Config validation ---

  if (!isConfigValid())
  {
    output.result =
        NavigationModeUpdateResult::INVALID_CONFIG;
    output.status = status_;
    return output;
  }

  // --- 3. Task validation ---

  if (!isTaskValid(task))
  {
    output.result =
        NavigationModeUpdateResult::INVALID_TASK;
    output.status = status_;
    return output;
  }

  // --- 4. Task initialization / sequence change ---

  const bool initialized_this_update =
      !status_.initialized ||
      task.sequence != active_task_sequence_;

  if (initialized_this_update)
  {
    initializeForTask(task, now_sec);
    // Continue processing the current Corridor so an immediate
    // obstacle can still enter LOCAL_AVOID in the same update.
  }

  const NavigationMode mode_after_initialization = status_.mode;
  const NavigationModeReason initialization_reason = status_.reason;

  // Update task-related fields.
  status_.task_sequence = task.sequence;
  status_.avoidance_allowed =
      taskAllowsAvoidance(task.mode);

  // --- 5. Progress validation ---

  if (!isProgressValid(task, progress))
  {
    output.result =
        NavigationModeUpdateResult::INVALID_PROGRESS;
    output.status = status_;
    return output;
  }

  // --- 6. Robot numeric validation ---

  if (robot.valid && !isRobotNumericValid(robot))
  {
    output.result =
        NavigationModeUpdateResult::INVALID_ROBOT;
    output.status = status_;
    return output;
  }

  // --- 7. Corridor classification ---

  const bool is_clear =
      corridor.result ==
      RouteCorridorObservationResult::CLEAR;
  const bool is_blocked =
      corridor.result ==
      RouteCorridorObservationResult::BLOCKED;

  if (!is_clear && !is_blocked)
  {
    // Check for internal errors.
    if (corridor.result ==
            RouteCorridorObservationResult::INVALID_TIME ||
        corridor.result ==
            RouteCorridorObservationResult::INVALID_CONFIG ||
        corridor.result ==
            RouteCorridorObservationResult::INVALID_PROGRESS)
    {
      output.result =
          NavigationModeUpdateResult::
              INVALID_CORRIDOR_RESULT;
      output.status = status_;
      return output;
    }

    // Transient corridor unavailable.
    status_.corridor_available = false;
    status_.route_blocked = false;
    status_.route_blocked_near = false;
    resetBlockedEvidence();
    resetClearEvidence();
    resetRejoinEvidence();
    status_.reason =
        NavigationModeReason::WAITING_FOR_CORRIDOR;
    status_.stamp_sec = now_sec;
    status_.valid = status_.initialized;
    output.result =
        NavigationModeUpdateResult::WAITING_FOR_CORRIDOR;
    output.status = status_;
    return output;
  }

  // Corridor is CLEAR or BLOCKED — validate consistency.
  if (!corridor.assessment.valid ||
      corridor.assessment.task_sequence !=
          task.sequence)
  {
    output.result =
        NavigationModeUpdateResult::
            INVALID_CORRIDOR_RESULT;
    output.status = status_;
    return output;
  }

  // --- 8. Set corridor-derived fields ---

  status_.corridor_available = true;
  status_.route_blocked = is_blocked;

  const double blocked_dist =
      corridor.assessment.first_blocked_distance_ahead_m;
  const bool near_blocked = is_blocked &&
      std::isfinite(blocked_dist) &&
      blocked_dist <=
          config_.avoid_enter_distance_m;
  status_.route_blocked_near = near_blocked;

  // --- 9. Mode-specific processing ---

  NavigationModeUpdateResult update_result =
      NavigationModeUpdateResult::UPDATED;

  switch (status_.mode)
  {
    // =========================================================================
    // ROUTE_FOLLOW
    // =========================================================================

    case NavigationMode::ROUTE_FOLLOW:
    {
      if (is_clear)
      {
        status_.reason =
            NavigationModeReason::ROUTE_CLEAR;
        status_.route_blocked = false;
        status_.route_blocked_near = false;
        resetBlockedEvidence();
      }
      else  // is_blocked
      {
        if (!near_blocked)
        {
          status_.reason =
              NavigationModeReason::BLOCKED_FAR_AHEAD;
          status_.route_blocked = true;
          status_.route_blocked_near = false;
          resetBlockedEvidence();
        }
        else
        {
          // Near blocked.
          if (!status_.avoidance_allowed)
          {
            // ROUTE_ONLY: never enter avoid.
            status_.reason =
                NavigationModeReason::
                    ROUTE_ONLY_BLOCKED;
          }
          else if (std::isfinite(blocked_dist) &&
                   blocked_dist <=
                       config_.avoid_immediate_distance_m)
          {
            // Immediate block.
            transitionTo(
                NavigationMode::LOCAL_AVOID,
                NavigationModeReason::BLOCK_IMMEDIATE,
                progress,
                now_sec);
          }
          else
          {
            // Normal confirmation.
            if (!blocked_timer_active_)
            {
              blocked_since_sec_ = now_sec;
              blocked_timer_active_ = true;
            }

            const double elapsed =
                now_sec - blocked_since_sec_;
            status_.blocked_confirm_elapsed_sec =
                elapsed;

            if (elapsed + kTimeEpsilonSec >=
                config_.avoid_block_confirm_sec)
            {
              transitionTo(
                  NavigationMode::LOCAL_AVOID,
                  NavigationModeReason::
                      BLOCK_CONFIRMED,
                  progress,
                  now_sec);
            }
            else
            {
              status_.reason =
                  NavigationModeReason::
                      BLOCK_CONFIRMING;
            }
          }
        }
      }
      break;
    }

    // =========================================================================
    // LOCAL_AVOID
    // =========================================================================

    case NavigationMode::LOCAL_AVOID:
    {
      if (is_blocked)
      {
        status_.reason =
            NavigationModeReason::LOCAL_AVOID_ACTIVE;
        resetClearEvidence();
        resetRejoinEvidence();
      }
      else  // is_clear
      {
        if (!clear_timer_active_)
        {
          clear_since_sec_ = now_sec;
          clear_timer_active_ = true;
          status_.clear_confirm_elapsed_sec = 0.0;
        }

        const double mode_elapsed =
            now_sec - status_.mode_enter_stamp_sec;
        const double clear_elapsed =
            now_sec - clear_since_sec_;
        status_.clear_confirm_elapsed_sec =
            clear_elapsed;

        if (mode_elapsed + kTimeEpsilonSec >=
                config_.local_avoid_min_hold_sec &&
            clear_elapsed + kTimeEpsilonSec >=
                config_.route_clear_confirm_sec)
        {
          transitionTo(
              NavigationMode::ROUTE_REJOIN,
              NavigationModeReason::CLEAR_CONFIRMED,
              progress,
              now_sec);
        }
        else
        {
          status_.reason =
              NavigationModeReason::
                  CLEAR_CONFIRMING;
        }
      }
      break;
    }

    // =========================================================================
    // ROUTE_REJOIN
    // =========================================================================

    case NavigationMode::ROUTE_REJOIN:
    {
      if (is_blocked)
      {
        if (!near_blocked)
        {
          // Far blocked.
          status_.reason =
              NavigationModeReason::
                  BLOCKED_FAR_AHEAD;
          resetRejoinEvidence();
          resetBlockedEvidence();
        }
        else if (std::isfinite(blocked_dist) &&
                 blocked_dist <=
                     config_.avoid_immediate_distance_m)
        {
          // Immediate block.
          transitionTo(
              NavigationMode::LOCAL_AVOID,
              NavigationModeReason::REJOIN_BLOCKED,
              progress,
              now_sec);
        }
        else
        {
          // Confirm block.
          if (!blocked_timer_active_)
          {
            blocked_since_sec_ = now_sec;
            blocked_timer_active_ = true;
            status_.blocked_confirm_elapsed_sec = 0.0;
          }

          const double elapsed =
              now_sec - blocked_since_sec_;
          status_.blocked_confirm_elapsed_sec =
              elapsed;

          if (elapsed + kTimeEpsilonSec >=
              config_.avoid_block_confirm_sec)
          {
            transitionTo(
                NavigationMode::LOCAL_AVOID,
                NavigationModeReason::REJOIN_BLOCKED,
                progress,
                now_sec);
          }
          else
          {
            status_.reason =
                NavigationModeReason::
                    BLOCK_CONFIRMING;
            resetRejoinEvidence();
          }
        }
      }
      else  // is_clear
      {
        // CLEAR breaks continuity of any previous blocked evidence.
        resetBlockedEvidence();

        // Check robot validity.
        if (!robot.valid)
        {
          status_.reason =
              NavigationModeReason::
                  WAITING_FOR_ROBOT;
          resetRejoinEvidence();
          update_result =
              NavigationModeUpdateResult::
                  WAITING_FOR_ROBOT;
        }
        else
        {
          // Check rejoin conditions.
          const bool lateral_ok =
              std::fabs(progress.lateral_error_m) <=
              config_.rejoin_lateral_tolerance_m;

          const double heading_error =
              shortestAngularDistance(
                  robot.yaw,
                  progress.route_yaw);

          const bool heading_ok =
              std::fabs(heading_error) <=
              config_.rejoin_heading_tolerance_rad;

          if (!lateral_ok || !heading_ok)
          {
            status_.reason =
                NavigationModeReason::
                    ROUTE_REJOIN_ACTIVE;
            resetRejoinEvidence();
          }
          else
          {
            if (!rejoin_timer_active_)
            {
              rejoin_since_sec_ = now_sec;
              rejoin_timer_active_ = true;
              status_.rejoin_confirm_elapsed_sec = 0.0;
            }

            const double elapsed =
                now_sec - rejoin_since_sec_;
            status_.rejoin_confirm_elapsed_sec =
                elapsed;

            if (elapsed + kTimeEpsilonSec >=
                config_.rejoin_confirm_sec)
            {
              transitionTo(
                  NavigationMode::ROUTE_FOLLOW,
                  NavigationModeReason::
                      REJOIN_COMPLETE,
                  progress,
                  now_sec);
            }
            else
            {
              status_.reason =
                  NavigationModeReason::
                      REJOIN_CONFIRMING;
            }
          }
        }
      }
      break;
    }

    default:
      break;
  }

  // --- 10. Finalize ---

  if (initialized_this_update &&
      status_.mode == mode_after_initialization)
  {
    status_.reason = initialization_reason;
  }

  status_.stamp_sec = now_sec;
  status_.valid = status_.initialized;
  output.result = update_result;
  output.status = status_;

  return output;
}

// =============================================================================
// status
// =============================================================================

const NavigationModeStatus&
NavigationModeManager::status() const noexcept
{
  return status_;
}

}  // namespace navdog
