#include "navdog_core/navigation_coordinator.hpp"

#include <cmath>

namespace navdog
{

// =============================================================================
// Constructor
// =============================================================================

NavigationCoordinator::NavigationCoordinator(
    const NavdogConfig& config)
    : config_(config),
      state_(NavState::IDLE),
      task_manager_(config.task)
{
}

// =============================================================================
// reset
// =============================================================================

void NavigationCoordinator::reset()
{
  state_ = NavState::IDLE;
  task_manager_.reset();
  pending_planner_actions_.clear();
  clearPlanningContext();
}

// =============================================================================
// enqueuePlannerAction
// =============================================================================

void NavigationCoordinator::enqueuePlannerAction(
    const PlannerAction& action)
{
  if (action.type == PlannerActionType::NONE)
  {
    return;
  }

  if (action.type == PlannerActionType::CANCEL)
  {
    pending_planner_actions_.clear();
  }

  pending_planner_actions_.push_back(action);
}

// =============================================================================
// takeNextPlannerAction
// =============================================================================

PlannerAction NavigationCoordinator::takeNextPlannerAction()
{
  if (pending_planner_actions_.empty())
  {
    return PlannerAction{};
  }

  const PlannerAction action =
      pending_planner_actions_.front();

  pending_planner_actions_.pop_front();

  return action;
}

// =============================================================================
// clearPlanningContext
// =============================================================================

void NavigationCoordinator::clearPlanningContext() noexcept
{
  planning_request_sent_ = false;
  planning_started_sec_ = 0.0;
  expected_trajectory_id_ = 0;
}

// =============================================================================
// startPlanningContext
// =============================================================================

void NavigationCoordinator::startPlanningContext(
    const PlannerAction& set_route_action,
    double now_sec) noexcept
{
  if (set_route_action.type != PlannerActionType::SET_ROUTE)
  {
    return;
  }

  planning_request_sent_ = true;
  planning_started_sec_ = now_sec;
  expected_trajectory_id_ =
      set_route_action.task.sequence;
}

// =============================================================================
// isPlannerFeedbackUsable
// =============================================================================

bool NavigationCoordinator::isPlannerFeedbackUsable(
    const PlannerFeedback& feedback,
    double now_sec) const noexcept
{
  if (!planning_request_sent_)
  {
    return false;
  }

  if (!feedback.valid)
  {
    return false;
  }

  if (!std::isfinite(feedback.stamp_sec) ||
      !std::isfinite(now_sec))
  {
    return false;
  }

  if (feedback.trajectory_id == 0 ||
      feedback.trajectory_id != expected_trajectory_id_)
  {
    return false;
  }

  if (feedback.stamp_sec < planning_started_sec_)
  {
    return false;
  }

  if (feedback.stamp_sec > now_sec)
  {
    return false;
  }

  return true;
}

// =============================================================================
// updatePlanningState
// =============================================================================

void NavigationCoordinator::updatePlanningState(
    const PlannerFeedback& feedback,
    double now_sec)
{
  if (state_ != NavState::PLANNING ||
      !planning_request_sent_)
  {
    return;
  }

  if (isPlannerFeedbackUsable(feedback, now_sec))
  {
    switch (feedback.state)
    {
      case PlannerState::READY:
      case PlannerState::EXECUTING:
        state_ = NavState::START_ALIGN;
        clearPlanningContext();
        return;

      case PlannerState::FAILED:
        state_ = NavState::FAILED;
        pending_planner_actions_.clear();
        clearPlanningContext();
        return;

      case PlannerState::UNAVAILABLE:
      case PlannerState::IDLE:
      case PlannerState::PLANNING:
        break;
    }
  }

  const double timeout_sec =
      config_.planner.planning_timeout_sec;

  if (!std::isfinite(timeout_sec) ||
      timeout_sec <= 0.0)
  {
    return;
  }

  if (!std::isfinite(now_sec) ||
      now_sec < planning_started_sec_)
  {
    return;
  }

  if ((now_sec - planning_started_sec_) >
      timeout_sec)
  {
    state_ = NavState::FAILED;
    pending_planner_actions_.clear();
    clearPlanningContext();
  }
}

// =============================================================================
// handleEvent
// =============================================================================

TaskHandleResult NavigationCoordinator::handleEvent(
    const NavigationEvent& event)
{
  const TaskManagerOutput task_output =
      task_manager_.handleEvent(event);

  switch (task_output.result)
  {
    case TaskHandleResult::STARTED:
      clearPlanningContext();
      state_ = NavState::PLANNING;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::CANCELLED:
      clearPlanningContext();
      state_ = NavState::IDLE;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::MAX_VX_UPDATED:
      if (state_ == NavState::PLANNING ||
          state_ == NavState::START_ALIGN ||
          state_ == NavState::TRACKING)
      {
        enqueuePlannerAction(task_output.planner_action);
      }
      break;

    case TaskHandleResult::NONE:
    case TaskHandleResult::REJECTED_BUSY:
    case TaskHandleResult::REJECTED_INVALID_TASK:
    case TaskHandleResult::CANCEL_IGNORED:
    case TaskHandleResult::MAX_VX_UNCHANGED:
    case TaskHandleResult::MAX_VX_UPDATE_IGNORED:
    case TaskHandleResult::REJECTED_INVALID_MAX_VX:
    case TaskHandleResult::UNSUPPORTED_EVENT:
      break;
  }

  return task_output.result;
}

// =============================================================================
// hasActiveTask
// =============================================================================

bool NavigationCoordinator::hasActiveTask() const noexcept
{
  return task_manager_.hasActiveTask();
}

// =============================================================================
// copyActiveTask
// =============================================================================

bool NavigationCoordinator::copyActiveTask(
    NavigationTask& task) const
{
  return task_manager_.copyActiveTask(task);
}

// =============================================================================
// update
// =============================================================================

CoreOutput NavigationCoordinator::update(
    const CoreInput& input,
    double now_sec)
{
  CoreOutput output{};

  if (state_ == NavState::PLANNING &&
      !planning_request_sent_)
  {
    output.planner_action =
        takeNextPlannerAction();

    if (output.planner_action.type ==
        PlannerActionType::SET_ROUTE)
    {
      startPlanningContext(
          output.planner_action,
          now_sec);
    }
  }
  else
  {
    if (state_ == NavState::PLANNING)
    {
      updatePlanningState(
          input.planner,
          now_sec);
    }

    output.planner_action =
        takeNextPlannerAction();
  }

  output.state = state_;
  output.task_sequence =
      task_manager_.activeSequence();

  output.final_cmd.vx = 0.0;
  output.final_cmd.vy = 0.0;
  output.final_cmd.yaw_rate = 0.0;
  output.final_cmd.valid = true;
  output.final_cmd.stamp_sec = now_sec;

  if (output.planner_action.type ==
      PlannerActionType::CANCEL)
  {
    output.final_cmd.source =
        CommandSource::CANCEL_STOP;
  }
  else
  {
    switch (state_)
    {
      case NavState::IDLE:
        output.final_cmd.source =
            CommandSource::IDLE_STOP;
        break;

      case NavState::PLANNING:
        output.final_cmd.source =
            CommandSource::PLANNING_STOP;
        break;

      case NavState::START_ALIGN:
        output.final_cmd.source =
            CommandSource::START_ALIGN;
        break;

      case NavState::FAILED:
        output.final_cmd.source =
            CommandSource::FAILED_STOP;
        break;

      default:
        output.final_cmd.source =
            CommandSource::SAFETY_STOP;
        break;
    }
  }

  return output;
}

// =============================================================================
// state
// =============================================================================

NavState NavigationCoordinator::state() const noexcept
{
  return state_;
}

// =============================================================================
// config
// =============================================================================

const NavdogConfig& NavigationCoordinator::config() const noexcept
{
  return config_;
}

}  // namespace navdog
