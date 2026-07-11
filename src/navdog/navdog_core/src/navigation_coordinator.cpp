#include "navdog_core/navigation_coordinator.hpp"

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
      state_ = NavState::PLANNING;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::CANCELLED:
      state_ = NavState::IDLE;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::MAX_VX_UPDATED:
      enqueuePlannerAction(task_output.planner_action);
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
  (void)input;

  CoreOutput output{};

  output.state = state_;
  output.task_sequence =
      task_manager_.activeSequence();

  output.planner_action =
      takeNextPlannerAction();

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
  else if (state_ == NavState::PLANNING)
  {
    output.final_cmd.source =
        CommandSource::PLANNING_STOP;
  }
  else
  {
    output.final_cmd.source =
        CommandSource::IDLE_STOP;
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
