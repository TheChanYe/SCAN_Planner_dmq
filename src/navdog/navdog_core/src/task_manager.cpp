#include "navdog_core/task_manager.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

// =============================================================================
// Constructor
// =============================================================================

TaskManager::TaskManager(const TaskConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

void TaskManager::reset()
{
  has_active_task_ = false;
  active_task_ = NavigationTask{};
  next_sequence_ = 1;
}

// =============================================================================
// clearActiveTask
// =============================================================================

void TaskManager::clearActiveTask()
{
  has_active_task_ = false;
  active_task_ = NavigationTask{};
}

// =============================================================================
// isTaskModeValid
// =============================================================================

bool TaskManager::isTaskModeValid(TaskMode mode) const noexcept
{
  switch (mode)
  {
    case TaskMode::NORMAL_AVOID:
    case TaskMode::ROUTE_ONLY:
    case TaskMode::CHARGING:
      return true;

    default:
      return false;
  }
}

// =============================================================================
// isTaskValid
// =============================================================================

bool TaskManager::isTaskValid(const NavigationTask& task) const noexcept
{
  if (!isTaskModeValid(task.mode))
  {
    return false;
  }

  if (task.points.empty())
  {
    return false;
  }

  if (!std::isfinite(task.max_vx) || task.max_vx <= 0.0)
  {
    return false;
  }

  for (const auto& pt : task.points)
  {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
    {
      return false;
    }

    if (pt.has_yaw && !std::isfinite(pt.yaw))
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// clampMaxVx
// =============================================================================

double TaskManager::clampMaxVx(double max_vx) const noexcept
{
  const double lower =
      std::min(config_.min_max_vx, config_.max_max_vx);

  const double upper =
      std::max(config_.min_max_vx, config_.max_max_vx);

  if (max_vx < lower)
  {
    return lower;
  }
  if (max_vx > upper)
  {
    return upper;
  }
  return max_vx;
}

// =============================================================================
// handleEvent
// =============================================================================

TaskManagerOutput TaskManager::handleEvent(const NavigationEvent& event)
{
  TaskManagerOutput output{};

  // -------------------------------------------------------------------------
  // NONE
  // -------------------------------------------------------------------------
  if (event.type == NavigationEventType::NONE)
  {
    output.result = TaskHandleResult::NONE;
    output.planner_action.type = PlannerActionType::NONE;
    return output;
  }

  // -------------------------------------------------------------------------
  // START_TASK
  // -------------------------------------------------------------------------
  if (event.type == NavigationEventType::START_TASK)
  {
    // Step 1: check task lock
    if (has_active_task_)
    {
      output.result = TaskHandleResult::REJECTED_BUSY;
      output.planner_action.type = PlannerActionType::NONE;
      return output;
    }

    // Step 2: validate task
    if (!isTaskValid(event.task))
    {
      output.result = TaskHandleResult::REJECTED_INVALID_TASK;
      output.planner_action.type = PlannerActionType::NONE;
      return output;
    }

    // Step 3: accept task
    active_task_ = event.task;
    active_task_.sequence = next_sequence_;
    ++next_sequence_;

    active_task_.max_vx = clampMaxVx(active_task_.max_vx);

    has_active_task_ = true;

    output.result = TaskHandleResult::STARTED;
    output.planner_action.type = PlannerActionType::SET_ROUTE;
    output.planner_action.task = active_task_;
    output.planner_action.max_vx = active_task_.max_vx;
    return output;
  }

  // -------------------------------------------------------------------------
  // CANCEL_TASK
  // -------------------------------------------------------------------------
  if (event.type == NavigationEventType::CANCEL_TASK)
  {
    if (has_active_task_)
    {
      const NavigationTask cancelled_task = active_task_;
      clearActiveTask();

      output.result = TaskHandleResult::CANCELLED;
      output.planner_action.type = PlannerActionType::CANCEL;
      output.planner_action.task = cancelled_task;
      output.planner_action.max_vx = cancelled_task.max_vx;
      return output;
    }

    output.result = TaskHandleResult::CANCEL_IGNORED;
    output.planner_action.type = PlannerActionType::NONE;
    return output;
  }

  // -------------------------------------------------------------------------
  // UPDATE_MAX_VX
  // -------------------------------------------------------------------------
  if (event.type == NavigationEventType::UPDATE_MAX_VX)
  {
    // No active task
    if (!has_active_task_)
    {
      output.result = TaskHandleResult::MAX_VX_UPDATE_IGNORED;
      output.planner_action.type = PlannerActionType::NONE;
      return output;
    }

    // Invalid speed
    if (!std::isfinite(event.max_vx) || event.max_vx <= 0.0)
    {
      output.result = TaskHandleResult::REJECTED_INVALID_MAX_VX;
      output.planner_action.type = PlannerActionType::NONE;
      return output;
    }

    // Clamp
    const double limited_max_vx = clampMaxVx(event.max_vx);

    // Unchanged
    if (std::fabs(limited_max_vx - active_task_.max_vx) <= 1e-9)
    {
      output.result = TaskHandleResult::MAX_VX_UNCHANGED;
      output.planner_action.type = PlannerActionType::NONE;
      return output;
    }

    // Update
    active_task_.max_vx = limited_max_vx;

    output.result = TaskHandleResult::MAX_VX_UPDATED;
    output.planner_action.type = PlannerActionType::UPDATE_SPEED_LIMIT;
    output.planner_action.task = active_task_;
    output.planner_action.max_vx = active_task_.max_vx;
    return output;
  }

  // -------------------------------------------------------------------------
  // Unsupported events
  // -------------------------------------------------------------------------
  output.result = TaskHandleResult::UNSUPPORTED_EVENT;
  output.planner_action.type = PlannerActionType::NONE;
  return output;
}

// =============================================================================
// hasActiveTask
// =============================================================================

bool TaskManager::hasActiveTask() const noexcept
{
  return has_active_task_;
}

// =============================================================================
// copyActiveTask
// =============================================================================

bool TaskManager::copyActiveTask(NavigationTask& task) const
{
  if (has_active_task_)
  {
    task = active_task_;
    return true;
  }

  task = NavigationTask{};
  return false;
}

// =============================================================================
// activeSequence
// =============================================================================

std::uint64_t TaskManager::activeSequence() const noexcept
{
  if (has_active_task_)
  {
    return active_task_.sequence;
  }
  return 0;
}

}  // namespace navdog
