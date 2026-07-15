#include "navdog_task/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navdog_task
{

TaskManager::TaskManager(const TaskConfig& config) : config_(config) {}

void TaskManager::reset() noexcept
{
  session_ = TaskSession{};
}

bool TaskManager::isTaskModeValid(TaskMode mode) noexcept
{
  return mode == TaskMode::NORMAL_AVOID || mode == TaskMode::ROUTE_ONLY ||
         mode == TaskMode::CHARGING;
}

bool TaskManager::isTaskValid(const NavigationTask& task) const noexcept
{
  if (!isTaskModeValid(task.mode) || task.points.empty() ||
      !std::isfinite(task.max_vx) || task.max_vx <= 0.0)
    return false;
  for (const auto& point : task.points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) ||
        (point.has_yaw && !std::isfinite(point.yaw)))
      return false;
  }
  return true;
}

double TaskManager::clampMaxVx(double value) const noexcept
{
  const double low = std::min(config_.min_max_vx, config_.max_max_vx);
  const double high = std::max(config_.min_max_vx, config_.max_max_vx);
  return std::max(low, std::min(high, value));
}

TaskTransition TaskManager::handleEvent(NavigationEvent event)
{
  TaskTransition transition{};
  transition.session = session_;

  switch (event.type)
  {
    case NavigationEventType::NONE:
      return transition;
    case NavigationEventType::START_TASK:
      if (session_.active)
      {
        transition.result = TaskHandleResult::REJECTED_BUSY;
        return transition;
      }
      if (!isTaskValid(event.task))
      {
        transition.result = TaskHandleResult::REJECTED_INVALID_TASK;
        return transition;
      }
      session_.sequence = next_sequence_++;
      session_.mode = event.task.mode;
      session_.max_vx = clampMaxVx(event.task.max_vx);
      session_.active = true;
      session_.paused = false;
      transition.result = TaskHandleResult::STARTED;
      transition.accepted_route = std::move(event.task.points);
      transition.route_changed = true;
      break;
    case NavigationEventType::CANCEL_TASK:
      if (!session_.active)
      {
        transition.result = TaskHandleResult::CANCEL_IGNORED;
        return transition;
      }
      session_ = TaskSession{};
      transition.result = TaskHandleResult::CANCELLED;
      transition.route_changed = true;
      break;
    case NavigationEventType::PAUSE:
    case NavigationEventType::RESUME:
      if (!session_.active)
      {
        transition.result = TaskHandleResult::PAUSE_RESUME_IGNORED;
        return transition;
      }
      session_.paused = event.type == NavigationEventType::PAUSE;
      transition.result = session_.paused ? TaskHandleResult::PAUSED
                                          : TaskHandleResult::RESUMED;
      break;
    case NavigationEventType::UPDATE_MAX_VX:
      if (!session_.active)
      {
        transition.result = TaskHandleResult::MAX_VX_UPDATE_IGNORED;
        return transition;
      }
      if (!std::isfinite(event.max_vx) || event.max_vx <= 0.0)
      {
        transition.result = TaskHandleResult::REJECTED_INVALID_MAX_VX;
        return transition;
      }
      {
        const double max_vx = clampMaxVx(event.max_vx);
        if (std::fabs(max_vx - session_.max_vx) <= 1e-9)
          transition.result = TaskHandleResult::MAX_VX_UNCHANGED;
        else
        {
          session_.max_vx = max_vx;
          transition.result = TaskHandleResult::MAX_VX_UPDATED;
        }
      }
      break;
    default:
      transition.result = TaskHandleResult::UNSUPPORTED_EVENT;
      return transition;
  }

  transition.session = session_;
  return transition;
}

const TaskSession& TaskManager::session() const noexcept { return session_; }
bool TaskManager::hasActiveTask() const noexcept { return session_.active; }

}  // namespace navdog_task
