#include "navdog_task/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

/**
 * @brief TaskManager
 * 任务是否合法
    当前有没有任务
    任务编号
    任务模式
    暂停、继续、取消
    最大速度更新
 */

namespace navdog_task
{

TaskManager::TaskManager(const TaskConfig& config) : config_(config) {}

void TaskManager::reset() noexcept
{
  session_ = TaskSession{};
}

/**
 * @brief isTaskModeValid
 * 检测任务模式是否合法
 */
bool TaskManager::isTaskModeValid(TaskMode mode) noexcept
{
  return mode == TaskMode::NORMAL_AVOID || mode == TaskMode::ROUTE_ONLY ||
         mode == TaskMode::CHARGING;
}

/**
 * @brief isTaskValid
 * 检测任务是否合法
 * mode是否合法
 * points是否为空
 * max_vx是否有效且大于0
 * 每个点的x/y/z/yaw是否有效
 */
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
/**
 * @brief clampMaxVx
 * 限制最大速度
 */
double TaskManager::clampMaxVx(double value) const noexcept
{
  const double low = std::min(config_.min_max_vx, config_.max_max_vx);
  const double high = std::max(config_.min_max_vx, config_.max_max_vx);
  return std::max(low, std::min(high, value));
}
/**
 * @brief handleEvent
 * 处理导航事件,
 * 它是任务层唯一主入口。
 *  处理开始任务时： 
 *  检查当前是否已有任务。
 *  检查点是否为空。
 *  检查坐标和速度是否是有限数。
 *  为任务生成新的内部sequence。
 *  对max_vx限幅。
 *  保存任务会话。
 *  把路线交给上层。 
 *  开始任务时并不是直接使用MQTT中的sequence，而是由TaskManager维护内部递增序号。
 */
TaskTransition TaskManager::handleEvent(NavigationEvent event)
{
  TaskTransition transition{};
  transition.session = session_;

  /**
   * @brief 状态机
   * 输入：导航事件
   */
  switch (event.type)
  {
    case NavigationEventType::NONE: // 无效事件
      return transition;
    case NavigationEventType::START_TASK: // 开始任务
      if (session_.active) // 如果当前有任务正在执行，拒绝新任务
      {
        transition.result = TaskHandleResult::REJECTED_BUSY;
        return transition;
      }
      if (!isTaskValid(event.task)) // 任务不合法，拒绝新任务
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
    case NavigationEventType::CANCEL_TASK: // 取消任务，如果当前没有任务正在执行，忽略
      if (!session_.active)
      {
        transition.result = TaskHandleResult::CANCEL_IGNORED;
        return transition;
      }
      session_ = TaskSession{};
      transition.result = TaskHandleResult::CANCELLED;
      transition.route_changed = true;
      break;
    case NavigationEventType::PAUSE: // 暂停任务，如果当前没有任务正在执行，忽略
    case NavigationEventType::RESUME: // 继续任务，如果当前没有任务正在执行，忽略
      if (!session_.active)
      {
        transition.result = TaskHandleResult::PAUSE_RESUME_IGNORED;
        return transition;
      }
      session_.paused = event.type == NavigationEventType::PAUSE;
      transition.result = session_.paused ? TaskHandleResult::PAUSED
                                          : TaskHandleResult::RESUMED;
      break;
    case NavigationEventType::UPDATE_MAX_VX: // 更新最大速度，如果当前没有任务正在执行，忽略
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
/**
 * @brief session
 * 获取当前任务会话
 */
const TaskSession& TaskManager::session() const noexcept { return session_; }
/**
 * @brief hasActiveTask
 * 检查是否有活动任务
 */
bool TaskManager::hasActiveTask() const noexcept { return session_.active; }

}  // namespace navdog_task
