#pragma once

#include "navdog_task/task_types.hpp"

/**
 * @brief 导航任务管理器类，负责处理导航事件并维护任务会话状态
 */
namespace navdog_task
{

class TaskManager
{
public:
  explicit TaskManager(const TaskConfig& config = TaskConfig{});

  /** @brief 清空当前会话；不回退 next_sequence_，避免旧异步反馈误关联到新任务。 */
  void reset() noexcept;
  /** @brief 处理 START/CANCEL/PAUSE/RESUME 等事件并返回一次可观察的会话转换。 */
  TaskTransition handleEvent(NavigationEvent event);
  const TaskSession& session() const noexcept; // 获取当前任务会话状态
  bool hasActiveTask() const noexcept; // 检查是否有正在执行的任务

private:
  /** @brief 在不访问外部系统的前提下校验路线数值和任务模式。 */
  bool isTaskValid(const NavigationTask& task) const noexcept;
  static bool isTaskModeValid(TaskMode mode) noexcept;          // 检测任务模式是否有效
  /** @brief 将有限的请求速度限在 TaskConfig 范围，单位 m/s。 */
  double clampMaxVx(double max_vx) const noexcept;

  TaskConfig config_{};     // 配置参数,                                        
  TaskSession session_{};   // 当前任务会话状态
  std::uint64_t next_sequence_{1}; // 下一个任务序列号
};

}  // namespace navdog_task
