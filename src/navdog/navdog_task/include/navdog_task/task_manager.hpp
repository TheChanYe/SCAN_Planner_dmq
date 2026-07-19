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

  void reset() noexcept;  // 重置任务管理器状态
  TaskTransition handleEvent(NavigationEvent event); // 处理导航事件并返回任务转换结果
  const TaskSession& session() const noexcept; // 获取当前任务会话状态
  bool hasActiveTask() const noexcept; // 检查是否有正在执行的任务

private:
  bool isTaskValid(const NavigationTask& task) const noexcept;  // 检测任务是否有效
  static bool isTaskModeValid(TaskMode mode) noexcept;          // 检测任务模式是否有效
  double clampMaxVx(double max_vx) const noexcept;              // 限制最大速度在配置范围内

  TaskConfig config_{};     // 配置参数,                                        
  TaskSession session_{};   // 当前任务会话状态
  std::uint64_t next_sequence_{1}; // 下一个任务序列号
};

}  // namespace navdog_task
