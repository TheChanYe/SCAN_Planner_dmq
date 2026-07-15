#pragma once

#include "navdog_task/task_types.hpp"

namespace navdog_task
{

class TaskManager
{
public:
  explicit TaskManager(const TaskConfig& config = TaskConfig{});

  void reset() noexcept;
  TaskTransition handleEvent(NavigationEvent event);
  const TaskSession& session() const noexcept;
  bool hasActiveTask() const noexcept;

private:
  bool isTaskValid(const NavigationTask& task) const noexcept;
  static bool isTaskModeValid(TaskMode mode) noexcept;
  double clampMaxVx(double max_vx) const noexcept;

  TaskConfig config_{};
  TaskSession session_{};
  std::uint64_t next_sequence_{1};
};

}  // namespace navdog_task
