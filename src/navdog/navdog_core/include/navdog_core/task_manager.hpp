#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// =============================================================================
// TaskHandleResult
// =============================================================================

enum class TaskHandleResult : std::uint8_t
{
  NONE = 0,

  STARTED,
  REJECTED_BUSY,
  REJECTED_INVALID_TASK,

  CANCELLED,
  CANCEL_IGNORED,

  MAX_VX_UPDATED,
  MAX_VX_UNCHANGED,
  MAX_VX_UPDATE_IGNORED,
  REJECTED_INVALID_MAX_VX,

  UNSUPPORTED_EVENT
};

// =============================================================================
// TaskManagerOutput
// =============================================================================

struct TaskManagerOutput
{
  TaskHandleResult result{TaskHandleResult::NONE};
  PlannerAction planner_action{};
};

// =============================================================================
// TaskManager
// =============================================================================

class TaskManager
{
public:
  explicit TaskManager(
      const TaskConfig& config = TaskConfig{});

  void reset();

  TaskManagerOutput handleEvent(
      const NavigationEvent& event);

  bool hasActiveTask() const noexcept;

  bool copyActiveTask(
      NavigationTask& task) const;

  std::uint64_t activeSequence() const noexcept;

private:
  bool isTaskValid(
      const NavigationTask& task) const noexcept;

  bool isTaskModeValid(
      TaskMode mode) const noexcept;

  double clampMaxVx(
      double max_vx) const noexcept;

  void clearActiveTask();

  TaskConfig config_{};

  bool has_active_task_{false};
  NavigationTask active_task_{};

  std::uint64_t next_sequence_{1};
};

}  // namespace navdog
