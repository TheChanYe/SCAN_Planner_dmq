#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/task_manager.hpp"
#include "navdog_core/types.hpp"

#include <deque>

namespace navdog
{

class NavigationCoordinator
{
public:
  explicit NavigationCoordinator(
      const NavdogConfig& config = NavdogConfig{});

  void reset();

  TaskHandleResult handleEvent(
      const NavigationEvent& event);

  CoreOutput update(
      const CoreInput& input,
      double now_sec);

  NavState state() const noexcept;

  bool hasActiveTask() const noexcept;

  bool copyActiveTask(
      NavigationTask& task) const;

  const NavdogConfig& config() const noexcept;

private:
  void enqueuePlannerAction(
      const PlannerAction& action);

  PlannerAction takeNextPlannerAction();

  NavdogConfig config_{};
  NavState state_{NavState::IDLE};

  TaskManager task_manager_{};
  std::deque<PlannerAction> pending_planner_actions_;
};

}  // namespace navdog
