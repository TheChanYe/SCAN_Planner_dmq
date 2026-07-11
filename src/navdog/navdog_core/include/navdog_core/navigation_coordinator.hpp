#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/route_corridor_evaluator.hpp"
#include "navdog_core/route_progress_tracker.hpp"
#include "navdog_core/start_align_controller.hpp"
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

  void clearPlanningContext() noexcept;

  bool startPlanningContext(
      const PlannerAction& set_route_action,
      double now_sec) noexcept;

  bool isPlannerFeedbackUsable(
      const PlannerFeedback& feedback,
      double now_sec) const noexcept;

  void updatePlanningState(
      const PlannerFeedback& feedback,
      double now_sec);

  void enterFailedState() noexcept;

  void resetStartAlign() noexcept;

  VelocityCommand makeZeroCommand(
      CommandSource source,
      double now_sec) const noexcept;

  NavdogConfig config_{};
  NavState state_{NavState::IDLE};

  TaskManager task_manager_{};
  std::deque<PlannerAction> pending_planner_actions_;

  bool planning_request_sent_{false};
  double planning_started_sec_{0.0};
  std::uint64_t expected_trajectory_id_{0};

  StartAlignController start_align_controller_{};

  RouteProgressTracker route_progress_tracker_{};

  RouteCorridorEvaluator route_corridor_evaluator_{};
};

}  // namespace navdog
