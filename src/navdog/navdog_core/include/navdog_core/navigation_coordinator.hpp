#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/goal_controller.hpp"
#include "navdog_core/navigation_mode_manager.hpp"
#include "navdog_core/rejoin_target_selector.hpp"
#include "navdog_core/route_corridor_observation_gate.hpp"
#include "navdog_core/route_follower.hpp"
#include "navdog_core/route_progress_tracker.hpp"
#include "navdog_core/safety_supervisor.hpp"
#include "navdog_core/start_align_controller.hpp"
#include "navdog_core/task_manager.hpp"
#include "navdog_core/trajectory_follower.hpp"
#include "navdog_core/types.hpp"

#include <deque>
#include <memory>

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

  // 注入局部规划适配器。Coordinator 不拥有其生命周期。
  void setLocalPlannerAdapter(
      LocalPlannerAdapter* adapter) noexcept;

  // 注入三维占据查询接口。Coordinator 不拥有其生命周期。
  void setOccupancyQuery(
      OccupancyQuery3D* query) noexcept;

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

  void clearLocalPlanningContext() noexcept;

  VelocityCommand makeZeroCommand(
      CommandSource source,
      double now_sec) const noexcept;

  VelocityCommand executeMode(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      const ObstacleSummary& obstacles,
      const RouteCorridorAssessment& corridor,
      bool corridor_available,
      double max_vx,
      double now_sec);

  VelocityCommand executeRouteFollow(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec);

  VelocityCommand executeLocalAvoid(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec);

  VelocityCommand executeRouteRejoin(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec);

  void requestLocalPlanIfNeeded(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      NavigationMode mode,
      double now_sec);

  bool needsNewLocalPlan(
      NavigationMode mode,
      const RouteProgress& progress,
      double now_sec) const;

  bool isTrajectoryHealthy(
      NavigationMode mode,
      const RouteProgress& progress,
      double now_sec) const;

  bool trajectoryEndingSoon(
      const LocalTrajectory& trajectory,
      double exec_time_sec) const noexcept;

  bool isTrajectoryExecutable(
      const LocalTrajectory& trajectory,
      NavigationMode expected_mode,
      std::uint64_t expected_task_sequence,
      std::uint64_t expected_plan_sequence) const noexcept;

  bool interpolateRoutePoint(
      const NavigationTask& task,
      double target_arc_length_m,
      RoutePoint& out) const noexcept;

  bool selectLocalAvoidTarget(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      RoutePoint& out_target) const;

  NavdogConfig config_{};
  NavState state_{NavState::IDLE};

  TaskManager task_manager_{};
  std::deque<PlannerAction> pending_planner_actions_;

  bool planning_request_sent_{false};
  double planning_started_sec_{0.0};
  std::uint64_t expected_trajectory_id_{0};

  StartAlignController start_align_controller_{};

  RouteProgressTracker route_progress_tracker_{};

  RouteCorridorObservationGate
      route_corridor_observation_gate_{};

  NavigationModeManager navigation_mode_manager_{};

  RouteFollower route_follower_;
  TrajectoryFollower trajectory_follower_;
  RejoinTargetSelector rejoin_target_selector_;
  GoalController goal_controller_;
  SafetySupervisor safety_supervisor_;

  LocalPlannerAdapter* local_planner_adapter_{nullptr};
  OccupancyQuery3D* occupancy_query_{nullptr};

  NavigationMode last_mode_{NavigationMode::NONE};
  std::uint64_t last_local_plan_task_sequence_{0};
  std::uint64_t last_local_plan_plan_sequence_{0};
  double last_local_plan_request_stamp_sec_{0.0};
  bool local_plan_request_pending_{false};
  bool last_local_plan_failed_{false};

  std::uint64_t expected_local_plan_sequence_{0};
  std::uint64_t expected_local_plan_task_sequence_{0};
  NavigationMode expected_local_plan_purpose_{NavigationMode::NONE};

  // Last requested target for change detection.
  double last_request_target_x_{0.0};
  double last_request_target_y_{0.0};

  bool force_replan_{false};
};

}  // namespace navdog
