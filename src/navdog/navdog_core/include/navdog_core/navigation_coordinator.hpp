#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/goal_controller.hpp"
#include "navdog_core/navigation_mode_manager.hpp"
#include "navdog_core/route_manager.hpp"
#include "navdog_core/route_corridor_observation_gate.hpp"
#include "navdog_core/route_follower.hpp"
#include "navdog_core/safety_supervisor.hpp"
#include "navdog_core/start_align_controller.hpp"

#include "navdog_core/types.hpp"

#include <navdog_task/task_manager.hpp>

#include <deque>
#include <memory>

namespace navdog
{

class NavigationCoordinator
{
public:
  /**
   * @brief 纯 C++ 导航协调器，连接任务会话、路线跟随、安全门和模式机。
   *
   * Runtime 是唯一调用者：它负责把 ROS/MQTT/SCAN 数据转换为 CoreInput。
   * 本类不访问 ROS、MQTT 或 /cmd_vel，所有副作用经 CoreOutput 返回。
   */
  explicit NavigationCoordinator(
      const NavdogConfig& config = NavdogConfig{},
      const navdog_task::TaskConfig& task_config =
          navdog_task::TaskConfig{});

  /** @brief 清除任务、路线、规划握手和控制器内部状态，回到 IDLE。 */
  void reset();

  /** @brief 处理已解析的任务事件；START/CANCEL 等约束由 navdog_task 保证。 */
  TaskHandleResult handleEvent(NavigationEvent event);

  /**
   * @brief 执行一个控制周期并返回纯 C++ 输出。
   *
   * 先处理路线确认握手（PLANNING 不是 SCAN 局部规划），再执行当前状态，
   * 最后仅对 Navdog Route 控制命令施加 SafetySupervisor。now_sec 必须有限、
   * 单调且与输入时间戳同一时间基准；无效时间会走现有 FAILED 安全路径。
   */
  CoreOutput update(
      const CoreInput& input,
      double now_sec);

  NavState state() const noexcept;

  bool hasActiveTask() const noexcept;

  const RouteManager& routeManager() const noexcept;
  const navdog_task::TaskSession& taskSession() const noexcept;

  const NavdogConfig& config() const noexcept;

private:
  friend class NavigationCoordinatorTestPeer;

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

  void resetNearGoalBlockedTimer() noexcept;

  NavdogConfig config_{};
  NavState state_{NavState::IDLE};
  navdog_task::TaskManager task_manager_{};
  RouteManager route_manager_{};
  std::deque<PlannerAction> pending_planner_actions_;

  bool planning_request_sent_{false};
  double planning_started_sec_{0.0};
  std::uint64_t expected_trajectory_id_{0};

  StartAlignController start_align_controller_{};

  RouteCorridorObservationGate
      route_corridor_observation_gate_{};

  NavigationModeManager navigation_mode_manager_{};

  RouteFollower route_follower_;
  GoalController goal_controller_;
  SafetySupervisor safety_supervisor_;

  NavigationMode last_mode_{NavigationMode::NONE};
  double near_goal_blocked_since_sec_{0.0};
  bool near_goal_blocked_timer_active_{false};
  NavState state_before_pause_{NavState::IDLE};
};

}  // namespace navdog
