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

  /** @brief 返回当前导航状态机状态。 */
  NavState state() const noexcept;

  /** @brief 返回是否存在一个活动中的任务。 */
  bool hasActiveTask() const noexcept;

  /** @brief 返回内部路线管理器的只读引用。 */
  const RouteManager& routeManager() const noexcept;
  /** @brief 返回当前任务会话的只读引用。 */
  const navdog_task::TaskSession& taskSession() const noexcept;

  /** @brief 返回当前使用的全部配置。 */
  const NavdogConfig& config() const noexcept;

  /**
   * @brief 外部执行后端出现不可恢复故障时，将当前活动导航任务终止为FAILED。
   *
   * Core 不知道 ROS/SCAN/MQTT 失败来源，只表达当前导航执行已经不可继续。
   */
  bool failActiveTask() noexcept;

private:
  friend class NavigationCoordinatorTestPeer;

  // enqueuePlannerAction：将一个规划器动作加入待发送队列（CANCEL会先清空队列）。
  void enqueuePlannerAction(
      const PlannerAction& action);

  // takeNextPlannerAction：弹出并返回下一个待发规划器动作（队列为空时返回默认值）。
  PlannerAction takeNextPlannerAction();

  // clearPlanningContext：清除当前规划握手上下文（发送标志/时刻/期望轨迹ID）。
  void clearPlanningContext() noexcept;

  // startPlanningContext：开启一次新的规划握手，返回是否成功。
  bool startPlanningContext(
      const PlannerAction& set_route_action,
      double now_sec) noexcept;

  // isPlannerFeedbackUsable：校验规划器反馈是否属于当前规划且时间/ID合法。
  bool isPlannerFeedbackUsable(
      const PlannerFeedback& feedback,
      double now_sec) const noexcept;

  // updatePlanningState：在PLANNING状态下根据规划反馈/超时推进状态机。
  void updatePlanningState(
      const PlannerFeedback& feedback,
      double now_sec);

  // enterFailedState：进入FAILED并清理所有与当前任务相关的中间状态。
  void enterFailedState() noexcept;

  // makeZeroCommand：构造一个带来源标记与时间戳的全零速度指令。
  VelocityCommand makeZeroCommand(
      CommandSource source,
      double now_sec) const noexcept;

  // executeMode：先统一处理最终目标边界，再根据当前导航模式分发到
  // executeRouteFollow/executeLocalAvoid，并处理“靠近终点但被阻”的超时逻辑。
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

  // executeRouteFollow：ROUTE_FOLLOW模式执行逻辑，处理路线阻挡、近终点限速并
  // 交给RouteFollower；最终目标边界由executeMode统一处理。
  VelocityCommand executeRouteFollow(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec);

  // executeLocalAvoid：LOCAL_AVOID模式下不产生实际控制量，只返回零速度（实际控制由
  // SCAN原生闭环控制器产生并由Mux层选择）。
  VelocityCommand executeLocalAvoid(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec);

  // resetNearGoalBlockedTimer：重置“靠近终点但被阻挡”计时器。
  void resetNearGoalBlockedTimer() noexcept;

  NavdogConfig config_{};                    // 全部配置
  NavState state_{NavState::IDLE};           // 导航状态机当前状态
  navdog_task::TaskManager task_manager_{};  // 任务会话管理器
  RouteManager route_manager_{};             // 路线与进度管理器
  std::deque<PlannerAction> pending_planner_actions_;  // 待发送给规划器的动作队列

  bool planning_request_sent_{false};        // 当前规划握手是否已发送请求
  double planning_started_sec_{0.0};         // 规划请求发送时刻
  std::uint64_t expected_trajectory_id_{0};  // 期望的规划反馈轨迹ID

  StartAlignController start_align_controller_{};  // 起点对齐子控制器

  RouteCorridorObservationGate
      route_corridor_observation_gate_{};  // 路径观测门控

  NavigationModeManager navigation_mode_manager_{};  // ROUTE_FOLLOW/LOCAL_AVOID模式切换状态机

  RouteFollower route_follower_;    // 路线跟随子控制器
  GoalController goal_controller_;  // 终点对齐子控制器
  SafetySupervisor safety_supervisor_;  // 最终安全监督层

  NavigationMode last_mode_{NavigationMode::NONE};  // 上一周期的导航模式，用于检测模式切换
  double near_goal_blocked_since_sec_{0.0};   // “靠近终点但被阻挡”开始计时的时刻
  bool near_goal_blocked_timer_active_{false};  // 该计时器是否处于计时中
  bool obstacle_finished_{false};             // 最近一次成功是否由近终点障碍超时触发
  NavState state_before_pause_{NavState::IDLE};  // 暂停前的状态，用于恢复时回退
};

}  // namespace navdog
