#include "navdog_core/goal_controller.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

// normalizeAngle：把任意弧度角归一化到 (-pi, pi] 区间。
// 输入：angle - 原始角度差（弧度，可能远超 ±pi）
// 输出：归一化后的角度，用于避免"绕远路"转向（例如 -350度 应转成 +10度）
double normalizeAngle(double angle) noexcept
{
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

GoalController::GoalController(
    const GoalControllerConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// =============================================================================

// reset：清空对齐计时状态，在任务取消/重规划/终点丢失时调用，
// 避免下次对齐时误用旧的计时起点。
void GoalController::reset() noexcept
{
  align_started_sec_ = 0.0;
  align_timer_active_ = false;
}

// =============================================================================
// isNearGoal
// =============================================================================

// isNearGoal：判断路线剩余距离是否已进入近终点区间，供上层状态机
// 决定是否从 TRACKING 切换到 GOAL_ALIGN。进度无效或剩余距离非有限数时返回 false。
bool GoalController::isNearGoal(
    const RouteProgress& progress) const noexcept
{
  if (!progress.valid ||
      !std::isfinite(progress.remaining_distance_m))
  {
    return false;
  }

  return progress.remaining_distance_m <=
         config_.near_goal_switch_dist;
}

// =============================================================================
// update
// =============================================================================

// update：终点对齐主逻辑，每个控制周期调用一次，核心步骤如下：
//   1. 前置校验：任务点为空 / 路线进度无效 / 机器人状态无效，直接返回无效指令；
//   2. 取任务最后一个目标点作为终点，坐标非有限数则同样返回无效指令；
//   3. 计算机器人到终点的直线距离 dist，以及朝向误差 yaw_error
//      （优先用目标点自带朝向，否则退化用路线朝向，并做角度归一化避免绕远路）；
//   4. 若 dist 超过 finish_dist 和 goal_align_reacquire_dist 中的较大值，
//      认为终点"丢失"（例如中途被推离太远），下发零速、reset()计时器、
//      并置位 position_lost=true，交由上层重新规划；
//   5. 首次进入本函数时启动对齐计时器（align_started_sec_/align_timer_active_）；
//   6. 计算对齐已用时长，若超过 goal_align_timeout_sec 则视为超时，
//      作为兜底防止因朝向误差长期收敛不到阈值而卡死在 GOAL_ALIGN；
//   7. 若"位置达标且朝向达标"或"对齐超时"，则下发零速并标记 finished=true
//      （timed_out 区分是真正达标还是超时兜底）；
//   8. 否则进入原地转向阶段：只输出 yaw_rate（vx/vy 恒为0），
//      用比例控制 near_goal_kp_w * yaw_error 并限幅到 effective_max_w；
//   9. 为避免角速度太小导致转向"磨洋工"，若未达标且角速度低于
//      goal_align_min_yaw_rate，则按误差符号强制拉到最小角速度；
//  10. 最终对 vx/vy/yaw_rate 做非有限数兜底（NaN/Inf 时清零），保证输出安全。
GoalController::Result GoalController::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double max_yaw_rate,
    double now_sec)
{
  Result result{};
  result.command.stamp_sec = now_sec;
  result.command.source = CommandSource::GOAL_ALIGN;

  if (task.points.empty() ||
      !progress.valid ||
      !robot.valid)
  {
    result.command.valid = false;
    return result;
  }

  const RoutePoint goal = task.points.back();

  if (!std::isfinite(goal.x) ||
      !std::isfinite(goal.y) ||
      !std::isfinite(robot.x) ||
      !std::isfinite(robot.y))
  {
    result.command.valid = false;
    return result;
  }

  const double dx = goal.x - robot.x;
  const double dy = goal.y - robot.y;
  const double dist = std::hypot(dx, dy);

  const double goal_yaw = goal.has_yaw ? goal.yaw : progress.route_yaw;
  const double yaw_error =
      std::isfinite(goal_yaw)
          ? normalizeAngle(goal_yaw - robot.yaw)
          : 0.0;

  const bool position_reached = dist <= config_.finish_dist;
  const bool yaw_reached =
      std::abs(yaw_error) <=
      config_.finish_yaw_tolerance_rad;

  if (dist > std::max(config_.finish_dist,
                      config_.goal_align_reacquire_dist))
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate = 0.0;
    result.command.valid = true;
    result.position_lost = true;
    reset();
    return result;
  }

  if (!align_timer_active_ && std::isfinite(now_sec))
  {
    align_started_sec_ = now_sec;
    align_timer_active_ = true;
  }

  const double align_elapsed = now_sec - align_started_sec_;
  const bool align_timed_out = align_timer_active_ &&
      std::isfinite(align_elapsed) &&
      config_.goal_align_timeout_sec > 0.0 &&
      align_elapsed >= config_.goal_align_timeout_sec;

  if ((position_reached && yaw_reached) || align_timed_out)
  {
    result.command.vx = 0.0;
    result.command.vy = 0.0;
    result.command.yaw_rate = 0.0;
    result.command.valid = true;
    result.finished = true;
    result.timed_out = align_timed_out;
    return result;
  }

  (void)max_vx;
  (void)position_reached;
  const double effective_max_w =
      std::max(0.0,
          std::min(max_yaw_rate, config_.near_goal_max_w));
  result.command.vx = 0.0;
  result.command.vy = 0.0;
  result.command.yaw_rate =
      std::max(-effective_max_w,
          std::min(effective_max_w,
              config_.near_goal_kp_w * yaw_error));

  const double minimum_yaw_rate = std::min(
      effective_max_w, std::max(0.0, config_.goal_align_min_yaw_rate));
  if (!yaw_reached && minimum_yaw_rate > 0.0 &&
      std::abs(result.command.yaw_rate) < minimum_yaw_rate)
  {
    result.command.yaw_rate = std::copysign(minimum_yaw_rate, yaw_error);
  }

  if (!std::isfinite(result.command.vx))
    result.command.vx = 0.0;
  if (!std::isfinite(result.command.vy))
    result.command.vy = 0.0;
  if (!std::isfinite(result.command.yaw_rate))
    result.command.yaw_rate = 0.0;

  result.command.valid = true;
  return result;
}

}  // namespace navdog
