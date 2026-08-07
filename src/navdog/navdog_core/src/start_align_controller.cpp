#include "navdog_core/start_align_controller.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

// =============================================================================
// Constructor
// 构造函数：保存对齐配置。
// =============================================================================

StartAlignController::StartAlignController(
    const StartAlignConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// 重置对齐阶段/目标解析/需要旋转标志与开始时刻/目标角全部内部状态。
// =============================================================================

void StartAlignController::reset() noexcept
{
  phase_started_ = false;
  target_resolved_ = false;
  rotation_required_ = false;
  phase_started_sec_ = 0.0;
  target_yaw_ = 0.0;
}

// =============================================================================
// active
// 返回对齐阶段是否已开始。
// =============================================================================

bool StartAlignController::active() const noexcept
{
  return phase_started_;
}

// =============================================================================
// normalizeAngle
// 将角度归一化到 (-π, π] 区间（通过atan2(sin,cos)实现）。
// =============================================================================

double StartAlignController::normalizeAngle(
    double angle) noexcept
{
  return std::atan2(
      std::sin(angle),
      std::cos(angle));
}

// =============================================================================
// degreesToRadians
// 度数转弧度。
// =============================================================================

double StartAlignController::degreesToRadians(
    double degrees) noexcept
{
  return degrees * 3.14159265358979323846 / 180.0;
}

// =============================================================================
// isConfigValid
// 校验所有配置字段均为有限数，且超时时长/比例增益/最大角速度/目标最小距离均为正数。
// =============================================================================

bool StartAlignController::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.enter_deg) ||
      !std::isfinite(config_.exit_deg) ||
      !std::isfinite(config_.max_hold_sec) ||
      !std::isfinite(config_.kp_yaw) ||
      !std::isfinite(config_.max_yaw_rate) ||
      !std::isfinite(config_.target_min_dist_m))
  {
    return false;
  }

  if (config_.enter_deg < 0.0 ||
      config_.exit_deg < 0.0)
  {
    return false;
  }

  if (config_.max_hold_sec <= 0.0 ||
      config_.kp_yaw <= 0.0 ||
      config_.max_yaw_rate <= 0.0 ||
      config_.target_min_dist_m <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isRobotUsable
// 校验机器人位姿是否为合理有限数。
// =============================================================================

bool StartAlignController::isRobotUsable(
    const RobotState& robot) const noexcept
{
  if (!robot.valid)
  {
    return false;
  }

  if (!std::isfinite(robot.x) ||
      !std::isfinite(robot.y) ||
      !std::isfinite(robot.yaw))
  {
    return false;
  }

  return true;
}

// =============================================================================
// resolveTargetYaw
// 按优先级解析对齐目标朝向角：
//   优先1：第一个足够长（≥target_min_dist_m）的路线直线段方向；
//   优先2：任何路线点自带的显式yaw；
//   优先3：机器人当前位置指向第一个足够远的路线点的方向角。
// 任一优先级命中即返回true，全部失败则返回false。
// =============================================================================

bool StartAlignController::resolveTargetYaw(
    const NavigationTask& task,
    const RobotState& robot,
    double& target_yaw) const noexcept
{
  const double min_dist =
      std::fabs(config_.target_min_dist_m);

  // --- Priority 1: first valid route segment ---
  if (task.points.size() >= 2)
  {
    for (std::size_t i = 0;
         i + 1 < task.points.size();
         ++i)
    {
      const RoutePoint& current = task.points[i];
      const RoutePoint& next = task.points[i + 1];

      const double dx = next.x - current.x;
      const double dy = next.y - current.y;
      const double distance = std::hypot(dx, dy);

      if (distance >= min_dist)
      {
        target_yaw = std::atan2(dy, dx);
        return true;
      }
    }
  }

  // --- Priority 2: explicit yaw on a route point ---
  for (const auto& point : task.points)
  {
    if (point.has_yaw &&
        std::isfinite(point.yaw))
    {
      target_yaw = normalizeAngle(point.yaw);
      return true;
    }
  }

  // --- Priority 3: robot pointing toward a target point ---
  if (isRobotUsable(robot))
  {
    for (const auto& point : task.points)
    {
      const double dx = point.x - robot.x;
      const double dy = point.y - robot.y;
      const double distance = std::hypot(dx, dy);

      if (distance >= min_dist)
      {
        target_yaw = std::atan2(dy, dx);
        return true;
      }
    }
  }

  return false;
}

// =============================================================================
// update
// 每周期驱动起点对齐。
// 步骤：
//   1.校验时间有限；2.校验配置合法；3.首次进入则记录开始时刻；
//   4.若目标朝向尚未解析，调用resolveTargetYaw尝试解析，失败且机器人不可用则等待
//      （并检查是否超时），失败且机器人可用却无法解析方向则判定任务无效；
//   5.等待机器人位姿可用（并同样检查超时）；6.计算当前与目标朝向的角度误差；
//   7.带滞后判断是否需要开始/继续旋转（enter角度触发旋转，exit角度判定对齐完成，
//      exit取enter/exit中较小者）；8.未对齐完成时按比例控制kp_yaw*yaw_error计算角速度
//      并限幅；9.最后检查是否超时（ALIGNED优先于超时），超时则强制角速度为0并标记TIMED_OUT。
// =============================================================================

StartAlignOutput StartAlignController::update(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec)
{
  // --- Safe default output ---
  StartAlignOutput output{};

  output.command.vx = 0.0;
  output.command.vy = 0.0;
  output.command.yaw_rate = 0.0;
  output.command.valid = true;
  output.command.source = CommandSource::START_ALIGN;
  output.command.stamp_sec =
      std::isfinite(now_sec) ? now_sec : 0.0;

  // --- 12.1 Check time ---
  if (!std::isfinite(now_sec))
  {
    output.result = StartAlignResult::INVALID_TIME;
    return output;
  }

  // --- 12.2 Check config ---
  if (!isConfigValid())
  {
    output.result = StartAlignResult::INVALID_CONFIG;
    return output;
  }

  // --- 12.3 Start alignment phase ---
  if (!phase_started_)
  {
    phase_started_ = true;
    phase_started_sec_ = now_sec;
  }

  // --- 12.4 Resolve target direction ---
  if (!target_resolved_)
  {
    double resolved_yaw = 0.0;

    if (resolveTargetYaw(task, robot, resolved_yaw))
    {
      target_yaw_ = normalizeAngle(resolved_yaw);
      target_resolved_ = true;
    }
    else
    {
      // Cannot resolve direction
      if (!isRobotUsable(robot))
      {
        // Robot not available yet; wait but check timeout
        output.result = StartAlignResult::WAITING_FOR_ROBOT;

        const double elapsed =
            now_sec - phase_started_sec_;
        const double max_hold =
            std::fabs(config_.max_hold_sec);

        if (elapsed > max_hold)
        {
          output.result = StartAlignResult::TIMED_OUT;
        }

        return output;
      }

      // Robot valid but direction cannot be resolved
      output.result = StartAlignResult::INVALID_TASK;
      return output;
    }
  }

  // --- 12.5 Wait for robot state ---
  if (!isRobotUsable(robot))
  {
    output.result = StartAlignResult::WAITING_FOR_ROBOT;

    // Check timeout even while waiting
    const double elapsed =
        now_sec - phase_started_sec_;
    const double max_hold =
        std::fabs(config_.max_hold_sec);

    if (elapsed > max_hold)
    {
      output.result = StartAlignResult::TIMED_OUT;
    }

    return output;
  }

  // --- 12.6 Compute angle error ---
  const double yaw_error =
      normalizeAngle(target_yaw_ - robot.yaw);

  output.target_yaw = target_yaw_;
  output.yaw_error = yaw_error;
  output.has_target = true;

  // --- Hysteresis thresholds ---
  const double enter_rad =
      degreesToRadians(
          std::max(
              std::fabs(config_.enter_deg),
              std::fabs(config_.exit_deg)));

  const double exit_rad =
      degreesToRadians(
          std::min(
              std::fabs(config_.enter_deg),
              std::fabs(config_.exit_deg)));

  // --- Not yet rotating ---
  if (!rotation_required_)
  {
    if (std::fabs(yaw_error) <= enter_rad)
    {
      output.result = StartAlignResult::ALIGNED;
      return output;
    }

    rotation_required_ = true;
  }

  // --- Already rotating: check exit threshold ---
  if (std::fabs(yaw_error) <= exit_rad)
  {
    output.result = StartAlignResult::ALIGNED;
    return output;
  }

  // --- Alignment in progress: compute yaw_rate ---
  double yaw_rate =
      config_.kp_yaw * yaw_error;

  const double max_yaw_rate =
      std::fabs(config_.max_yaw_rate);

  yaw_rate =
      std::max(
          -max_yaw_rate,
          std::min(max_yaw_rate, yaw_rate));

  output.command.yaw_rate = yaw_rate;
  output.result = StartAlignResult::ALIGNING;

  // --- Check timeout (ALIGNED takes priority) ---
  const double elapsed =
      now_sec - phase_started_sec_;
  const double max_hold =
      std::fabs(config_.max_hold_sec);

  if (elapsed > max_hold)
  {
    output.result = StartAlignResult::TIMED_OUT;
    output.command.yaw_rate = 0.0;
  }

  return output;
}

}  // namespace navdog
