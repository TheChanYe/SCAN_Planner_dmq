#include "navdog_core/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;

// clamp：将数值限制在 [min_value, max_value] 区间内。
double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

// isExplicitStopCommand：判断这是否是一个"上游主动请求的零速停止"指令
//（例如暂停/取消/失败等主动停止）：必须同时满足 vx/vy/yaw_rate 均为零，
// 且 source 属于已知的"显式停止"来源集合。命中后 apply() 会直接放行该零速指令，
// 不受传感器新鲜度等其他安全检查影响（因为零速本身就是安全的）。
bool isExplicitStopCommand(const VelocityCommand& cmd) noexcept
{
  const bool zero_motion =
      std::abs(cmd.vx) <= kEpsilon &&
      std::abs(cmd.vy) <= kEpsilon &&
      std::abs(cmd.yaw_rate) <= kEpsilon;
  if (!zero_motion)
    return false;

  switch (cmd.source)
  {
    case CommandSource::IDLE_STOP:
    case CommandSource::PLANNING_STOP:
    case CommandSource::TRACKING_STOP:
    case CommandSource::FAILED_STOP:
    case CommandSource::PAUSE_STOP:
    case CommandSource::CANCEL_STOP:
    case CommandSource::SAFETY_STOP:
      return true;
    default:
      return false;
  }
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

SafetySupervisor::SafetySupervisor(
    const SafetyConfig& config,
    const LimitConfig& limits)
    : safety_config_(config),
      limit_config_(limits)
{
}

// =============================================================================
// reset
// =============================================================================

// reset：清空上一周期输出记录，避免新任务开始时被旧速度影响加速度限制基准。
void SafetySupervisor::reset() noexcept
{
  previous_output_ = VelocityCommand{};
  previous_stamp_sec_ = 0.0;
  has_previous_output_ = false;
}

// safetyStop：生成一个零速安全指令。
// 步骤：构造 vx=vy=yaw_rate=0 的有效指令，打上时间戳与来源，
// 并同时将其写入 previous_output_/previous_stamp_sec_，作为下一周期加速度限制的新基准，
// 保证下次从零速重新加速时仍受加速度上限约束。
VelocityCommand SafetySupervisor::safetyStop(
    double now_sec,
    CommandSource source) noexcept
{
  VelocityCommand cmd{};
  cmd.vx = 0.0;
  cmd.vy = 0.0;
  cmd.yaw_rate = 0.0;
  cmd.stamp_sec = std::isfinite(now_sec) ? now_sec : 0.0;
  cmd.valid = true;
  cmd.source = source;
  previous_output_ = cmd;
  previous_stamp_sec_ = cmd.stamp_sec;
  has_previous_output_ = true;
  return cmd;
}

// =============================================================================
// checkTimeouts
// =============================================================================

// checkTimeouts：依次检查里程计、障碍物、地图三个数据源的时效性：
//   1. now_sec 本身必须是有限数；
//   2. 机器人状态必须有效且时间戳有限，且其龄期(now_sec-stamp)在
//      [-future_tolerance_sec, odom_timeout_sec] 范围内（既不能太旧也不能是"未来"时间戳）；
//   3. 障碍物数据同样需要有效且龄期在 [-future_tolerance_sec, obstacle_timeout_sec] 内；
//   4. 地图同样需要有效且龄期在同一阈值内（复用 obstacle_timeout_sec）。
// 任一项不满足则立即返回 false，调用方应视为不安全而触发停止（防定位退化/感知卡死）。
bool SafetySupervisor::checkTimeouts(
    const Context& context,
    double now_sec) const noexcept
{
  if (!std::isfinite(now_sec))
    return false;

  if (!context.robot.valid ||
      !std::isfinite(context.robot.stamp_sec))
  {
    return false;
  }

  const double odom_age = now_sec - context.robot.stamp_sec;
  if (odom_age > safety_config_.odom_timeout_sec ||
      odom_age < -safety_config_.future_tolerance_sec)
  {
    return false;
  }

  if (!context.obstacles.valid ||
      !std::isfinite(context.obstacles.stamp_sec))
  {
    return false;
  }

  const double obstacle_age = now_sec - context.obstacles.stamp_sec;
  if (obstacle_age > safety_config_.obstacle_timeout_sec ||
      obstacle_age < -safety_config_.future_tolerance_sec)
  {
    return false;
  }

  if (!context.map_valid || !std::isfinite(context.map_stamp_sec))
  {
    return false;
  }

  const double map_age = now_sec - context.map_stamp_sec;
  if (map_age > safety_config_.obstacle_timeout_sec ||
      map_age < -safety_config_.future_tolerance_sec)
  {
    return false;
  }

  return true;
}

// =============================================================================
// checkTrajectoryIdentity
// =============================================================================

// checkTrajectoryIdentity：校验上报的局部轨迹是否属于当前有效任务。
//   - 若根本没有上报轨迹（valid=false），认为无需校验，直接通过；
//   - 若有轨迹但持续时长非法（非正数），视为无效；
//   - 若 purpose 为 NONE 或 task_sequence 为 0，说明这是一条"没归属"的残留轨迹，
//     视为无效以防止旧轨迹未及时清除而被错误使用。
bool SafetySupervisor::checkTrajectoryIdentity(
    const Context& context) const noexcept
{
  if (!context.trajectory.valid)
    return true;  // No trajectory to validate.

  if (!std::isfinite(context.trajectory.duration_sec) ||
      context.trajectory.duration_sec <= 0.0)
  {
    return false;
  }

  // If the caller provided a trajectory, it must match the active identity.
  // When not tracking a local trajectory, callers should leave trajectory.valid
  // as false.
  if (context.trajectory.purpose == NavigationMode::NONE ||
      context.trajectory.task_sequence == 0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// computeFrontSpeedLimit
// =============================================================================

// computeFrontSpeedLimit：根据正前方最近障碍物距离计算一个 [0,1] 的前进速度缩放系数。
//   - 障碍物数据无效/距离非有限数时，认为没有前方障碍，返回无穷大（不限制）；
//   - 距离 <= emergency_stop（紧急停止阈值）时返回 0（必须停下）；
//   - 距离 >= slow_down_front（开始减速阈值）时返回无穷大（无需减速）；
//   - 中间距离区间内按线性插值换算成 [0,1] 系数（距离越近系数越小）。
double SafetySupervisor::computeFrontSpeedLimit(
    const ObstacleSummary& obstacles) const noexcept
{
  if (!obstacles.valid ||
      !std::isfinite(obstacles.front_min))
  {
    return std::numeric_limits<double>::infinity();
  }

  const double d = obstacles.front_min;

  if (d <= safety_config_.emergency_stop)
    return 0.0;

  if (d >= safety_config_.slow_down_front)
    return std::numeric_limits<double>::infinity();

  const double range =
      safety_config_.slow_down_front -
      safety_config_.emergency_stop;

  if (range < kEpsilon)
    return 0.0;

  return (d - safety_config_.emergency_stop) / range;
}

// =============================================================================
// computeYawRateSpeedPenalty
// =============================================================================

// computeYawRateSpeedPenalty：根据当前转向角速度占最大角速度的比例，
// 计算前进速度的惩罚系数 = 1 - |yaw_rate|/max_yaw_rate。
// 参数非法（非有限数、max_vx 过小、最大角速度过小）时直接返回 1.0（不惩罚）。
// 目的是让转得越快时允许的前进速度越低，避免转弯时打滑或过弯半径过大。
double SafetySupervisor::computeYawRateSpeedPenalty(
    double yaw_rate_cmd,
    double max_vx) const noexcept
{
  if (!std::isfinite(yaw_rate_cmd) ||
      !std::isfinite(max_vx) ||
      max_vx < kEpsilon ||
      limit_config_.max_yaw_rate < kEpsilon)
  {
    return 1.0;
  }

  const double ratio =
      std::abs(yaw_rate_cmd) / limit_config_.max_yaw_rate;

  return 1.0 - ratio;
}

// =============================================================================
// shouldApplyAccelerationLimit
// =============================================================================

// shouldApplyAccelerationLimit：判断本周期是否应对加速度进行限制。
//   - 无效指令不限制；
//   - 来源为 SAFETY_STOP/FAILED_STOP/PAUSE_STOP/CANCEL_STOP 等主动减速/停止场景不限制
//     （允许立即停下，不能因为加速度限制而延迟制动）；
//   - 地图无效时也不限制（后续会走到安全停止分支）；
//   - 其余情况下需要限制加速度。
bool SafetySupervisor::shouldApplyAccelerationLimit(
    const VelocityCommand& raw_cmd,
    const Context& context) const noexcept
{
  // Do not limit deceleration during safety stops or invalid commands.
  if (!raw_cmd.valid)
    return false;

  if (raw_cmd.source == CommandSource::SAFETY_STOP ||
      raw_cmd.source == CommandSource::FAILED_STOP ||
      raw_cmd.source == CommandSource::PAUSE_STOP ||
      raw_cmd.source == CommandSource::CANCEL_STOP)
  {
    return false;
  }

  if (!context.map_valid)
    return false;

  return true;
}

// =============================================================================
// apply
// =============================================================================

// apply：安全监督主入口，每个控制周期对上游输出的原始指令做完整的安全审查与限幅。
// 整体流程（按优先级从高到低）：
//   1. 无效指令 -> 直接安全停止；
//   2. vx/vy/yaw_rate 任一项非有限数(NaN/Inf) -> 安全停止；
//   3. 若是上游主动请求的显式零速停止（isExplicitStopCommand）-> 直接放行，
//      保持暂停/等待/取消语义不受传感器新鲜度影响（不削弱安全性）；
//   4. 时效性检查失败(checkTimeouts) 或地图无效 -> 安全停止；
//   5. 还没有历史输出记录（刚启动）-> 先输出一次停止作为基准；
//   6. 局部轨迹身份校验失败(checkTrajectoryIdentity) -> 安全停止；
//   7. 判断是否为 GOAL_ALIGN 或原地转向（vx=vy=0但yaw_rate非0），
//      这两种情况强制将 vx/vy 归零（immediate_linear_stop），避免原地转向时有残留平移量；
//   8. 计算动态最大线速度 effective_max_vx（任务上限与全局上限取较小值）；
//   9. 前方障碍物减速：用 computeFrontSpeedLimit 限制 limited_vx（不能为负）；
//  10. 转向惩罚：用 computeYawRateSpeedPenalty 进一步限制 limited_vx；
//  11. 侧向速度 limited_vy 直接限幅到 ±max_vy；
//  12. 角速度 limited_w 限幅到 ±min(max_yaw_rate, 0.65)（硬件安全上限）；
//  13. 判断前方紧急障碍(front_emergency_stop：距离<=紧急停止阈值)与局部脱困轨迹激活
//      (local_escape_active：当前轨迹目的为LOCAL_AVOID且属于当前任务)：
//      全向底盘即使前方紧急也应允许旋转/侧移/沿经碰撞检查过的局部避障轨迹小幅度倒退，
//      因此局部脱困时允许一个很小的前进封顶(kLocalEscapeForwardCapMps=0.06m/s)，
//      否则直接归零制止正向前进；
//  14. 加速度限制（仅当 shouldApplyAccelerationLimit 为 true 且已有历史输出时）：
//      根据 dt 和最大加速度计算本周期允许的最大变化量，将 vx/vy/yaw_rate 钳制在
//      [上次输出-最大变化, 上次输出+最大变化] 范围内（原地转向/对齐时不限制 vx/vy）；
//  15. 加速度限制之后再次复查前方紧急停止（防止加速度限制把前进速度"拉回来"）；
//  16. 构造最终指令，对接近零的分量做小幅度清零（避免浮点残留噪声）；
//  17. 根据最终指令与原始指令的差异重新标记 source：
//      若最终全零但原始非零 -> SAFETY_STOP；若任一分量被明显修改 -> SAFETY_SLOW；
//      否则保留原始来源（未受安全干预）；
//  18. 更新 previous_output_/previous_stamp_sec_/has_previous_output_，供下一周期使用。
VelocityCommand SafetySupervisor::apply(
    const VelocityCommand& raw_cmd,
    const Context& context,
    double max_vx,
    double now_sec)
{

  // Invalid raw command -> stop.
  if (!raw_cmd.valid)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // NaN/Inf guard.
  const bool raw_finite =
      std::isfinite(raw_cmd.vx) &&
      std::isfinite(raw_cmd.vy) &&
      std::isfinite(raw_cmd.yaw_rate);

  if (!raw_finite)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  // A controller-requested zero remains zero regardless of sensor freshness.
  // This preserves pause/wait/cancel semantics without weakening safety.
  if (isExplicitStopCommand(raw_cmd))
  {
    return safetyStop(now_sec, raw_cmd.source);
  }

  // Timeout / map validity guard.
  const bool timeouts_ok = checkTimeouts(context, now_sec);
  if (!timeouts_ok)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  if (!context.map_valid)
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  if (!has_previous_output_)
  {
    return safetyStop(now_sec, raw_cmd.source);
  }

  // Local trajectory identity guard.
  if (!checkTrajectoryIdentity(context))
  {
    return safetyStop(now_sec, CommandSource::SAFETY_STOP);
  }

  double limited_vx = raw_cmd.vx;
  double limited_vy = raw_cmd.vy;
  double limited_w = raw_cmd.yaw_rate;
  const bool goal_align =
      raw_cmd.source == CommandSource::GOAL_ALIGN;
  const bool turn_in_place =
      std::abs(raw_cmd.vx) <= kEpsilon &&
      std::abs(raw_cmd.vy) <= kEpsilon &&
      std::abs(raw_cmd.yaw_rate) > kEpsilon;
  const bool immediate_linear_stop = goal_align || turn_in_place;
  if (immediate_linear_stop)
  {
    limited_vx = 0.0;
    limited_vy = 0.0;
  }

  // Dynamic max_vx cap.
  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, limit_config_.max_vx));

  ObstacleSummary effective_obstacles = context.obstacles;
  if (context.prefer_route_corridor_front && context.corridor.valid &&
      !context.corridor.out_of_map)
  {
    effective_obstacles.front_min = context.corridor.blocked
        ? context.corridor.first_blocked_distance_ahead_m
        : std::numeric_limits<double>::infinity();
  }

  // Front obstacle slowdown.
  const double front_limit =
      computeFrontSpeedLimit(effective_obstacles);

  if (std::isfinite(front_limit) && front_limit < 1.0)
  {
    limited_vx = std::min(limited_vx, front_limit * effective_max_vx);
    limited_vx = std::max(0.0, limited_vx);
  }

  // Yaw rate penalty on forward speed.
  const double yaw_penalty =
      computeYawRateSpeedPenalty(limited_w, effective_max_vx);

  limited_vx = std::min(
      limited_vx, yaw_penalty * effective_max_vx);
  limited_vx = std::max(0.0, limited_vx);

  // Lateral limit.
  const double effective_max_vy =
      std::max(0.0, limit_config_.max_vy);
  limited_vy = clamp(limited_vy, -effective_max_vy, effective_max_vy);

  // Yaw rate limit.
  const double effective_max_w =
      std::max(0.0,
          std::min(limit_config_.max_yaw_rate, 0.65));
  limited_w = clamp(limited_w, -effective_max_w, effective_max_w);

  // A front emergency obstacle must stop positive forward
  // motion immediately, but an omnidirectional robot must still
  // be allowed to rotate, move laterally, or reverse along a
  // collision-checked LOCAL_AVOID trajectory.
  const bool front_emergency_stop =
      effective_obstacles.valid &&
      std::isfinite(effective_obstacles.front_min) &&
      effective_obstacles.front_min <=
          safety_config_.emergency_stop;

  const bool local_escape_active =
      context.trajectory.valid &&
      context.trajectory.purpose ==
          NavigationMode::LOCAL_AVOID &&
      context.trajectory.task_sequence != 0;

  constexpr double kLocalEscapeForwardCapMps = 0.06;

  if (front_emergency_stop && raw_cmd.vx > 0.0)
  {
    limited_vx = local_escape_active
        ? std::min(raw_cmd.vx, kLocalEscapeForwardCapMps)
        : 0.0;
  }

  // Acceleration limits.
  if (shouldApplyAccelerationLimit(raw_cmd, context) &&
      has_previous_output_ &&
      std::isfinite(previous_stamp_sec_))
  {
    const double dt = now_sec - previous_stamp_sec_;
    if (dt > 0.0 && std::isfinite(dt))
    {
      const double max_dvx = limit_config_.max_accel_x * dt;
      const double max_dvy = limit_config_.max_accel_y * dt;
      const double max_dw = limit_config_.max_accel_yaw * dt;

      if (!immediate_linear_stop)
      {
        limited_vx = clamp(
            limited_vx,
            previous_output_.vx - max_dvx,
            previous_output_.vx + max_dvx);
        limited_vy = clamp(
            limited_vy,
            previous_output_.vy - max_dvy,
            previous_output_.vy + max_dvy);
      }
      limited_w = clamp(
          limited_w,
          previous_output_.yaw_rate - max_dw,
          previous_output_.yaw_rate + max_dw);
    }
  }

  // Acceleration limiting must never reintroduce positive
  // forward velocity while the front emergency condition exists.
  if (front_emergency_stop && raw_cmd.vx > 0.0)
  {
    limited_vx = local_escape_active
        ? std::min(raw_cmd.vx, kLocalEscapeForwardCapMps)
        : 0.0;
  }

  VelocityCommand cmd{};
  cmd.vx = limited_vx;
  cmd.vy = limited_vy;
  cmd.yaw_rate = limited_w;
  cmd.stamp_sec = std::isfinite(now_sec) ? now_sec : 0.0;
  cmd.valid = true;

  if (cmd.vx < kEpsilon)
    cmd.vx = 0.0;
  if (std::abs(cmd.vy) < kEpsilon)
    cmd.vy = 0.0;
  if (std::abs(cmd.yaw_rate) < kEpsilon)
    cmd.yaw_rate = 0.0;

  // Preserve the original source when no safety intervention occurred.
  // Mark as safety-slow only when the command was actually reduced, and
  // safety-stop only when the final command is zero while the raw command
  // requested non-zero motion.
  const bool all_zero =
      cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.yaw_rate == 0.0;
  const bool raw_all_zero =
      raw_cmd.vx == 0.0 && raw_cmd.vy == 0.0 &&
      raw_cmd.yaw_rate == 0.0;

  if (all_zero && !raw_all_zero)
  {
    cmd.source = CommandSource::SAFETY_STOP;
  }
  else if (std::abs(cmd.vx - raw_cmd.vx) > kEpsilon ||
           std::abs(cmd.vy - raw_cmd.vy) > kEpsilon ||
           std::abs(cmd.yaw_rate - raw_cmd.yaw_rate) > kEpsilon)
  {
    cmd.source = CommandSource::SAFETY_SLOW;
  }
  else
  {
    cmd.source = raw_cmd.source;
  }

  previous_output_ = cmd;
  previous_stamp_sec_ = cmd.stamp_sec;
  has_previous_output_ = true;

  return cmd;
}

}  // namespace navdog
