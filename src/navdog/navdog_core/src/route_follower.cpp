#include "navdog_core/route_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

// normalizeAngle：将任意弧度角归一化到 (-pi, pi]，避免转向时绕远路。
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

RouteFollower::RouteFollower(
    const RouteFollowerConfig& config)
    : config_(config)
{
}

// interpolateRoutePoint：根据目标累积弧长，在任务的折线路线上插值出对应位置与朝向。
// 步骤：
//   1. 若路线点数少于2个，无法构成线段，直接返回失败；
//   2. 若目标弧长<=0，直接返回起点位置/朝向；
//   3. 沿着路线逐段累加段长，一旦累加长度超过目标弧长，说明目标点落在这一段内，
//      按比例 ratio 在该段两端点之间线性插值出 x/y；
//   4. 朝向默认取该段的切线方向 atan2(dy,dx)，若两端点都带显式 yaw，
//      则改为对两端 yaw 做角度插值（先归一化差值再按比例加回去）；
//   5. 若遍历完所有段仍未达到目标弧长（目标点超出路线总长），则钳到路线终点。
// 输入：task - 路线点列；target_arc_length_m - 目标累积弧长（米）
// 输出：out_x/out_y/out_yaw - 插值结果；返回值表示是否成功。
bool RouteFollower::interpolateRoutePoint(
    const NavigationTask& task,
    double target_arc_length_m,
    double& out_x,
    double& out_y,
    double& out_yaw) const noexcept
{
  out_x = 0.0;
  out_y = 0.0;
  out_yaw = 0.0;

  const auto& points = task.points;
  if (points.size() < 2)
    return false;

  if (target_arc_length_m <= 0.0)
  {
    out_x = points.front().x;
    out_y = points.front().y;
    out_yaw = points.front().has_yaw
        ? points.front().yaw
        : 0.0;
    return true;
  }

  double accumulated = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double seg_len = std::hypot(dx, dy);

    if (seg_len < kEpsilon)
      continue;

    if (accumulated + seg_len >= target_arc_length_m)
    {
      const double ratio =
          (target_arc_length_m - accumulated) / seg_len;
      out_x = points[i - 1].x + ratio * dx;
      out_y = points[i - 1].y + ratio * dy;

      out_yaw = std::atan2(dy, dx);
      if (points[i - 1].has_yaw && points[i].has_yaw)
      {
        double yaw_diff = normalizeAngle(
            points[i].yaw - points[i - 1].yaw);
        out_yaw = normalizeAngle(
            points[i - 1].yaw + ratio * yaw_diff);
      }

      return true;
    }

    accumulated += seg_len;
  }

  // target_arc_length_m beyond route end.
  out_x = points.back().x;
  out_y = points.back().y;
  out_yaw = points.back().has_yaw
      ? points.back().yaw
      : std::atan2(
            points.back().y - points[points.size() - 2].y,
            points.back().x - points[points.size() - 2].x);
  return true;
}

// =============================================================================
// isYawAligned
// =============================================================================

// isYawAligned：判断朝向误差绝对值是否在"仅转向"阈值以内，
// 只有对齐后才允许同时平移，避免朝向偏差过大时斜向乱走。
bool RouteFollower::isYawAligned(
    double heading_error) const noexcept
{
  return std::abs(heading_error) <=
         config_.heading_turn_only_threshold_rad;
}

// updatePointGoal：单点/极短路线的直达模式（不用前瞻插值）。
// 步骤：
//   1. 前置校验：路线为空/机器人无效/进度无效，直接返回 TRACKING_STOP；
//   2. 以任务最后一个点为目标，计算相对机器人的距离和朝向角 desired_yaw
//      （距离过近时直接沿用当前机器人朝向，避免除零风险）；
//   3. 若未对齐：只输出比例角速度（限幅到 ±kp_yaw），不向前走；
//   4. 若已对齐：根据距离计算目标速度（近距离时额外减速），
//      同时根据机器人坐标系下的横向偏差 lateral_error 算出 vy，yaw_rate 继续比例修正；
//   5. 对 vx/vy/yaw_rate 做非有限数兜底、对 vx 做非负且不超过最大速度限幅。
// 来源标记为 PLANNER，表示这是常规路线跟踪输出。
VelocityCommand RouteFollower::updatePointGoal(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double now_sec) const
{
  VelocityCommand cmd{};
  cmd.stamp_sec = now_sec;
  if (task.points.empty() || !robot.valid || !progress.valid)
  {
    cmd.source = CommandSource::TRACKING_STOP;
    return cmd;
  }

  const RoutePoint& target = task.points.back();
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double distance = std::hypot(dx, dy);
  const double desired_yaw = distance > kEpsilon
      ? std::atan2(dy, dx)
      : robot.yaw;
  const double heading_error = normalizeAngle(desired_yaw - robot.yaw);
  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, config_.max_vx));

  if (!isYawAligned(heading_error))
  {
    cmd.yaw_rate = std::max(
        -config_.kp_yaw,
        std::min(config_.kp_yaw, config_.kp_yaw * heading_error));
  }
  else
  {
    double target_speed = std::min(
        effective_max_vx,
        std::max(0.10, config_.kp_x * distance));
    if (distance < 0.30)
      target_speed = std::min(effective_max_vx, config_.kp_x * distance);

    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    const double lateral_error = -s * dx + c * dy;
    cmd.vx = target_speed;
    cmd.vy = config_.kp_y * lateral_error;
    cmd.yaw_rate = config_.kp_yaw * heading_error;
  }

  if (!std::isfinite(cmd.vx)) cmd.vx = 0.0;
  if (!std::isfinite(cmd.vy)) cmd.vy = 0.0;
  if (!std::isfinite(cmd.yaw_rate)) cmd.yaw_rate = 0.0;
  cmd.vx = std::max(0.0, std::min(cmd.vx, effective_max_vx));
  cmd.valid = true;
  cmd.source = CommandSource::PLANNER;
  return cmd;
}

// update：带前瞻点的路线跟踪主逻辑，每个控制周期调用一次。核心步骤：
//   1. 前置校验：路线为空/进度无效/弧长非法/机器人无效，直接返回 TRACKING_STOP；
//   2. 若只有单个点或路线总长度接近零，退化为 updatePointGoal 直达模式；
//   3. 根据当前实测速度计算动态前瞻距离 dynamic_lookahead：
//      基础前瞻 + 速度*前瞻时间，并限幅到 max_lookahead_distance_m（跑得越快看得越远）；
//   4. 用"已走弧长 + 前瞻距离"作为目标弧长，调用 interpolateRoutePoint 求出前瞻点；
//      若插值失败（点数不足）则返回 TRACKING_STOP；
//   5. 直接朝向前瞻点位置行驶（而不是用该点所在段的切线方向），
//      这样可以避免在每个路径拐点处因切线方向突变而停下对齐（轰磨现象）；
//   6. 将世界坐标系下的误差旋转到机器人自身坐标系（ex_robot/ey_robot）；
//   7. 若未对齐：只输出比例角速度（限幅），不向前平移；
//   8. 若已对齐：根据朝向误差大小计算一个减速系数 heading_speed_scale
//      （误差越接近"仅转向"阈值，速度越低，实现还未完全对齐时提前减速避免过弯），
//      用它限制前进速度上限，实际前进速度取该上限与比例控制 kp_x*ex_robot 的较小值，
//      vy 用横向误差比例控制，yaw_rate 继续比例修正；
//   9. 最终对 vx/vy/yaw_rate 做非有限数及负值兜底、对 vx 限幅到最大速度。
// 输入：task/robot/progress/max_vx/now_sec；输出：VelocityCommand。
VelocityCommand RouteFollower::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double now_sec)
{
  VelocityCommand cmd{};

  if (task.points.empty() ||
      !progress.valid ||
      !std::isfinite(progress.arc_length_m) ||
      !std::isfinite(progress.total_length_m) ||
      progress.arc_length_m < 0.0 ||
      !robot.valid)
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  if (task.points.size() == 1 || progress.total_length_m <= 1e-6)
    return updatePointGoal(task, robot, progress, max_vx, now_sec);

  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, config_.max_vx));

  const double base_lookahead =
      std::max(0.0, config_.lookahead_distance_m);
  const double max_lookahead =
      std::max(base_lookahead, config_.max_lookahead_distance_m);
  const double measured_speed =
      std::isfinite(robot.vx) && std::isfinite(robot.vy)
          ? std::hypot(robot.vx, robot.vy)
          : 0.0;
  const double dynamic_lookahead = std::min(
      max_lookahead,
      base_lookahead + measured_speed *
          std::max(0.0, config_.lookahead_time_sec));
  const double target_arc =
      progress.arc_length_m + dynamic_lookahead;

  double look_x = 0.0;
  double look_y = 0.0;
  double look_yaw = 0.0;

  if (!interpolateRoutePoint(
          task, target_arc, look_x, look_y, look_yaw))
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  const double ex_world = look_x - robot.x;
  const double ey_world = look_y - robot.y;

  // Aim at the lookahead point instead of using the tangent of whichever
  // polyline segment contains it. Segment tangents jump at every waypoint
  // and made the physical dog stop and realign at otherwise gentle corners.
  const double target_distance = std::hypot(ex_world, ey_world);
  const double desired_yaw = target_distance > kEpsilon
      ? std::atan2(ey_world, ex_world)
      : look_yaw;
  const double heading_error =
      normalizeAngle(desired_yaw - robot.yaw);

  const bool aligned = isYawAligned(heading_error);

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  // World error rotated into robot frame.
  const double ex_robot = c * ex_world + s * ey_world;
  const double ey_robot = -s * ex_world + c * ey_world;

  if (!aligned)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate =
        std::max(-config_.kp_yaw,
            std::min(config_.kp_yaw,
                config_.kp_yaw * heading_error));
  }
  else
  {
    const double abs_heading_error = std::abs(heading_error);
    const double slowdown_start = std::max(
        0.0, std::min(config_.heading_slowdown_start_rad,
                      config_.heading_turn_only_threshold_rad));
    double heading_speed_scale = 1.0;
    if (abs_heading_error > slowdown_start)
    {
      const double slowdown_range =
          config_.heading_turn_only_threshold_rad - slowdown_start;
      heading_speed_scale = slowdown_range > kEpsilon
          ? (config_.heading_turn_only_threshold_rad - abs_heading_error) /
                slowdown_range
          : 0.0;
      heading_speed_scale = std::max(0.0,
          std::min(1.0, heading_speed_scale));
    }

    const double heading_limited_vx =
        effective_max_vx * heading_speed_scale;
    cmd.vx = std::min(
        heading_limited_vx,
        config_.kp_x * std::max(0.0, ex_robot));
    cmd.vy = config_.kp_y * ey_robot;
    cmd.yaw_rate = config_.kp_yaw * heading_error;
  }

  // Clamp and sanitize.
  if (!std::isfinite(cmd.vx) || cmd.vx < 0.0)
    cmd.vx = 0.0;
  cmd.vx = std::min(cmd.vx, effective_max_vx);

  if (!std::isfinite(cmd.vy))
    cmd.vy = 0.0;

  if (!std::isfinite(cmd.yaw_rate))
    cmd.yaw_rate = 0.0;

  cmd.stamp_sec = now_sec;
  cmd.valid = true;
  cmd.source = CommandSource::PLANNER;

  return cmd;
}

}  // namespace navdog
