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

// interpolateRoutePoint：根据目标累积弧长，在任务的折线路线上插值出对应位置。
// 步骤：
//   1. 若路线点数少于2个，无法构成线段，直接返回失败；
//   2. 若目标弧长<=0，直接返回起点位置；
//   3. 沿着路线逐段累加段长，一旦累加长度超过目标弧长，说明目标点落在这一段内，
//      按比例 ratio 在该段两端点之间线性插值出 x/y；
//   4. 若遍历完所有段仍未达到目标弧长（目标点超出路线总长），则钳到路线终点。
// 输入：task - 路线点列；target_arc_length_m - 目标累积弧长（米）
// 输出：out_x/out_y - 插值结果；返回值表示是否成功。
bool RouteFollower::interpolateRoutePoint(
    const NavigationTask& task,
    double target_arc_length_m,
    double& out_x,
    double& out_y) const noexcept
{
  out_x = 0.0;
  out_y = 0.0;

  const auto& points = task.points;
  if (points.size() < 2)
    return false;

  if (target_arc_length_m <= 0.0)
  {
    out_x = points.front().x;
    out_y = points.front().y;
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
      return true;
    }

    accumulated += seg_len;
  }

  // target_arc_length_m beyond route end.
  out_x = points.back().x;
  out_y = points.back().y;
  return true;
}

// updatePointGoal：单点/极短路线的直达模式（不用前瞻插值）。
// 步骤：
//   1. 前置校验：路线为空/机器人无效/进度无效，直接返回 TRACKING_STOP；
//   2. 以任务最后一个点为目标，计算目标在机体坐标系下的 ex/ey；
//   3. 目标在后半平面时只输出比例角速度，不向前走；
//   4. 目标在前半平面时根据距离与 cos(alpha)^2 平滑控制前进速度；
//   5. yaw_rate 使用简单比例控制，适合短距离点目标，避免曲率数值放大。
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
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  const double ex_robot = c * dx + s * dy;
  const double ey_robot = -s * dx + c * dy;
  const double alpha = distance > kEpsilon
      ? std::atan2(ey_robot, ex_robot)
      : 0.0;
  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, config_.max_vx));
  const double yaw_rate = std::max(
      -config_.max_yaw_rate,
      std::min(config_.max_yaw_rate, config_.kp_yaw * alpha));

  if (ex_robot <= 0.0 && distance > kEpsilon)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate = yaw_rate;
  }
  else
  {
    const double base_speed = std::min(
        effective_max_vx,
        config_.kp_x * std::max(0.0, distance));
    const double cos_alpha = std::max(0.0, std::cos(alpha));
    cmd.vx = base_speed * cos_alpha * cos_alpha;
    cmd.vy = 0.0;
    cmd.yaw_rate = yaw_rate;
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
//   3. 根据当前可执行速度计算动态前瞻距离 dynamic_lookahead：
//      基础前瞻 + 速度*前瞻时间，并限幅到 max_lookahead_distance_m（跑得越快看得越远）；
//   4. 用"已走弧长 + 前瞻距离"作为目标弧长，调用 interpolateRoutePoint 求出前瞻点；
//      若插值失败（点数不足）则返回 TRACKING_STOP；
//   5. 直接朝向前瞻点位置行驶（而不是用该点所在段的切线方向），
//      这样可以避免在每个路径拐点处因切线方向突变而停下对齐（轰磨现象）；
//   6. 将世界坐标系下的误差旋转到机器人自身坐标系（ex_robot/ey_robot）；
//   7. 前瞻点在机体后半平面时只原地转向；
//   8. 前瞻点在前半平面时使用连续 pure-pursuit 曲率生成唯一 yaw steering，vy始终为0；
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
  const double speed_hint = effective_max_vx;
  const double dynamic_lookahead = std::min(
      max_lookahead,
      base_lookahead + speed_hint *
          std::max(0.0, config_.lookahead_time_sec));
  const double target_arc =
      progress.arc_length_m + dynamic_lookahead;

  double look_x = 0.0;
  double look_y = 0.0;

  if (!interpolateRoutePoint(
          task, target_arc, look_x, look_y))
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  const double ex_world = look_x - robot.x;
  const double ey_world = look_y - robot.y;

  const double target_distance = std::hypot(ex_world, ey_world);

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  // World error rotated into robot frame.
  const double ex_robot = c * ex_world + s * ey_world;
  const double ey_robot = -s * ex_world + c * ey_world;
  const double alpha = std::atan2(ey_robot, ex_robot);
  const double lookahead_actual = std::hypot(ex_robot, ey_robot);

  if (ex_robot <= 0.0)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate = std::max(
        -config_.max_yaw_rate,
        std::min(config_.max_yaw_rate, config_.kp_yaw * alpha));
  }
  else
  {
    const double base_speed = std::min(
        effective_max_vx,
        config_.kp_x * std::max(0.0, target_distance));
    const double cos_alpha = std::max(0.0, std::cos(alpha));
    cmd.vy = 0.0;
    constexpr double kMinLookahead = 0.05;
    constexpr double kCurvatureEpsilon = 1e-6;
    const double ld = std::max(kMinLookahead, lookahead_actual);
    const double curvature = 2.0 * std::sin(alpha) / ld;
    double steering_speed = base_speed;
    if (std::abs(curvature) > kCurvatureEpsilon &&
        config_.kp_yaw > kCurvatureEpsilon)
    {
      const double curve_speed_limit =
          config_.max_yaw_rate /
          (config_.kp_yaw * std::abs(curvature));
      steering_speed = std::min(
          steering_speed,
          std::max(0.0, curve_speed_limit));
    }
    cmd.vx = steering_speed * cos_alpha * cos_alpha;
    cmd.yaw_rate = std::max(
        -config_.max_yaw_rate,
        std::min(config_.max_yaw_rate,
            config_.kp_yaw * steering_speed * curvature));
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
