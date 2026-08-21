#include "navdog_core/route_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultHeadingLookaheadM = 0.40;

struct RawTrackingPoint
{
  double arc_m{0.0};
  double x{0.0};
  double y{0.0};
};

// normalizeAngle：将任意弧度角归一化到 (-pi, pi]，避免转向时绕远路。
double normalizeAngle(double angle) noexcept
{
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

double perpendicularDistance(
    const RawTrackingPoint& p,
    const RawTrackingPoint& a,
    const RawTrackingPoint& b) noexcept
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = std::hypot(dx, dy);
  if (length < kEpsilon)
    return std::hypot(p.x - a.x, p.y - a.y);
  return std::abs(dy * p.x - dx * p.y + b.x * a.y - b.y * a.x) /
      length;
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

RouteFollower::RouteFollower(
    const RouteFollowerConfig& config)
    : config_(config)
{
  if (!std::isfinite(config_.heading_lookahead_m) ||
      config_.heading_lookahead_m < 0.0)
  {
    config_.heading_lookahead_m = kDefaultHeadingLookaheadM;
  }
}

void RouteFollower::rebuildTrackingPath(
    const NavigationTask& task)
{
  tracking_path_ready_ = true;
  tracking_task_sequence_ = task.sequence;
  tracking_path_.clear();

  std::vector<RawTrackingPoint> raw;
  raw.reserve(task.points.size());
  double raw_arc = 0.0;
  bool have_last = false;
  double last_x = 0.0;
  double last_y = 0.0;

  for (const RoutePoint& point : task.points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      continue;
    if (have_last)
      raw_arc += std::hypot(point.x - last_x, point.y - last_y);

    if (raw.empty() ||
        std::hypot(point.x - raw.back().x, point.y - raw.back().y) >=
            kEpsilon)
    {
      RawTrackingPoint raw_point{};
      raw_point.arc_m = raw_arc;
      raw_point.x = point.x;
      raw_point.y = point.y;
      raw.push_back(raw_point);
    }

    last_x = point.x;
    last_y = point.y;
    have_last = true;
  }

  if (raw.empty())
    return;

  if (raw.size() <= 2 || config_.simplify_tolerance_m <= 0.0 ||
      !std::isfinite(config_.simplify_tolerance_m))
  {
    tracking_path_.reserve(raw.size());
    for (const RawTrackingPoint& point : raw)
    {
      TrackingPoint tracking_point{};
      tracking_point.arc_m = point.arc_m;
      tracking_point.x = point.x;
      tracking_point.y = point.y;
      tracking_path_.push_back(tracking_point);
    }
    return;
  }

  std::vector<bool> keep(raw.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.emplace_back(0, raw.size() - 1);
  while (!stack.empty())
  {
    const auto range = stack.back();
    stack.pop_back();
    const std::size_t first = range.first;
    const std::size_t last = range.second;
    if (last <= first + 1)
      continue;

    double max_distance = -1.0;
    std::size_t max_index = first;
    for (std::size_t i = first + 1; i < last; ++i)
    {
      const double distance =
          perpendicularDistance(raw[i], raw[first], raw[last]);
      if (distance > max_distance)
      {
        max_distance = distance;
        max_index = i;
      }
    }

    if (max_distance > config_.simplify_tolerance_m)
    {
      keep[max_index] = true;
      stack.emplace_back(first, max_index);
      stack.emplace_back(max_index, last);
    }
  }

  tracking_path_.reserve(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i)
  {
    if (keep[i])
    {
      TrackingPoint tracking_point{};
      tracking_point.arc_m = raw[i].arc_m;
      tracking_point.x = raw[i].x;
      tracking_point.y = raw[i].y;
      tracking_path_.push_back(tracking_point);
    }
  }
}

// interpolateTrackingPoint：根据目标原始弧长，在派生tracking path上插值出对应位置。
bool RouteFollower::interpolateTrackingPoint(
    double target_arc_m,
    double& out_x,
    double& out_y) const noexcept
{
  out_x = 0.0;
  out_y = 0.0;

  if (tracking_path_.size() < 2)
    return false;

  if (target_arc_m <= tracking_path_.front().arc_m)
  {
    out_x = tracking_path_.front().x;
    out_y = tracking_path_.front().y;
    return true;
  }

  for (std::size_t i = 1; i < tracking_path_.size(); ++i)
  {
    const TrackingPoint& prev = tracking_path_[i - 1];
    const TrackingPoint& next = tracking_path_[i];
    const double arc_delta = next.arc_m - prev.arc_m;

    if (arc_delta < kEpsilon)
      continue;

    if (target_arc_m <= next.arc_m)
    {
      const double ratio =
          (target_arc_m - prev.arc_m) / arc_delta;
      out_x = prev.x + ratio * (next.x - prev.x);
      out_y = prev.y + ratio * (next.y - prev.y);
      return true;
    }
  }

  out_x = tracking_path_.back().x;
  out_y = tracking_path_.back().y;
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
//   4. 用"已走弧长 + 前瞻距离"作为目标弧长，在tracking path上求出前瞻点；
//      若插值失败（点数不足）则返回 TRACKING_STOP；
//   5. 在前瞻点附近采样局部路线切线，并只用前瞻点到机器人的横向误差修正方向；
//   6. 将真实前瞻点误差旋转到机器人自身坐标系（ex_robot/ey_robot）；
//   7. 前瞻点在机体后半平面时只原地转向；
//   8. 前瞻点在前半平面时使用 guidance alpha 生成唯一 yaw steering，vy始终为0；
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

  if (!tracking_path_ready_ ||
      task.sequence != tracking_task_sequence_)
    rebuildTrackingPath(task);

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

  if (!interpolateTrackingPoint(target_arc, look_x, look_y))
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  const double ex_world = look_x - robot.x;
  const double ey_world = look_y - robot.y;

  const double target_distance = std::hypot(ex_world, ey_world);
  if (target_distance < kEpsilon)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.stamp_sec = now_sec;
    cmd.valid = true;
    cmd.source = CommandSource::PLANNER;
    return cmd;
  }

  constexpr double kMinGuidanceScale = 0.05;
  const double heading_half_window =
      0.5 * std::max(0.0, config_.heading_lookahead_m);
  const double tangent_start_arc = std::max(
      progress.arc_length_m,
      target_arc - heading_half_window);
  const double tangent_end_arc = std::min(
      progress.total_length_m,
      target_arc + heading_half_window);

  double tangent_start_x = 0.0;
  double tangent_start_y = 0.0;
  double tangent_end_x = 0.0;
  double tangent_end_y = 0.0;
  const bool have_tangent_start =
      interpolateTrackingPoint(
          tangent_start_arc,
          tangent_start_x,
          tangent_start_y);
  const bool have_tangent_end =
      interpolateTrackingPoint(
          tangent_end_arc,
          tangent_end_x,
          tangent_end_y);

  double tangent_x = have_tangent_start && have_tangent_end
      ? tangent_end_x - tangent_start_x
      : 0.0;
  double tangent_y = have_tangent_start && have_tangent_end
      ? tangent_end_y - tangent_start_y
      : 0.0;
  double tangent_norm = std::hypot(tangent_x, tangent_y);
  if (tangent_norm < kEpsilon)
  {
    tangent_x = ex_world;
    tangent_y = ey_world;
    tangent_norm = std::hypot(tangent_x, tangent_y);
  }
  if (tangent_norm < kEpsilon)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.stamp_sec = now_sec;
    cmd.valid = true;
    cmd.source = CommandSource::PLANNER;
    return cmd;
  }
  tangent_x /= tangent_norm;
  tangent_y /= tangent_norm;

  const double normal_x = -tangent_y;
  const double normal_y = tangent_x;
  const double lateral_error =
      ex_world * normal_x +
      ey_world * normal_y;
  const double guidance_scale =
      std::max(kMinGuidanceScale, dynamic_lookahead);
  const double lateral_correction = std::max(
      -1.0,
      std::min(1.0, lateral_error / guidance_scale));
  double guide_x = tangent_x + lateral_correction * normal_x;
  double guide_y = tangent_y + lateral_correction * normal_y;
  const double guide_norm = std::hypot(guide_x, guide_y);
  if (guide_norm < kEpsilon)
  {
    guide_x = tangent_x;
    guide_y = tangent_y;
  }
  else
  {
    guide_x /= guide_norm;
    guide_y /= guide_norm;
  }

  const double guide_yaw = std::atan2(guide_y, guide_x);

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  // World error rotated into robot frame.
  const double ex_robot = c * ex_world + s * ey_world;
  const double ey_robot = -s * ex_world + c * ey_world;
  (void)ey_robot;
  const double alpha = normalizeAngle(guide_yaw - robot.yaw);
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
