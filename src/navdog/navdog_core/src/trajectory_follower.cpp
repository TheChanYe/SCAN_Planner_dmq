#include "navdog_core/trajectory_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxControlDtSec = 0.2;

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
// 构造函数：保存局部轨迹跟随控制参数配置。
// =============================================================================

TrajectoryFollower::TrajectoryFollower(
    const TrajectoryFollowerConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// 清零轨迹执行时钟与上次更新时间戳标志，并清空当前跟随的任务/规划序号与导航模式，
// 用于任务切换或轨迹失效时重新开始跟踪。
// =============================================================================

void TrajectoryFollower::reset() noexcept
{
  exec_time_sec_ = 0.0;
  last_update_stamp_sec_ = 0.0;
  has_last_update_stamp_ = false;

  active_task_sequence_ = 0;
  active_plan_sequence_ = 0;
  active_purpose_ = NavigationMode::NONE;
}

// =============================================================================
// trajectoryTimeSec
// 返回当前轨迹的本地执行时钟（从0开始随真实dt累加）。
// =============================================================================

double TrajectoryFollower::trajectoryTimeSec() const noexcept
{
  return exec_time_sec_;
}

// =============================================================================
// sampleTrajectory
// 在轨迹点序列上按时间t_eval做插值采样，得到该时刻的期望位置/速度/朝向。
// 步骤：1.仅一个点或t_eval<=0时直接取首点；2.t_eval达到或超过总时长时取末点；
// 3.否则在覆盖t_eval的相邻两点区间内做线性插值（若区间时长趋近0则直接取右端点）；
// 4.朝向优先取两端点自带的yaw做角度差归一化插值，仅一端有yaw则直接用该yaw，
//   两端都没有yaw则用位移方向近似（并标记has_yaw为false）。
// 找不到覆盖区间（异常情况）返回false。
// =============================================================================

bool TrajectoryFollower::sampleTrajectory(
    const LocalTrajectory& trajectory,
    double t_eval,
    double& out_x,
    double& out_y,
    double& out_vx,
    double& out_vy,
    double& out_yaw,
    bool& out_has_yaw) const noexcept
{
  out_x = 0.0;
  out_y = 0.0;
  out_vx = 0.0;
  out_vy = 0.0;
  out_yaw = 0.0;
  out_has_yaw = false;

  const auto& points = trajectory.points;
  if (points.empty())
    return false;

  if (points.size() == 1 || t_eval <= 0.0)
  {
    out_x = points.front().x;
    out_y = points.front().y;
    out_vx = points.front().vx;
    out_vy = points.front().vy;
    out_yaw = points.front().yaw;
    out_has_yaw = points.front().has_yaw;
    return true;
  }

  if (t_eval >= trajectory.duration_sec)
  {
    out_x = points.back().x;
    out_y = points.back().y;
    out_vx = points.back().vx;
    out_vy = points.back().vy;
    out_yaw = points.back().yaw;
    out_has_yaw = points.back().has_yaw;
    return true;
  }

  for (std::size_t i = 1; i < points.size(); ++i)
  {
    if (t_eval >= points[i - 1].time_from_start_sec &&
        t_eval <= points[i].time_from_start_sec)
    {
      const double dt =
          points[i].time_from_start_sec -
          points[i - 1].time_from_start_sec;
      if (dt < kEpsilon)
      {
        out_x = points[i].x;
        out_y = points[i].y;
        out_vx = points[i].vx;
        out_vy = points[i].vy;
        out_yaw = points[i].yaw;
        out_has_yaw = points[i].has_yaw;
        return true;
      }

      const double ratio =
          (t_eval - points[i - 1].time_from_start_sec) / dt;
      out_x = points[i - 1].x + ratio *
          (points[i].x - points[i - 1].x);
      out_y = points[i - 1].y + ratio *
          (points[i].y - points[i - 1].y);
      out_vx = points[i - 1].vx + ratio *
          (points[i].vx - points[i - 1].vx);
      out_vy = points[i - 1].vy + ratio *
          (points[i].vy - points[i - 1].vy);

      if (points[i - 1].has_yaw && points[i].has_yaw)
      {
        const double yaw_diff = normalizeAngle(
            points[i].yaw - points[i - 1].yaw);
        out_yaw = normalizeAngle(
            points[i - 1].yaw + ratio * yaw_diff);
        out_has_yaw = true;
      }
      else if (points[i].has_yaw)
      {
        out_yaw = points[i].yaw;
        out_has_yaw = true;
      }
      else if (points[i - 1].has_yaw)
      {
        out_yaw = points[i - 1].yaw;
        out_has_yaw = true;
      }
      else
      {
        out_yaw = std::atan2(
            points[i].y - points[i - 1].y,
            points[i].x - points[i - 1].x);
        out_has_yaw = false;
      }

      return true;
    }
  }

  return false;
}

// =============================================================================
// update
// 每周期驱动局部轨迹跟随，输出一帧速度指令。
// 步骤：
//   1.先构造默认的无效停止指令；2.校验轨迹本身合法性（非空、时长为正、
//      对应的导航模式与任务序号与期望一致、时间戳有限），任一不满足则reset()并返回停止；
//   3.若轨迹身份（任务/规划序号/导航用途）发生变化，视为新轨迹，reset()后重新记录身份；
//   4.首次更新记录时间戳基准，之后计算本周期dt；dt为负（时间回退）视为异常，reset()后停止；
//   5.若控制间隔dt过大（超过kMaxControlDtSec），本帧不推进轨迹执行时钟（避免大跳变）；
//   6.在当前执行时刻exec_time_sec_上采样得到期望位置/速度/朝向；
//   7.若采样点本身无朝向，则向前看time_forward_sec采样一个前瞻点，用位移方向
//      （退化时用速度方向）近似期望朝向；
//   8.按比例控制kp_yaw*朝向误差计算角速度并限幅；
//   9.按比例控制kp_pos*位置误差叠加期望速度得到世界系期望速度，并按朝向误差余弦做
//      衰减（朝向偏差越大，允许的平动速度越小，避免侧滑冲过头）；
//   10.对世界系速度按总速度上限限幅，再旋转到机体系得到vx/vy；
//   11.对vx/vy/yaw_rate做最终限幅与NaN兜底；12.仅当dt未超限时才推进执行时钟。
// =============================================================================

VelocityCommand TrajectoryFollower::update(
    const LocalTrajectory& trajectory,
    const RobotState& robot,
    double max_vx,
    double max_vy,
    double max_yaw_rate,
    NavigationMode expected_mode,
    std::uint64_t expected_task_sequence,
    double now_sec)
{
  VelocityCommand cmd{};
  cmd.stamp_sec = now_sec;
  cmd.source = CommandSource::TRACKING_STOP;
  cmd.valid = false;

  if (!trajectory.valid ||
      trajectory.points.empty() ||
      !std::isfinite(trajectory.duration_sec) ||
      trajectory.duration_sec <= 0.0 ||
      trajectory.purpose != expected_mode ||
      trajectory.task_sequence != expected_task_sequence ||
      !std::isfinite(trajectory.source_stamp_sec) ||
      !std::isfinite(now_sec))
  {
    reset();
    return cmd;
  }

  // Identity changed: reset execution time.
  const bool identity_changed =
      trajectory.task_sequence != active_task_sequence_ ||
      trajectory.plan_sequence != active_plan_sequence_ ||
      trajectory.purpose != active_purpose_;

  if (identity_changed)
  {
    reset();
    active_task_sequence_ = trajectory.task_sequence;
    active_plan_sequence_ = trajectory.plan_sequence;
    active_purpose_ = trajectory.purpose;
  }

  if (!has_last_update_stamp_)
  {
    last_update_stamp_sec_ = now_sec;
    has_last_update_stamp_ = true;
    exec_time_sec_ = 0.0;
  }

  const double dt = now_sec - last_update_stamp_sec_;
  last_update_stamp_sec_ = now_sec;

  // Time regression is invalid.
  if (dt < 0.0)
  {
    reset();
    return cmd;
  }

  // Large control gap: do not advance trajectory time this frame.
  const bool advance_time = dt <= kMaxControlDtSec;

  double pos_des_x = 0.0;
  double pos_des_y = 0.0;
  double vel_des_x = 0.0;
  double vel_des_y = 0.0;
  double yaw_des = 0.0;
  bool has_yaw = false;

  if (!sampleTrajectory(
          trajectory,
          exec_time_sec_,
          pos_des_x,
          pos_des_y,
          vel_des_x,
          vel_des_y,
          yaw_des,
          has_yaw))
  {
    reset();
    return cmd;
  }

  if (has_yaw)
  {
    yaw_des = normalizeAngle(yaw_des);
  }
  else
  {
    // Look ahead for yaw.
    double look_x = 0.0;
    double look_y = 0.0;
    double look_vx = 0.0;
    double look_vy = 0.0;
    double look_yaw = 0.0;
    bool look_has_yaw = false;

    const double t_look = std::min(
        trajectory.duration_sec,
        exec_time_sec_ + config_.time_forward_sec);

    if (sampleTrajectory(
            trajectory,
            t_look,
            look_x,
            look_y,
            look_vx,
            look_vy,
            look_yaw,
            look_has_yaw))
    {
      double dx = look_x - pos_des_x;
      double dy = look_y - pos_des_y;
      if (dx * dx + dy * dy < 1e-4)
      {
        dx = vel_des_x;
        dy = vel_des_y;
      }
      if (dx * dx + dy * dy >= 1e-4)
      {
        yaw_des = std::atan2(dy, dx);
        has_yaw = true;
      }
    }
  }

  const double yaw_err =
      has_yaw ? normalizeAngle(yaw_des - robot.yaw) : 0.0;
  const double vyaw_cmd =
      std::max(-max_yaw_rate,
          std::min(max_yaw_rate,
              config_.kp_yaw * yaw_err));

  const double pos_err_x = pos_des_x - robot.x;
  const double pos_err_y = pos_des_y - robot.y;

  double vel_world_x = vel_des_x + config_.kp_pos * pos_err_x;
  double vel_world_y = vel_des_y + config_.kp_pos * pos_err_y;
  const double heading_scale = std::max(
      0.0,
      std::cos(std::min(std::abs(yaw_err), kPi * 0.5)));
  vel_world_x *= heading_scale;
  vel_world_y *= heading_scale;

  const double norm = std::hypot(vel_world_x, vel_world_y);
  const double max_world_v = std::max(max_vx, max_vy);
  if (norm > max_world_v && norm > kEpsilon)
  {
    vel_world_x = vel_world_x / norm * max_world_v;
    vel_world_y = vel_world_y / norm * max_world_v;
  }

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  cmd.vx = c * vel_world_x + s * vel_world_y;
  cmd.vy = -s * vel_world_x + c * vel_world_y;
  cmd.yaw_rate = vyaw_cmd;

  // Clamp.
  cmd.vx = std::max(-max_vx, std::min(max_vx, cmd.vx));
  cmd.vy = std::max(-max_vy, std::min(max_vy, cmd.vy));
  cmd.yaw_rate =
      std::max(-max_yaw_rate, std::min(max_yaw_rate, cmd.yaw_rate));

  if (!std::isfinite(cmd.vx))
    cmd.vx = 0.0;
  if (!std::isfinite(cmd.vy))
    cmd.vy = 0.0;
  if (!std::isfinite(cmd.yaw_rate))
    cmd.yaw_rate = 0.0;

  cmd.source = CommandSource::PLANNER;
  cmd.valid = true;

  if (advance_time)
  {
    exec_time_sec_ += dt;
  }

  return cmd;
}

}  // namespace navdog
