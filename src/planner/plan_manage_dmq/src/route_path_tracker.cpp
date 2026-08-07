#include "plan_manage_dmq/route_path_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner_dmq
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kDuplicatePointDistanceM = 0.01;
constexpr double kPi = 3.14159265358979323846;

// normalizeAngle：将角度归一化到[-pi, pi]区间。
double normalizeAngle(double angle) noexcept
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

// clamp：将value限制在[low, high]区间内。
double clamp(double value, double low, double high) noexcept
{
  return std::max(low, std::min(high, value));
}

// finitePoint：校验三维点的三个分量是否均为有限数。
bool finitePoint(const Eigen::Vector3d& point) noexcept
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

}  // namespace

// 构造函数：保存跟踪器配置。
RoutePathTracker::RoutePathTracker(const RoutePathTrackerConfig& config)
    : config_(config)
{
}

// setPath：设置新路径点列。步骤：1.先重置当前状态；2.过滤掉非有限点与与上一个
// 保留点过于接近的重复点（避免零长度段引发数值问题）；3.若有效点少于2个则
// 重置并返回false；4.计算每个点的累计弧长；5.若总长度接近0则重置并返回false；
// 6.标记需要重新全路径搜索投影点。
bool RoutePathTracker::setPath(
    const std::vector<Eigen::Vector3d>& points)
{
  reset();
  points_.reserve(points.size());
  for (const auto& point : points)
  {
    if (!finitePoint(point)) continue;
    if (!points_.empty() &&
        (point - points_.back()).head<2>().norm() <
            kDuplicatePointDistanceM)
      continue;
    points_.push_back(point);
  }

  if (points_.size() < 2)
  {
    reset();
    return false;
  }

  cumulative_length_.resize(points_.size(), 0.0);
  for (std::size_t i = 1; i < points_.size(); ++i)
  {
    const double segment_length =
        (points_[i] - points_[i - 1]).head<2>().norm();
    cumulative_length_[i] = cumulative_length_[i - 1] + segment_length;
  }

  if (cumulative_length_.back() <= kEpsilon)
  {
    reset();
    return false;
  }

  reacquire_requested_ = true;
  return true;
}

// reset：清空路径点、累计弧长与进度状态，并标记需重新搜索投影。
void RoutePathTracker::reset() noexcept
{
  points_.clear();
  cumulative_length_.clear();
  segment_index_ = 0;
  progress_m_ = 0.0;
  reacquire_requested_ = true;
}

// requestReacquire：标记下一次projectProgress需从头全路径搜索（而非从上次
// 段附近局部搜索）。
void RoutePathTracker::requestReacquire() noexcept
{
  reacquire_requested_ = true;
}

// projectProgress：将机器人当前位置投影到路径上，找到距离最近且弧长不小于当前
// 进度（防止进度回退）的投影点。
// 步骤：1.若未要求重新搜索，仅从当前段开始搜索到前搜索上限search_end，否则
// 从头全路径搜索；2.对每一段计算机器人到该段的最近点（比例坐标ratio夹在[0,1]）及
// 对应弧长；3.跳过弧长明显小于当前进度的段（防止倒退）；4.保留最小距离对应的
// 弧长与段索引。输出的projected_arc不会小于当前进度。
bool RoutePathTracker::projectProgress(
    const Eigen::Vector3d& robot_position,
    double& projected_arc,
    std::size_t& projected_segment) const noexcept
{
  if (points_.size() < 2 || !finitePoint(robot_position)) return false;

  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_arc = progress_m_;
  std::size_t best_segment = segment_index_;
  const std::size_t first_segment = reacquire_requested_ ? 0 : segment_index_;
  const double search_end = progress_m_ +
      std::max(0.0, config_.max_forward_search_m);

  for (std::size_t i = first_segment; i + 1 < points_.size(); ++i)
  {
    if (!reacquire_requested_ && cumulative_length_[i] > search_end)
      break;

    const Eigen::Vector2d start = points_[i].head<2>();
    const Eigen::Vector2d delta =
        points_[i + 1].head<2>() - start;
    const double length_sq = delta.squaredNorm();
    if (length_sq <= kEpsilon) continue;

    const double ratio = clamp(
        (robot_position.head<2>() - start).dot(delta) / length_sq,
        0.0, 1.0);
    const Eigen::Vector2d projection = start + ratio * delta;
    const double distance_sq =
        (robot_position.head<2>() - projection).squaredNorm();
    const double arc = cumulative_length_[i] +
        ratio * std::sqrt(length_sq);
    if (arc + 0.05 < progress_m_)
      continue;

    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      best_arc = arc;
      best_segment = i;
    }
  }

  if (!std::isfinite(best_distance_sq)) return false;
  projected_arc = std::max(progress_m_, best_arc);
  projected_segment = best_segment;
  return true;
}

// sampleAtArc：根据给定累计弧长arc（先限制到[0, 总长度]），二分查找到对应的
// 路径段，在段内线性插值得到位置点（可选输出所在段索引）。
bool RoutePathTracker::sampleAtArc(
    double arc,
    Eigen::Vector3d& point,
    std::size_t* segment) const noexcept
{
  if (points_.size() < 2 || cumulative_length_.empty()) return false;

  const double bounded_arc = clamp(arc, 0.0, cumulative_length_.back());
  auto upper = std::upper_bound(
      cumulative_length_.begin(), cumulative_length_.end(), bounded_arc);
  std::size_t i = upper == cumulative_length_.begin()
      ? 0
      : static_cast<std::size_t>(upper - cumulative_length_.begin() - 1);
  i = std::min(i, points_.size() - 2);

  const double segment_length =
      cumulative_length_[i + 1] - cumulative_length_[i];
  const double ratio = segment_length > kEpsilon
      ? (bounded_arc - cumulative_length_[i]) / segment_length
      : 0.0;
  point = points_[i] + ratio * (points_[i + 1] - points_[i]);
  if (segment) *segment = i;
  return finitePoint(point);
}

// update：根据机器人当前位置/朝向计算本周期的跟踪速度指令。
// 步骤：
// 1. 投影得到当前进度弧长与所在段，并标记已完成一次搜索；
// 2. 计算剩余距离与预看目标弧长（progress_m_ + lookahead）；
// 3. 采样目标点与其前后一小段距离的两个点，用于估计局部切线方向（若失败直接
//    返回无效输出）；
// 4. 若切线接近零向量，回退使用当前段的方向向量；
// 5. 合成世界系目标速度：切线方向分量(最大速度) + 位置误差比例项，若超过最大
//    速度则按比例缩放；
// 6. 根据目标速度方向与当前朝向的夹角误差计算朝向余弦衰减系数，对线速度做衰减
//    （朝向偏差越大越应该减速转向而非直接前进）；
// 7. 接近终点时按剩余距离比例逐渐减速；
// 8. 对线/角速度做最终限幅并填充输出结果。
RoutePathTrackerOutput RoutePathTracker::update(
    const Eigen::Vector3d& robot_position,
    double robot_yaw) noexcept
{
  RoutePathTrackerOutput output{};
  if (!std::isfinite(robot_yaw) || points_.size() < 2) return output;

  double projected_arc = 0.0;
  std::size_t projected_segment = 0;
  if (!projectProgress(robot_position, projected_arc, projected_segment))
    return output;

  progress_m_ = projected_arc;
  segment_index_ = projected_segment;
  reacquire_requested_ = false;

  const double total_length = cumulative_length_.back();
  const double remaining = std::max(0.0, total_length - progress_m_);
  const double target_arc = std::min(
      total_length,
      progress_m_ + std::max(0.0, config_.lookahead_distance_m));

  Eigen::Vector3d target;
  Eigen::Vector3d heading_start;
  Eigen::Vector3d heading_end;
  if (!sampleAtArc(target_arc, target) ||
      !sampleAtArc(std::max(progress_m_, target_arc -
          0.5 * std::max(0.0, config_.heading_lookahead_m)),
          heading_start) ||
      !sampleAtArc(std::min(total_length, target_arc +
          0.5 * std::max(0.0, config_.heading_lookahead_m)),
          heading_end))
    return output;

  Eigen::Vector2d tangent =
      heading_end.head<2>() - heading_start.head<2>();
  if (tangent.norm() <= kEpsilon)
  {
    tangent = points_[segment_index_ + 1].head<2>() -
        points_[segment_index_].head<2>();
  }
  if (tangent.norm() <= kEpsilon) return output;
  tangent.normalize();

  const double max_vx = std::max(0.0, config_.max_vx);
  Eigen::Vector2d velocity_world = max_vx * tangent +
      std::max(0.0, config_.kp_position) *
          (target.head<2>() - robot_position.head<2>());
  const double velocity_norm = velocity_world.norm();
  if (velocity_norm > max_vx && velocity_norm > kEpsilon)
    velocity_world *= max_vx / velocity_norm;

  const double desired_yaw = std::atan2(
      velocity_world.y(), velocity_world.x());
  const double yaw_error = normalizeAngle(desired_yaw - robot_yaw);
  const double heading_scale = std::max(0.0, std::cos(yaw_error));
  double commanded_vx = velocity_world.norm() * heading_scale;

  const double slowdown_distance =
      std::max(0.0, config_.near_goal_slowdown_m);
  if (slowdown_distance > kEpsilon && remaining < slowdown_distance)
  {
    commanded_vx *= clamp(remaining / slowdown_distance, 0.0, 1.0);
  }

  output.vx = clamp(commanded_vx, 0.0, max_vx);
  output.vy = 0.0;
  output.yaw_rate = clamp(
      std::max(0.0, config_.kp_yaw) * yaw_error,
      -std::max(0.0, config_.max_yaw_rate),
      std::max(0.0, config_.max_yaw_rate));
  output.progress_m = progress_m_;
  output.remaining_m = remaining;
  output.target_x = target.x();
  output.target_y = target.y();
  output.valid = std::isfinite(output.vx) &&
      std::isfinite(output.yaw_rate);
  return output;
}

}  // namespace scan_planner_dmq
