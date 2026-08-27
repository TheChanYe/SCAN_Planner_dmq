#include "navdog_core/route_progress_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{
constexpr double kDistanceTieEpsilon = 1e-12;
}  // namespace

// =============================================================================
// Constructor
// 构造函数：保存进度追踪器配置。
// =============================================================================

RouteProgressTracker::RouteProgressTracker(
    const RouteProgressConfig& config)
    : config_(config)
{
}

// =============================================================================
// reset
// 清空当前活动任务sequence、直线段列表、初始化标志、弧长与当前段索引等全部内部状态。
// =============================================================================

void RouteProgressTracker::reset() noexcept
{
  active_task_sequence_ = 0;
  segments_.clear();

  initialized_ = false;
  single_point_route_ = false;

  total_length_m_ = 0.0;
  current_arc_length_m_ = 0.0;
  forward_arc_budget_m_ = 0.0;
  last_robot_x_ = 0.0;
  last_robot_y_ = 0.0;
  have_last_robot_position_ = false;
  current_segment_vector_index_ = 0;

  last_progress_ = RouteProgress{};
}

// =============================================================================
// initialized
// 返回当前是否已完成首次全路线搜索初始化。
// =============================================================================

bool RouteProgressTracker::initialized() const noexcept
{
  return initialized_;
}

// =============================================================================
// isConfigValid
// 校验最小段长度/最大向前搜索距离/路上容差均为有限且为正数。
// =============================================================================

bool RouteProgressTracker::isConfigValid() const noexcept
{
  if (!std::isfinite(config_.min_segment_length_m) ||
      !std::isfinite(config_.max_forward_search_m) ||
      !std::isfinite(config_.on_route_lateral_tolerance_m))
  {
    return false;
  }

  if (config_.min_segment_length_m <= 0.0 ||
      config_.max_forward_search_m <= 0.0 ||
      config_.on_route_lateral_tolerance_m <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isTaskUsable
// 校验任务可用性：sequence非零、路线点集非空且所有点坐标为有限数。
// =============================================================================

bool RouteProgressTracker::isTaskUsable(
    const NavigationTask& task) const noexcept
{
  if (task.sequence == 0)
  {
    return false;
  }

  if (task.points.empty())
  {
    return false;
  }

  for (const RoutePoint& p : task.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
    {
      return false;
    }
  }

  return true;
}

// =============================================================================
// isRobotUsable
// 校验机器人位姿数据可用性：valid标志为真且x/y为有限数。
// =============================================================================

bool RouteProgressTracker::isRobotUsable(
    const RobotState& robot) const noexcept
{
  if (!robot.valid)
  {
    return false;
  }

  if (!std::isfinite(robot.x) || !std::isfinite(robot.y))
  {
    return false;
  }

  return true;
}

// =============================================================================
// rebuildRoute
// 根据新任务的路线点集重建直线段列表。
// 步骤：1.遍历相邻点对，计算每段方向向量/长度，短于min_segment_length_m的段直接跳过
// （合并到下一段）；2.累加计算总长度。若构建后段列表为空，若点集为空则失败，
// 否则判定为单点路线。最后更新活动sequence并重置初始化/弧长/当前段索引。
// =============================================================================

bool RouteProgressTracker::rebuildRoute(
    const NavigationTask& task)
{
  segments_.clear();
  total_length_m_ = 0.0;
  single_point_route_ = false;

  double cumulative = 0.0;

  for (std::size_t i = 0; i + 1 < task.points.size(); ++i)
  {
    const double x0 = task.points[i].x;
    const double y0 = task.points[i].y;
    const double x1 = task.points[i + 1].x;
    const double y1 = task.points[i + 1].y;

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double length = std::hypot(dx, dy);

    if (length < config_.min_segment_length_m)
    {
      continue;
    }

    Segment seg{};
    seg.original_index = i;
    seg.x0 = x0;
    seg.y0 = y0;
    seg.x1 = x1;
    seg.y1 = y1;
    seg.dx = dx;
    seg.dy = dy;
    seg.length = length;
    seg.cumulative_start_m = cumulative;

    segments_.push_back(seg);
    cumulative += length;
  }

  if (segments_.empty())
  {
    if (task.points.empty())
    {
      return false;
    }

    single_point_route_ = true;
    total_length_m_ = 0.0;
  }
  else
  {
    single_point_route_ = false;
    total_length_m_ = cumulative;
  }

  active_task_sequence_ = task.sequence;
  initialized_ = false;
  current_arc_length_m_ = 0.0;
  forward_arc_budget_m_ = 0.0;
  last_robot_x_ = 0.0;
  last_robot_y_ = 0.0;
  have_last_robot_position_ = false;
  current_segment_vector_index_ = 0;
  last_progress_ = RouteProgress{};

  return true;
}

// =============================================================================
// clamp
// 通用数值限幅工具函数：将 value 限制到 [lower, upper] 区间内。
// =============================================================================

double RouteProgressTracker::clamp(
    double value,
    double lower,
    double upper) noexcept
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

// =============================================================================
// projectToSegment
// 将机器人投影到指定直线段上。
// 先将几何投影限制在线段端点，再检查真实线段投影是否超出本周期
// forward window。超窗候选直接无效，不得夹到窗口边界伪造进度。
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::projectToSegment(
    const Segment& segment,
    std::size_t segment_vector_index,
    const RobotState& robot,
    double minimum_arc_length_m,
    double maximum_arc_length_m) const noexcept
{
  ProjectionCandidate candidate{};
  candidate.valid = false;

  const double length_sq =
      segment.dx * segment.dx +
      segment.dy * segment.dy;

  if (length_sq < kDistanceTieEpsilon)
  {
    return candidate;
  }

  const double raw_ratio =
      ((robot.x - segment.x0) * segment.dx +
       (robot.y - segment.y0) * segment.dy) /
      length_sq;
  const double segment_start_arc = segment.cumulative_start_m;
  const double segment_end_arc = segment.cumulative_start_m + segment.length;
  const double projected_arc =
      segment_start_arc + raw_ratio * segment.length;
  if (std::isfinite(maximum_arc_length_m) &&
      projected_arc > maximum_arc_length_m + kDistanceTieEpsilon)
  {
    return candidate;
  }
  const double segment_ratio = clamp(raw_ratio, 0.0, 1.0);

  const double allowed_min_arc = clamp(
      minimum_arc_length_m,
      segment_start_arc,
      segment_end_arc);
  if (segment_end_arc < allowed_min_arc)
    return candidate;

  const double min_ratio =
      (allowed_min_arc - segment_start_arc) / segment.length;
  const double ratio = std::max(segment_ratio, min_ratio);

  const double arc_length_m =
      segment.cumulative_start_m +
      ratio * segment.length;

  const double projected_x =
      segment.x0 + ratio * segment.dx;
  const double projected_y =
      segment.y0 + ratio * segment.dy;

  const double ddx = robot.x - projected_x;
  const double ddy = robot.y - projected_y;
  const double distance_sq = ddx * ddx + ddy * ddy;

  const double route_yaw =
      std::atan2(segment.dy, segment.dx);

  candidate.valid = true;
  candidate.segment_vector_index = segment_vector_index;
  candidate.original_segment_index = segment.original_index;
  candidate.ratio = ratio;
  candidate.arc_length_m = arc_length_m;
  candidate.projected_x = projected_x;
  candidate.projected_y = projected_y;
  candidate.distance_sq = distance_sq;
  candidate.route_yaw = route_yaw;

  return candidate;
}

// =============================================================================
// isBetterCandidate
// 比较两个投影候选。优先选择距离更近的候选；距离相近（在误差范围内）时，
// 优先选择弧长更小的候选（更靠前的路线位置，用于避免在交叉点处跳跃）。
// =============================================================================

bool RouteProgressTracker::isBetterCandidate(
    const ProjectionCandidate& candidate,
    const ProjectionCandidate& best) const noexcept
{
  if (!best.valid)
  {
    return true;
  }

  if (!candidate.valid)
  {
    return false;
  }

  // Prefer closer projection.
  if (candidate.distance_sq <
      best.distance_sq - kDistanceTieEpsilon)
  {
    return true;
  }

  if (candidate.distance_sq >
      best.distance_sq + kDistanceTieEpsilon)
  {
    return false;
  }

  // Tie: prefer smaller arc length (earlier route position).
  return candidate.arc_length_m < best.arc_length_m;
}

// =============================================================================
// findInitialProjection
// 首次投影：遍历所有直线段（最小弧长限制为0，即不限制方向），选出全局最优投影，
// 支持机器人从任意位置启动导航。
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::findInitialProjection(
    const RobotState& robot) const noexcept
{
  ProjectionCandidate best{};
  best.valid = false;

  for (std::size_t i = 0; i < segments_.size(); ++i)
  {
    ProjectionCandidate candidate =
        projectToSegment(
            segments_[i],
            i,
            robot,
            0.0,
            std::numeric_limits<double>::infinity());

    if (isBetterCandidate(candidate, best))
    {
      best = candidate;
    }
  }

  return best;
}

// =============================================================================
// findForwardProjection
// 非首次调用时的向前搜索。
// 步骤：
//   1. 仅从当前段索引开始向后遍历，超出最大向前搜索距离(max_forward_search_m)则提前终止；
//   2. 对当前所在段，根据已行进的比例计算本段内的最小弧长下限（避免在本段内回退）；
//   3. 对每个候选段调用projectToSegment并用isBetterCandidate选出最优候选；
//   4. 若向前搜索无有效候选，从 current arc 重建当前进度候选：
//      arc不前进，但投影距离/横向误差仍使用本周期机器人位置。
// =============================================================================

RouteProgressTracker::ProjectionCandidate
RouteProgressTracker::findForwardProjection(
    const RobotState& robot) const noexcept
{
  ProjectionCandidate best{};
  best.valid = false;

  const double max_arc = std::min(
      current_arc_length_m_ + config_.max_forward_search_m,
      forward_arc_budget_m_);

  for (std::size_t i = current_segment_vector_index_;
       i < segments_.size();
       ++i)
  {
    const Segment& seg = segments_[i];

    // Skip segments beyond the forward search limit.
    if (seg.cumulative_start_m > max_arc)
    {
      break;
    }

    // Compute minimum arc length for this segment.
    double minimum_arc_length_m = current_arc_length_m_;

    // If this is the current segment, enforce ratio lower
    // bound based on current progress.
    if (i == current_segment_vector_index_ &&
        seg.length > 0.0)
    {
      const double min_ratio =
          (current_arc_length_m_ -
           seg.cumulative_start_m) /
          seg.length;

      if (min_ratio > 0.0)
      {
        minimum_arc_length_m =
            seg.cumulative_start_m +
            min_ratio * seg.length;
      }
    }

    ProjectionCandidate candidate =
        projectToSegment(
            seg,
            i,
            robot,
            minimum_arc_length_m,
            max_arc);

    if (isBetterCandidate(candidate, best))
    {
      best = candidate;
    }
  }

  // If no candidate found in forward search, keep current.
  if (!best.valid &&
      current_segment_vector_index_ < segments_.size())
  {
    const Segment& seg =
        segments_[current_segment_vector_index_];

    if (seg.length > kDistanceTieEpsilon)
    {
      const double ratio = clamp(
          (current_arc_length_m_ - seg.cumulative_start_m) /
              seg.length,
          0.0,
          1.0);
      best.valid = true;
      best.segment_vector_index = current_segment_vector_index_;
      best.original_segment_index = seg.original_index;
      best.ratio = ratio;
      best.arc_length_m = current_arc_length_m_;
      best.projected_x = seg.x0 + ratio * seg.dx;
      best.projected_y = seg.y0 + ratio * seg.dy;
      const double dx = robot.x - best.projected_x;
      const double dy = robot.y - best.projected_y;
      best.distance_sq = dx * dx + dy * dy;
      best.route_yaw = std::atan2(seg.dy, seg.dx);
    }
  }

  return best;
}

// =============================================================================
// makeProgress
// 由选定的投影候选构造完整的 RouteProgress：弧长、总长、剩余距离（非负）、投影点坐标、
// 路线方向角、横向误差（到投影点的距离）与是否在路上（误差不超过容差）。
// =============================================================================

RouteProgress RouteProgressTracker::makeProgress(
    const ProjectionCandidate& candidate,
    const RobotState& /*robot*/,
    double now_sec) const noexcept
{
  RouteProgress progress{};

  progress.task_sequence = active_task_sequence_;

  progress.segment_index =
      candidate.original_segment_index;

  progress.segment_ratio = candidate.ratio;

  progress.arc_length_m = candidate.arc_length_m;

  progress.total_length_m = total_length_m_;

  progress.remaining_distance_m =
      std::max(
          0.0,
          total_length_m_ -
          candidate.arc_length_m);

  progress.projected_x = candidate.projected_x;
  progress.projected_y = candidate.projected_y;

  progress.route_yaw = candidate.route_yaw;

  progress.lateral_error_m =
      std::sqrt(candidate.distance_sq);

  progress.on_route =
      progress.lateral_error_m <=
      config_.on_route_lateral_tolerance_m;

  progress.stamp_sec = now_sec;

  progress.valid = true;

  return progress;
}

// =============================================================================
// makeSinglePointProgress
// 单点路线的特殊进度构造（无需投影）。
// 步骤：1.目标点即为该唯一点，弧长/总长均为0；2.横向误差/剩余距离均为机器人到
// 该点的直线距离；3.路线朝向优先取目标点自带的yaw，否则距离足够远时用机器人指向目标点
// 的方向角，距离太近时回退为机器人当前朝向。
// =============================================================================

RouteProgress RouteProgressTracker::makeSinglePointProgress(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec) const noexcept
{
  RouteProgress progress{};

  const RoutePoint& target = task.points.back();

  progress.task_sequence = task.sequence;

  progress.segment_index = 0;
  progress.segment_ratio = 0.0;

  progress.arc_length_m = 0.0;
  progress.total_length_m = 0.0;

  progress.projected_x = target.x;
  progress.projected_y = target.y;

  progress.lateral_error_m =
      std::hypot(
          robot.x - target.x,
          robot.y - target.y);

  progress.remaining_distance_m =
      progress.lateral_error_m;

  // Route yaw
  if (target.has_yaw && std::isfinite(target.yaw))
  {
    progress.route_yaw = target.yaw;
  }
  else if (progress.lateral_error_m >
           config_.min_segment_length_m)
  {
    progress.route_yaw =
        std::atan2(
            target.y - robot.y,
            target.x - robot.x);
  }
  else
  {
    progress.route_yaw =
        std::isfinite(robot.yaw) ? robot.yaw : 0.0;
  }

  progress.on_route =
      progress.lateral_error_m <=
      config_.on_route_lateral_tolerance_m;

  progress.stamp_sec = now_sec;

  progress.valid = true;

  return progress;
}

// =============================================================================
// update
// 主入口：根据任务与机器人位置更新路线进度。
// 步骤：
//   1.校验时间有限；2.校验配置合法；3.校验任务可用性；
//   4.若任务sequence变化，重建路线直线段缓存；5.校验机器人可用性（不可用则等待）；
//   6.单点路线直接用makeSinglePointProgress构造结果并返回；
//   7.非单点路线时，首次调用用findInitialProjection，后续用findForwardProjection投影；
//   8.强制单调性：将内部累计弧长与本次候选弧长取最大值（交叉点/环形路线处最近点变化也不会
//      让进度回退，这是中心不变量），并更新当前段索引；
//   9.由候选构造并缓存最终的 RouteProgress，返回VALID结果。
// =============================================================================

RouteProgressOutput RouteProgressTracker::update(
    const NavigationTask& task,
    const RobotState& robot,
    double now_sec)
{
  RouteProgressOutput output{};

  // 1. Check time
  if (!std::isfinite(now_sec))
  {
    output.result = RouteProgressResult::INVALID_TIME;
    return output;
  }

  // 2. Check config
  if (!isConfigValid())
  {
    output.result = RouteProgressResult::INVALID_CONFIG;
    return output;
  }

  // 3. Check task
  if (!isTaskUsable(task))
  {
    output.result = RouteProgressResult::INVALID_TASK;
    return output;
  }

  // 4. Rebuild route if sequence changed
  if (task.sequence != active_task_sequence_)
  {
    if (!rebuildRoute(task))
    {
      output.result = RouteProgressResult::INVALID_TASK;
      return output;
    }
  }

  // 5. Check robot
  if (!isRobotUsable(robot))
  {
    output.result =
        RouteProgressResult::WAITING_FOR_ROBOT;
    return output;
  }

  // 6. Single point route
  if (single_point_route_)
  {
    RouteProgress progress =
        makeSinglePointProgress(
            task,
            robot,
            now_sec);

    initialized_ = true;
    last_progress_ = progress;

    output.result = RouteProgressResult::VALID;
    output.progress = progress;
    return output;
  }

  // 7. Projection
  ProjectionCandidate candidate{};

  if (!initialized_)
  {
    candidate = findInitialProjection(robot);
  }
  else
  {
    if (have_last_robot_position_)
    {
      const double robot_displacement = std::hypot(
          robot.x - last_robot_x_,
          robot.y - last_robot_y_);
      if (std::isfinite(robot_displacement) &&
          robot_displacement <= config_.max_forward_search_m)
      {
        forward_arc_budget_m_ = std::min(
            total_length_m_,
            forward_arc_budget_m_ + robot_displacement);
      }
    }
    candidate = findForwardProjection(robot);
  }

  if (!candidate.valid)
  {
    output.result = RouteProgressResult::INVALID_TASK;
    return output;
  }

  // 8. Enforce monotonic progress
  // Update internal state. This explicit clamp is the central invariant:
  // closest-point changes at crossings or loops can never move progress back.
  current_arc_length_m_ =
      std::max(current_arc_length_m_, candidate.arc_length_m);
  if (!initialized_)
  {
    forward_arc_budget_m_ = current_arc_length_m_;
  }
  candidate.arc_length_m = current_arc_length_m_;
  current_segment_vector_index_ =
      candidate.segment_vector_index;
  last_robot_x_ = robot.x;
  last_robot_y_ = robot.y;
  have_last_robot_position_ = true;
  initialized_ = true;

  // 9. Generate RouteProgress
  RouteProgress progress =
      makeProgress(candidate, robot, now_sec);

  last_progress_ = progress;

  output.result = RouteProgressResult::VALID;
  output.progress = progress;

  return output;
}

}  // namespace navdog
