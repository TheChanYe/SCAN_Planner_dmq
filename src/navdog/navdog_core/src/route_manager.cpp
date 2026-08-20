#include "navdog_core/route_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navdog
{

// 构造函数：用进度配置初始化内部进度追踪器。
RouteManager::RouteManager(const RouteProgressConfig& config)
    : progress_tracker_(config) {}

// reset：清空任务视图（路线点集与sequence）、进度追踪器内部状态与缓存的进度结果。
void RouteManager::reset() noexcept
{
  task_view_ = NavigationTask{};
  progress_tracker_.reset();
  last_progress_ = RouteProgress{};
}

// acceptRoute（拷贝版）：校验通过后先 reset() 清除旧状态，再拷贝设置新的 sequence 与路线点集。
bool RouteManager::acceptRoute(std::uint64_t sequence,
    const std::vector<navdog_task::RoutePoint>& points)
{
  if (!canAccept(sequence, points)) return false;
  reset();
  task_view_.sequence = sequence;
  task_view_.points = points;
  return true;
}

// acceptRoute（移动版）：与拷贝版逻辑相同，但用 std::move 避免较大点集的拷贝开销。
bool RouteManager::acceptRoute(std::uint64_t sequence,
    std::vector<navdog_task::RoutePoint>&& points)
{
  if (!canAccept(sequence, points)) return false;
  reset();
  task_view_.sequence = sequence;
  task_view_.points = std::move(points);
  return true;
}

// canAccept：判断能否接受新路线。
// 条件：sequence非零且点集非空；所有点均为有限数（含has_yaw时的yaw）；
// 若已有路线，新sequence不能与旧相同也不能小于旧（防止重复/乱序覆盖）。
bool RouteManager::canAccept(std::uint64_t sequence,
    const std::vector<navdog_task::RoutePoint>& points) const noexcept
{
  if (sequence == 0 || points.empty()) return false;
  for (const auto& point : points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) ||
        (point.has_yaw && !std::isfinite(point.yaw))) return false;
  }
  if (hasRoute() && sequence == task_view_.sequence) return false;
  if (hasRoute() && sequence < task_view_.sequence) return false;
  return true;
}

// hasRoute：当前是否持有一条有效路线（sequence非零且点集非空）。
bool RouteManager::hasRoute() const noexcept
{ return task_view_.sequence != 0 && !task_view_.points.empty(); }

// taskSequence：返回当前路线关联的任务sequence。
std::uint64_t RouteManager::taskSequence() const noexcept
{ return task_view_.sequence; }

// updateProgress：用机器人位置更新路线进度。
// 步骤：1.无路线直接返回默认输出；2.交给progress_tracker_计算本次原始进度；
// 3.若有效，强制与上一次的弧长取最大值（单调不回退不回置，这是保护交叉点/环形路线不震荡
// 的关键不变量），重新计算剩余距离并更新缓存。
RouteProgressOutput RouteManager::updateProgress(
    const RobotState& robot, double now_sec)
{
  if (!hasRoute()) return RouteProgressOutput{};
  RouteProgressOutput output =
      progress_tracker_.update(task_view_, robot, now_sec);
  if (output.progress.valid)
  {
    // This explicit clamp is the invariant protecting crossings and loops.
    output.progress.arc_length_m = std::max(
        last_progress_.valid ? last_progress_.arc_length_m : 0.0,
        output.progress.arc_length_m);
    output.progress.remaining_distance_m = std::max(
        0.0, output.progress.total_length_m - output.progress.arc_length_m);
    last_progress_ = output.progress;
  }
  return output;
}

// pointAtArcLength：查询累计弧长 arc 处的路线点（不改变当前进度）。
// 步骤：1.无路线或arc非法直接返回失败；2.单点路线直接返回该点；3.逐段累加
// 直线段长度，找到目标弧长所在的区间后按比例插值坐标与朝向角（直接用直线段方向，
// 已到达终点且终点自带yaw时用终点自带的yaw）；4.超出总长度则回退为路线最后一点。
bool RouteManager::pointAtArcLength(
    double arc, navdog_task::RoutePoint& output) const noexcept
{
  output = navdog_task::RoutePoint{};
  if (!hasRoute() || !std::isfinite(arc)) return false;
  if (task_view_.points.size() == 1)
  {
    output = task_view_.points.front();
    return true;
  }
  double traversed = 0.0;
  const double target = std::max(0.0, arc);
  for (std::size_t i = 1; i < task_view_.points.size(); ++i)
  {
    const auto& a = task_view_.points[i - 1];
    const auto& b = task_view_.points[i];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length <= 1e-12) continue;
    if (target <= traversed + length)
    {
      const double ratio = std::max(0.0,
          std::min(1.0, (target - traversed) / length));
      output.x = a.x + ratio * (b.x - a.x);
      output.y = a.y + ratio * (b.y - a.y);
      output.z = a.z + ratio * (b.z - a.z);
      output.yaw = std::atan2(b.y - a.y, b.x - a.x);
      output.has_yaw = true;
      if (ratio >= 1.0 && b.has_yaw) output = b;
      return true;
    }
    traversed += length;
  }
  output = task_view_.points.back();
  return true;
}

// forwardTarget：从 from 弧长处继续向前 distance 米，查询对应位置作为跟随目标点，
// 实际上是对 pointAtArcLength(from+distance) 的包装，用于 pure pursuit 前瞻距离计算。
bool RouteManager::forwardTarget(double from, double distance,
    navdog_task::RoutePoint& output) const noexcept
{
  if (!std::isfinite(from) || !std::isfinite(distance) || distance < 0.0)
    return false;
  return pointAtArcLength(from + distance, output);
}

RouteElevationAssessment RouteManager::assessElevation(
    const RouteProgress& progress,
    const StairUpConfig& config) const noexcept
{
  RouteElevationAssessment assessment{};
  if (!config.enabled || !hasRoute() || !progress.valid ||
      progress.task_sequence != task_view_.sequence ||
      !std::isfinite(config.lookahead_distance_m) ||
      config.lookahead_distance_m <= 0.0 ||
      !std::isfinite(config.trigger_rise_m) ||
      config.trigger_rise_m <= 0.0 ||
      !std::isfinite(config.min_average_slope) ||
      config.min_average_slope <= 0.0 ||
      !std::isfinite(config.baseline_lookback_distance_m) ||
      config.baseline_lookback_distance_m <= 0.0 ||
      !std::isfinite(config.baseline_drop_tolerance_m) ||
      config.baseline_drop_tolerance_m < 0.0)
  {
    return assessment;
  }
  if (config.min_consecutive_rising_points <= 0)
    return assessment;
  if (!std::isfinite(progress.segment_ratio) ||
      progress.segment_ratio < 0.0 || progress.segment_ratio > 1.0 ||
      !std::isfinite(progress.arc_length_m))
  {
    return assessment;
  }

  const auto& points = task_view_.points;
  if (points.size() < 2 || progress.segment_index >= points.size() - 1)
    return assessment;

  const auto elevation_z = [&](std::size_t index) noexcept -> double {
    const double raw_z = points[index].z;
    if (!std::isfinite(raw_z) || index == 0 || index + 1 >= points.size())
      return raw_z;

    const double prev_z = points[index - 1].z;
    const double next_z = points[index + 1].z;
    if (!std::isfinite(prev_z) || !std::isfinite(next_z))
      return raw_z;

    const double baseline_z = 0.5 * (prev_z + next_z);
    if (std::fabs(prev_z - next_z) <= config.flat_tolerance_m &&
        std::fabs(raw_z - baseline_z) >= config.trigger_rise_m)
    {
      return baseline_z;
    }
    return raw_z;
  };

  const double z0 = elevation_z(progress.segment_index);
  const double z1 = elevation_z(progress.segment_index + 1);
  if (!std::isfinite(z0) || !std::isfinite(z1))
    return assessment;
  const double current_z =
      z0 + progress.segment_ratio * (z1 - z0);

  assessment.valid = true;
  assessment.current_z = current_z;
  assessment.baseline_z = current_z;
  assessment.ascent_end_z = current_z;
  assessment.checked_until_arc_m = progress.arc_length_m;

  constexpr double kZEpsilon = 1e-9;
  double baseline_z = current_z;
  double lookback_distance = 0.0;
  std::size_t lookback_index = progress.segment_index;
  double lookback_ratio = progress.segment_ratio;
  while (lookback_distance <= config.baseline_lookback_distance_m + kZEpsilon)
  {
    if (lookback_index >= points.size())
      break;
    const double previous_z_sample = elevation_z(lookback_index);
    if (!std::isfinite(previous_z_sample))
      return RouteElevationAssessment{};
    baseline_z = std::max(baseline_z, previous_z_sample);
    if (lookback_index == 0)
      break;

    const auto& segment_start = points[lookback_index - 1];
    const auto& segment_end = points[lookback_index];
    const double segment_length = std::hypot(
        segment_end.x - segment_start.x,
        segment_end.y - segment_start.y);
    if (!std::isfinite(segment_length))
      return RouteElevationAssessment{};
    lookback_distance += segment_length * std::max(0.0, lookback_ratio);
    if (lookback_distance > config.baseline_lookback_distance_m + kZEpsilon)
      break;
    --lookback_index;
    lookback_ratio = 1.0;
  }

  assessment.baseline_z = baseline_z;
  assessment.baseline_drop_m = std::max(0.0, baseline_z - current_z);
  assessment.baseline_consistent =
      assessment.baseline_drop_m <= config.baseline_drop_tolerance_m;
  if (!assessment.baseline_consistent)
  {
    assessment.ascending = false;
    return assessment;
  }

  double distance_ahead = 0.0;
  double current_arc = progress.arc_length_m;
  double previous_arc = current_arc;
  double previous_z = current_z;
  double run_start_z = previous_z;
  double run_start_arc = current_arc;
  int consecutive_rising_points = 0;

  const std::size_t first_future_index = progress.segment_index + 1;
  for (std::size_t i = first_future_index; i < points.size(); ++i)
  {
    const auto& point = points[i];
    const double sample_z = elevation_z(i);
    if (!std::isfinite(sample_z))
      return RouteElevationAssessment{};
    double segment_remaining = 0.0;
    if (i == first_future_index)
    {
      const auto& segment_start = points[progress.segment_index];
      const auto& segment_end = points[progress.segment_index + 1];
      const double segment_length = std::hypot(
          segment_end.x - segment_start.x,
          segment_end.y - segment_start.y);
      segment_remaining = segment_length *
          std::max(0.0, 1.0 - progress.segment_ratio);
    }
    else
    {
      const auto& previous_point = points[i - 1];
      segment_remaining = std::hypot(
          point.x - previous_point.x,
          point.y - previous_point.y);
    }

    if (!std::isfinite(segment_remaining))
      return RouteElevationAssessment{};
    if (segment_remaining <= kZEpsilon)
    {
      consecutive_rising_points = 0;
      run_start_z = sample_z;
      run_start_arc = current_arc;
      assessment.ascent_end_z = sample_z;
      assessment.rise_m = 0.0;
      assessment.average_slope = 0.0;
      assessment.consecutive_rising_points = 0;
      previous_z = sample_z;
      previous_arc = current_arc;
      continue;
    }
    distance_ahead += std::max(0.0, segment_remaining);
    if (distance_ahead > config.lookahead_distance_m + kZEpsilon)
      break;

    current_arc += std::max(0.0, segment_remaining);
    assessment.checked_until_arc_m = current_arc;

    if (sample_z > previous_z + kZEpsilon)
    {
      ++consecutive_rising_points;
      if (consecutive_rising_points == 1)
      {
        run_start_z = previous_z;
        run_start_arc = previous_arc;
      }
      const double run_rise = sample_z - run_start_z;
      const double run_length = current_arc - run_start_arc;
      const double average_slope =
          run_length > kZEpsilon ? run_rise / run_length : 0.0;
      assessment.ascent_end_z = sample_z;
      assessment.rise_m = run_rise;
      assessment.average_slope = average_slope;
      assessment.consecutive_rising_points = consecutive_rising_points;
      if (consecutive_rising_points >=
              config.min_consecutive_rising_points &&
          run_rise + kZEpsilon >= config.trigger_rise_m &&
          average_slope + kZEpsilon >= config.min_average_slope &&
          sample_z - baseline_z + kZEpsilon >=
              config.trigger_rise_m)
      {
        assessment.ascending = true;
        return assessment;
      }
    }
    else
    {
      consecutive_rising_points = 0;
      run_start_z = sample_z;
      run_start_arc = current_arc;
      assessment.ascent_end_z = sample_z;
      assessment.rise_m = 0.0;
      assessment.average_slope = 0.0;
      assessment.consecutive_rising_points = 0;
    }
    previous_z = sample_z;
    previous_arc = current_arc;
  }
  return assessment;
}

// goal：返回路线终点指针，无路线时为nullptr。
const navdog_task::RoutePoint* RouteManager::goal() const noexcept
{ return hasRoute() ? &task_view_.points.back() : nullptr; }

// route：返回当前路线点集的只读引用。
const std::vector<navdog_task::RoutePoint>& RouteManager::route() const noexcept
{ return task_view_.points; }

// progress：返回最近一次成功的进度缓存。
const RouteProgress& RouteManager::progress() const noexcept
{ return last_progress_; }

// taskView：返回兼容旧接口的任务视图（包含sequence与路线点集）。
const NavigationTask& RouteManager::taskView() const noexcept
{ return task_view_; }

}  // namespace navdog
