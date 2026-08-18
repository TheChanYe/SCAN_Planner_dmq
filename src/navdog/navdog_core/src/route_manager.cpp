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
  constexpr std::size_t kMinRisingRouteSegments = 3;
  RouteElevationAssessment assessment{};
  if (!config.enabled || !hasRoute() || !progress.valid ||
      progress.task_sequence != task_view_.sequence ||
      !std::isfinite(config.lookahead_distance_m) ||
      config.lookahead_distance_m <= 0.0 ||
      !std::isfinite(config.sample_step_m) || config.sample_step_m <= 0.0 ||
      !std::isfinite(config.trigger_rise_m) || config.trigger_rise_m <= 0.0 ||
      !std::isfinite(config.min_local_slope_m_per_m) ||
      config.min_local_slope_m_per_m <= 0.0 ||
      !std::isfinite(config.flat_tolerance_m) ||
      config.flat_tolerance_m < 0.0)
  {
    return assessment;
  }

  navdog_task::RoutePoint current{};
  if (!pointAtArcLength(progress.arc_length_m, current))
    return assessment;

  const double checked_until = std::min(
      progress.total_length_m,
      progress.arc_length_m + config.lookahead_distance_m);
  double max_forward_z = current.z;
  double max_local_slope = 0.0;
  double steep_rise = 0.0;
  double running_max_z = current.z;
  double max_drawdown = 0.0;
  double previous_arc = progress.arc_length_m;
  navdog_task::RoutePoint previous_sample = current;

  const auto assessSample = [&](double arc,
      const navdog_task::RoutePoint& sample) {
    const double ds = arc - previous_arc;
    if (ds > 1e-9)
    {
      const double dz = sample.z - previous_sample.z;
      const double slope = dz / ds;
      max_local_slope = std::max(max_local_slope, slope);
      if (dz > 0.0 &&
          slope + 1e-9 >= config.min_local_slope_m_per_m)
      {
        steep_rise += dz;
      }
    }
    max_forward_z = std::max(max_forward_z, sample.z);
    running_max_z = std::max(running_max_z, sample.z);
    max_drawdown = std::max(max_drawdown, running_max_z - sample.z);
    previous_arc = arc;
    previous_sample = sample;
  };

  for (double arc = progress.arc_length_m + config.sample_step_m;
       arc < checked_until; arc += config.sample_step_m)
  {
    navdog_task::RoutePoint sample{};
    if (!pointAtArcLength(arc, sample))
      return RouteElevationAssessment{};
    assessSample(arc, sample);
  }

  navdog_task::RoutePoint final_sample{};
  if (!pointAtArcLength(checked_until, final_sample))
    return assessment;
  assessSample(checked_until, final_sample);

  // Interpolated samples are useful for measuring rise and drawdown, but one
  // bad source waypoint can turn a single Z jump into many apparently rising
  // samples. Require the rise to be present on at least three distinct source
  // route segments, which means at least four received route points provide
  // the stair evidence. Flat tread segments remain allowed.
  std::size_t rising_route_segments = 0;
  double segment_start_arc = 0.0;
  for (std::size_t i = 1; i < task_view_.points.size(); ++i)
  {
    const auto& segment_start = task_view_.points[i - 1];
    const auto& segment_end = task_view_.points[i];
    const double segment_length = std::hypot(
        segment_end.x - segment_start.x,
        segment_end.y - segment_start.y);
    const double segment_end_arc = segment_start_arc + segment_length;
    if (segment_length <= 1e-12)
      continue;
    if (segment_end_arc <= progress.arc_length_m + 1e-9)
    {
      segment_start_arc = segment_end_arc;
      continue;
    }
    if (segment_start_arc >= checked_until - 1e-9)
      break;

    const double overlap_start = std::max(
        segment_start_arc, progress.arc_length_m);
    const double overlap_end = std::min(segment_end_arc, checked_until);
    const double overlap_length = overlap_end - overlap_start;
    if (overlap_length > 1e-9)
    {
      const double start_ratio =
          (overlap_start - segment_start_arc) / segment_length;
      const double end_ratio =
          (overlap_end - segment_start_arc) / segment_length;
      const double start_z = segment_start.z +
          start_ratio * (segment_end.z - segment_start.z);
      const double end_z = segment_start.z +
          end_ratio * (segment_end.z - segment_start.z);
      const double dz = end_z - start_z;
      if (dz > 0.0 &&
          dz / overlap_length + 1e-9 >=
              config.min_local_slope_m_per_m)
      {
        ++rising_route_segments;
      }
    }
    segment_start_arc = segment_end_arc;
  }

  assessment.valid = true;
  assessment.current_z = current.z;
  assessment.max_forward_z = max_forward_z;
  assessment.rise_m = max_forward_z - current.z;
  assessment.max_local_slope_m_per_m = max_local_slope;
  assessment.steep_rise_m = steep_rise;
  assessment.max_drawdown_m = max_drawdown;
  assessment.checked_until_arc_m = checked_until;
  assessment.ascending =
      assessment.rise_m + 1e-9 >= config.trigger_rise_m &&
      assessment.steep_rise_m + 1e-9 >= 0.5 * config.trigger_rise_m &&
      assessment.max_drawdown_m <= config.flat_tolerance_m + 1e-9 &&
      rising_route_segments >= kMinRisingRouteSegments;
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
