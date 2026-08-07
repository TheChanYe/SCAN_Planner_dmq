#include "navdog_scan_adapter/scan_local_planner_adapter.hpp"

#include <bspline_opt/uniform_bspline.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog_scan_adapter
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kSampleDtSec = 0.05;  // 采样轨迹时的时间步长

// 忽略轨迹刚开始的一段时间。LOCAL_AVOID刚开始时机器人自身足印可能已经与
// 保守的膨胀层重叠，若检查t=0时刻会拒绝掉所有逃逸轨迹。
constexpr double kCollisionStartGraceSec = 0.30;  // 碰撞检查忽略的起始宽限时间
constexpr double kCollisionLookAheadSec = 0.05;   // 从当前时刻往后额外预留的提前量

// modeName：将导航模式枚举转为可读字符串，仅用于日志输出。
const char* modeName(navdog::NavigationMode mode) noexcept
{
  switch (mode)
  {
    case navdog::NavigationMode::LOCAL_AVOID:
      return "LOCAL_AVOID";
    default:
      return "NONE";
  }
}

// replanReasonName：将重规划原因枚举转为可读字符串，仅用于日志输出。
const char* replanReasonName(navdog::LocalReplanReason reason) noexcept
{
  switch (reason)
  {
    case navdog::LocalReplanReason::ENTER_AVOID:
      return "ENTER_AVOID";
    case navdog::LocalReplanReason::TRAJECTORY_ENDING:
      return "TRAJECTORY_ENDING";
    case navdog::LocalReplanReason::FUTURE_COLLISION:
      return "FUTURE_COLLISION";
    case navdog::LocalReplanReason::PREVIOUS_FAILED:
      return "PREVIOUS_FAILED";
    case navdog::LocalReplanReason::TASK_CHANGED:
      return "TASK_CHANGED";
    default:
      return "NONE";
  }
}

// finitePoint：检查路点的x/y（及可选的z/yaw）是否均为有限数。
bool finitePoint(const navdog::RoutePoint& point, bool require_z)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      (require_z && !std::isfinite(point.z)))
  {
    return false;
  }
  return !point.has_yaw || std::isfinite(point.yaw);
}

// validRequest：校验局部规划请求是否合法（用途必须为LOCAL_AVOID、任务/规划序号非0、
// 起点/终点位置与速度为有限数、时间戳与最大速度均合法）。
bool validRequest(const navdog::LocalPlanRequest& request)
{
  const bool valid_purpose =
      request.purpose == navdog::NavigationMode::LOCAL_AVOID;
  return request.valid && valid_purpose &&
      request.task_sequence != 0 && request.plan_sequence != 0 &&
      finitePoint(request.start, true) &&
      finitePoint(request.start_vel, false) &&
      finitePoint(request.target, true) &&
      finitePoint(request.target_vel, false) &&
      std::isfinite(request.robot_z) &&
      std::isfinite(request.request_stamp_sec) &&
      request.request_stamp_sec >= 0.0 &&
      std::isfinite(request.max_vx) && request.max_vx > 0.0;
}

}  // namespace

// =============================================================================
// Constructor
// 构造函数：保存规划触发配置、膨胀地图查询接口与SCAN规划器管理器，并启动
// 后台规划工作线程。
// =============================================================================

ScanLocalPlannerAdapter::ScanLocalPlannerAdapter(
    const navdog::PlannerTriggerConfig& config,
    const std::shared_ptr<InflatedGridQuery3D>& grid_query,
    const std::shared_ptr<scan_planner::SCANPlannerManager>&
        planner_manager)
    : config_(config),
      grid_query_(grid_query),
      planner_manager_(planner_manager)
{
  worker_thread_ = std::thread(
      &ScanLocalPlannerAdapter::planningLoop, this);
}

// =============================================================================
// Destructor
// 析构函数：设置关闭标志并清空待处理请求，唤醒工作线程并等待其退出。
// =============================================================================

ScanLocalPlannerAdapter::~ScanLocalPlannerAdapter()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    has_pending_request_ = false;
  }
  cv_.notify_all();

  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
}

// =============================================================================
// requestLocalPlan
// 控制线程提交一个局部规划请求。
// 步骤：1.校验请求合法性与规划器存在；2.加锁后用新请求覆盖待处理请求
// （newest-wins策略）；3.唤醒工作线程处理。
// =============================================================================

bool ScanLocalPlannerAdapter::requestLocalPlan(
    const navdog::LocalPlanRequest& request)
{
  if (!validRequest(request) || !planner_manager_)
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 新请求优先：覆盖任何尚未处理的旧请求。
    pending_request_ = request;
    has_pending_request_ = true;
  }

  cv_.notify_one();
  return true;
}

// =============================================================================
// planningLoop
// 后台规划工作线程主循环。
// 步骤：1.锁定等待直到有待处理请求或收到关闭信号；2.关闭则直接退出；
// 3.取出待处理请求并标记为当前活动请求；4.调用doReboundReplan执行实际重规划；
// 5.成功则采样生成LocalTrajectory，否则使用默认（无效）轨迹；
// 6.写入已完成缓存并打印请求/结果日志。
// =============================================================================

void ScanLocalPlannerAdapter::planningLoop()
{
  while (true)
  {
    navdog::LocalPlanRequest request{};
    bool has_request = false;

    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() {
        return shutdown_ || has_pending_request_;
      });

      if (shutdown_)
        return;

      if (has_pending_request_)
      {
        request = pending_request_;
        has_request = true;
        has_pending_request_ = false;
        active_request_ = request;
        has_active_request_ = true;
      }
    }

    if (!has_request)
      continue;

    ROS_INFO_STREAM(
        "LOCAL_PLAN_REQUEST task=" << request.task_sequence
        << " plan=" << request.plan_sequence
        << " reason=" << replanReasonName(request.reason)
        << " mode=" << modeName(request.purpose)
        << " start=(" << request.start.x << "," << request.start.y << ")"
        << " target=(" << request.target.x << "," << request.target.y << ")");

    bool deterministic_success = false;
    bool random_success = false;
    const bool ok = doReboundReplan(
        request, deterministic_success, random_success);
    const navdog::LocalTrajectory trajectory = ok
        ? sampleLocalTrajData(
              request.task_sequence,
              request.plan_sequence,
              request.purpose,
              request.request_stamp_sec)
        : navdog::LocalTrajectory{};
    storePlanResult(request, trajectory);
    const bool ready = trajectory.valid;

    ROS_INFO_STREAM(
        "LOCAL_PLAN_RESULT task=" << request.task_sequence
        << " plan=" << request.plan_sequence
        << " deterministic="
        << (deterministic_success ? "SUCCESS" : "FAILED")
        << " random="
        << (deterministic_success
                ? "SKIPPED"
                : (random_success ? "SUCCESS" : "FAILED"))
        << " state="
        << (ready ? "READY" : "FAILED")
        << " duration=" << trajectory.duration_sec);
  }
}

// storePlanResult：将本次规划请求与结果写入已完成缓存。
// 步骤：1.若已关闭直接返回；2.清除活动请求标志并记录已完成请求；
// 3.轨迹有效则写入候选轨迹并标记READY，否则标记FAILED。
void ScanLocalPlannerAdapter::storePlanResult(
    const navdog::LocalPlanRequest& request,
    const navdog::LocalTrajectory& trajectory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_)
    return;

  has_active_request_ = false;
  completed_request_ = request;
  if (trajectory.valid)
  {
    completed_candidate_ = trajectory;
    completed_state_ = navdog::LocalPlanState::READY;
  }
  else
  {
    completed_state_ = navdog::LocalPlanState::FAILED;
  }
}

// =============================================================================
// doReboundReplan
// 实际调用SCANPlannerManager::reboundReplan执行一次重规划。
// 步骤：1.无规划器且无测试存根则直接返回失败；2.构造起/终点位置、速度、加速度
// （z统一用robot_z，加速度固定为0）；3.先尝试确定性多项式重规划，成功则直接
// 返回；4.失败则再尝试随机多项式重规划作为兜底，返回最终是否成功。
// =============================================================================

bool ScanLocalPlannerAdapter::doReboundReplan(
    const navdog::LocalPlanRequest& request,
    bool& deterministic_success,
    bool& random_success)
{
  deterministic_success = false;
  random_success = false;

  if (!planner_manager_ && !replan_attempt_for_test_)
    return false;

  Eigen::Vector3d start_pt(
      request.start.x, request.start.y, request.robot_z);
  Eigen::Vector3d start_vel(
      request.start_vel.x, request.start_vel.y, 0.0);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);

  Eigen::Vector3d end_pt(
      request.target.x, request.target.y, request.robot_z);
  Eigen::Vector3d end_vel(
      request.target_vel.x, request.target_vel.y, 0.0);

  const auto attempt = [&](bool random_poly) {
    if (replan_attempt_for_test_)
      return replan_attempt_for_test_(random_poly);
    return planner_manager_->reboundReplan(
        start_pt,
        start_vel,
        start_acc,
        end_pt,
        end_vel,
        true,
        random_poly);
  };

  deterministic_success = attempt(false);
  if (deterministic_success)
    return true;

  random_success = attempt(true);
  return random_success;
}

// =============================================================================
// sampleLocalTrajData
// 从SCANPlannerManager的LocalTrajData（位置/速度B样条）采样生成均匀时间间隔的
// navdog::LocalTrajectory。
// 步骤：1.无规划器或时长非法直接返回空轨迹；2.按kSampleDtSec步长均匀采样，
// 每个采样点调用evaluateDeBoorT得到位置与速度；3.若非末点，用与下一采样点
// 的位移方向近似计算朝向（位移过小则不设置朝向）；4.校验采样后轨迹合法性，
// 合法则标记valid=true并返回，否则返回空（无效）轨迹。
// =============================================================================

navdog::LocalTrajectory ScanLocalPlannerAdapter::sampleLocalTrajData(
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence,
    navdog::NavigationMode purpose,
    double source_stamp_sec)
{
  navdog::LocalTrajectory trajectory{};

  if (!planner_manager_)
    return trajectory;

  scan_planner::LocalTrajData& local_data =
      planner_manager_->local_data_;

  if (!std::isfinite(local_data.duration_) ||
      local_data.duration_ <= kEpsilon)
    return trajectory;

  scan_planner::UniformBspline& pos_traj =
      local_data.position_traj_;
  scan_planner::UniformBspline& vel_traj =
      local_data.velocity_traj_;

  trajectory.task_sequence = task_sequence;
  trajectory.plan_sequence = plan_sequence;
  trajectory.purpose = purpose;
  trajectory.duration_sec = local_data.duration_;
  trajectory.source_stamp_sec = source_stamp_sec;

  const int sample_count = static_cast<int>(
      std::ceil(local_data.duration_ / kSampleDtSec)) + 1;

  for (int i = 0; i < sample_count; ++i)
  {
    const double t = std::min(
        local_data.duration_,
        static_cast<double>(i) * kSampleDtSec);

    Eigen::Vector3d pos = pos_traj.evaluateDeBoorT(t);
    Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(t);

    navdog::TimedTrajectoryPoint point{};
    point.time_from_start_sec = t;
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
    point.vx = vel(0);
    point.vy = vel(1);

    if (i + 1 < sample_count)
    {
      const double t_next = std::min(
          local_data.duration_,
          static_cast<double>(i + 1) * kSampleDtSec);
      Eigen::Vector3d pos_next = pos_traj.evaluateDeBoorT(t_next);
      const double dx = pos_next(0) - pos(0);
      const double dy = pos_next(1) - pos(1);
      if (dx * dx + dy * dy > 1e-6)
      {
        point.yaw = std::atan2(dy, dx);
        point.has_yaw = true;
      }
    }

    trajectory.points.push_back(point);
  }

  if (!isSampledTrajectoryValid(trajectory))
    return navdog::LocalTrajectory{};

  trajectory.valid = true;
  return trajectory;
}

// isSampledTrajectoryValid：校验采样后的轨迹是否合法。
// 步骤：1.时长非法或点数少于2则不合法；2.逐点检查时间单调不递减且各字段均为
// 有限数；3.最后一点的时间必须与总时长基本一致。
bool ScanLocalPlannerAdapter::isSampledTrajectoryValid(
    const navdog::LocalTrajectory& trajectory) noexcept
{
  if (!std::isfinite(trajectory.duration_sec) ||
      trajectory.duration_sec <= 0.0 ||
      trajectory.points.size() < 2)
  {
    return false;
  }

  double previous_time = -1.0;
  for (const auto& point : trajectory.points)
  {
    if (!std::isfinite(point.time_from_start_sec) ||
        point.time_from_start_sec < 0.0 ||
        point.time_from_start_sec + kEpsilon < previous_time ||
        !std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.vx) ||
        !std::isfinite(point.vy) ||
        (point.has_yaw && !std::isfinite(point.yaw)))
    {
      return false;
    }
    previous_time = point.time_from_start_sec;
  }

  return std::abs(trajectory.points.back().time_from_start_sec -
                  trajectory.duration_sec) <= kEpsilon;
}

// =============================================================================
// getLocalTrajectory
// 返回与指定导航模式/任务序号匹配的已完成候选轨迹，不匹配则返回默认（无效）轨迹。
// =============================================================================

navdog::LocalTrajectory ScanLocalPlannerAdapter::getLocalTrajectory(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (completed_candidate_.task_sequence != task_sequence ||
      completed_candidate_.purpose != purpose)
  {
    return navdog::LocalTrajectory{};
  }

  return completed_candidate_;
}

// =============================================================================
// hasValidTrajectory
// 判断当前候选轨迹是否与指定模式/任务匹配且本身有效且时长为正。
// =============================================================================

bool ScanLocalPlannerAdapter::hasValidTrajectory(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (completed_candidate_.task_sequence != task_sequence ||
      completed_candidate_.purpose != purpose)
  {
    return false;
  }

  return completed_candidate_.valid &&
         completed_candidate_.duration_sec > kEpsilon;
}

// =============================================================================
// localPlanState
// 按优先级依次判断：1.若待处理请求匹配三元组（用途/任务/规划序号）返回QUEUED；
// 2.若活动请求匹配返回PLANNING；3.若已完成请求不匹配则返回IDLE；
// 4.否则返回已完成的具体状态（READY/FAILED）。
// =============================================================================

navdog::LocalPlanState ScanLocalPlannerAdapter::localPlanState(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (has_pending_request_ &&
      pending_request_.purpose == purpose &&
      pending_request_.task_sequence == task_sequence &&
      pending_request_.plan_sequence == plan_sequence)
  {
    return navdog::LocalPlanState::QUEUED;
  }

  if (has_active_request_ &&
      active_request_.purpose == purpose &&
      active_request_.task_sequence == task_sequence &&
      active_request_.plan_sequence == plan_sequence)
  {
    return navdog::LocalPlanState::PLANNING;
  }

  if (completed_request_.purpose != purpose ||
      completed_request_.task_sequence != task_sequence ||
      completed_request_.plan_sequence != plan_sequence)
  {
    return navdog::LocalPlanState::IDLE;
  }

  return completed_state_;
}

// =============================================================================
// isTrajectoryColliding
// 加锁后转发至checkTrajectoryCollision。
// =============================================================================

bool ScanLocalPlannerAdapter::isTrajectoryColliding(
    const navdog::LocalTrajectory& trajectory,
    double from_time_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return checkTrajectoryCollision(trajectory, from_time_sec);
}

// =============================================================================
// checkTrajectoryCollision
// 从指定时刻起检查轨迹剩余部分是否与膨胀地图碰撞。
// 步骤：1.地图未就绪直接保守地视为碰撞（返回true）；2.计算实际检查起始时刻
// （取固定忽略宽限与from_time_sec加预留量中的较大值，避免重复拒绝当前机器人
// 自身足印或已执行过的采样点）；3.跳过时间尚未到达该起始时刻的点，对剩余点逐一
// 查询膨胀地图；4.任一采样点为OCCUPIED/OUT_OF_MAP/INVALID均判定为碰撞；
// 5.所有采样点均安全则返回false。
// =============================================================================

bool ScanLocalPlannerAdapter::checkTrajectoryCollision(
    const navdog::LocalTrajectory& trajectory,
    double from_time_sec) const
{
  if (!grid_query_ || !grid_query_->ready())
    return true;

  // Check only the future part of the trajectory. Do not repeatedly
  // reject the current robot footprint or already executed samples.
  const double t_start = std::max(
      kCollisionStartGraceSec,
      from_time_sec + kCollisionLookAheadSec);

  for (const auto& point : trajectory.points)
  {
    if (point.time_from_start_sec < t_start)
      continue;

    const InflatedGridQueryResult result = grid_query_->query(
        point.x,
        point.y,
        point.z,
        point.has_yaw ? point.yaw : 0.0);

    if (result == InflatedGridQueryResult::OCCUPIED ||
        result == InflatedGridQueryResult::OUT_OF_MAP ||
        result == InflatedGridQueryResult::INVALID)
    {
      return true;
    }
  }

  return false;
}

}  // namespace navdog_scan_adapter
