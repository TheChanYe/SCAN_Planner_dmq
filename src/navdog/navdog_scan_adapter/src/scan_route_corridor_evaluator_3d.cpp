#include "navdog_scan_adapter/scan_route_corridor_evaluator_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog_scan_adapter
{

namespace
{

constexpr double kEpsilon = 1e-9; 

}  // namespace

// 构造函数：保存走廊评估配置与膨胀地图查询接口。
ScanRouteCorridorEvaluator3D::ScanRouteCorridorEvaluator3D(
    const navdog::RouteCorridorConfig& config,
    const std::shared_ptr<InflatedGridQuery3D>& grid)
    : config_(config),
      grid_(grid)
{
}
/**
 * @brief evaluate
 * 沿路线从当前投影位置开始向前逐段采样，检查路径走廊（中心线两侧half_width_m范围）
 * 内是否存在障碍物，从而判断前方路径是否可行。
 * 步骤：
 *   1.校验地图就绪、分辨率/采样步长合法、机器人位姿有效、路线进度有效且任务序号匹配，
 *      任一不满足直接返回默认（无效）结果；
 *   2.初始化评估结果的来源/任务序号/当前弧长/地图分辨率等元信息字段；
 *   3.校验前视距离/半宽配置合法；取前视距离与剩余距离中的较小值作为实际检查距离，
 *      若已到达终点则直接返回未阻塞的有效结果；
 *   4.单点路线与多点路线分别处理：从当前投影位置沿后续路线段逐段采样，
 *      每个采样点在垂直于路径方向的法线上按sample_step步长展开到±half_width_m进行密集采样，
 *      一旦遇到OCCUPIED则标记blocked并记录阻塞位置（前方距离与弧长），遇到OUT_OF_MAP或
 *      INVALID则分别标记相应状态并提前终止；
 *   5.所有补段均无阻塞则返回已检查距离与 blocked=false 的有效结果。
 * @param task 导航任务
 * @param progress 路径进度
 * @param robot 机器人状态
 * @param now_sec 当前时间戳（秒）
 * @return 路径走廊评估结果
 */
navdog::RouteCorridorAssessment
ScanRouteCorridorEvaluator3D::evaluate(
    const navdog::NavigationTask& task,
    const navdog::RouteProgress& progress,
    const navdog::RobotState& robot,
    double now_sec) const
{
  navdog::RouteCorridorAssessment assessment;

  // --- 地图就绪性检查 ---
  if (!grid_ || !grid_->ready())
    return assessment;

  const double resolution = grid_->resolutionM();
  if (!std::isfinite(resolution) || resolution <= 0.0)
    return assessment;

  const double sample_step = resolution * 0.5;
  if (!std::isfinite(sample_step) || sample_step <= 0.0)
    return assessment;

  // --- 机器人位姿有效性检查 ---
  if (!robot.valid)
    return assessment;
  if (!std::isfinite(robot.z))
    return assessment;

  // --- 路线进度有效性与任务序号匹配检查 ---
  if (!progress.valid)
    return assessment;
  if (progress.task_sequence != task.sequence)
    return assessment;

  // --- 初始化评估结果的基础元信息 ---
  assessment.source =
      navdog::RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  assessment.task_sequence = task.sequence;
  assessment.evaluated_from_arc_length_m =
      progress.arc_length_m;
  assessment.map_resolution_m = resolution;
  assessment.sample_step_m = sample_step;
  assessment.map_stamp_sec = grid_->mapStampSec();
  assessment.evaluation_stamp_sec = now_sec;
  assessment.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  assessment.first_blocked_arc_length_m =
      std::numeric_limits<double>::infinity();

  // 前视距离由RouteCorridorConfig统一控制
  // 配置无效时返回默认无效assessment，禁止使用错误距离继续判断。
  if (!std::isfinite(config_.lookahead_distance_m) ||
      config_.lookahead_distance_m <= 0.0 ||
      !std::isfinite(config_.half_width_m) || config_.half_width_m <= 0.0)
  {
    return assessment;
  }

  const double check_distance = std::min(
      config_.lookahead_distance_m,
      progress.remaining_distance_m);
  
  if (check_distance <= 0.0)
  {
    // 已到达终点，无需检查
    assessment.checked_distance_m = 0.0;
    assessment.samples_checked = 0;
    assessment.blocked = false;
    assessment.valid = true;
    return assessment;
  }

  // --- 构建待采样的路径段端点列表 ---
  // 从 progress.projected_x/y 开始沿路线点向前行进。
  //
  // 路段划分：
  //   [0] 投影点 → points[segment_index + 1]
  //   [1] points[segment_index + 1] → points[segment_index + 2]
  //   ...
  //
  // 每个路段都会根据剩余的前视预算被截断。

  double remaining_budget = check_distance;
  double cumulative_distance = 0.0;  // 从进度起点开始累计的已检查距离

  // 当前路段起点
  double cur_x = progress.projected_x;
  double cur_y = progress.projected_y;
  double cur_z = robot.z;
  if (task.points.size() > 1 &&
      progress.segment_index + 1 < task.points.size())
  {
    const navdog::RoutePoint& a = task.points[progress.segment_index];
    const navdog::RoutePoint& b = task.points[progress.segment_index + 1];
    const double ratio = std::max(0.0, std::min(1.0, progress.segment_ratio));
    cur_z = a.z + ratio * (b.z - a.z);
  }
  assessment.query_z_m = cur_z;

  // 辅助lambda：对单个中心点沿垂直于行进方向的法线方向展开采样（走廊宽度方向）。
  // 返回true表示继续评估，返回false表示已命中终止评估（障碍/超图/无效）。
  auto queryPoint = [&](
      double px, double py, double pz,
      double seg_yaw,
      double dist_from_start) -> bool
  {
    const double normal_x = -std::sin(seg_yaw);
    const double normal_y = std::cos(seg_yaw);
    for (double lateral = -config_.half_width_m;
         lateral <= config_.half_width_m + kEpsilon;
         lateral += sample_step)
    {
      ++assessment.samples_checked;
      const InflatedGridQueryResult result = grid_->query(
          px + lateral * normal_x,
          py + lateral * normal_y,
          pz,
          seg_yaw);
      if (result == InflatedGridQueryResult::OCCUPIED)
      {
        assessment.blocked = true;
        assessment.first_blocked_distance_ahead_m = dist_from_start;
        assessment.first_blocked_arc_length_m =
            progress.arc_length_m + dist_from_start;
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = true;
        assessment.out_of_map = false;
        return false;
      }
      if (result == InflatedGridQueryResult::OUT_OF_MAP)
      {
        assessment.out_of_map = true;
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = true;
        return false;
      }
      if (result == InflatedGridQueryResult::INVALID)
      {
        assessment.checked_distance_m = dist_from_start;
        assessment.valid = false;
        return false;
      }
    }
    return true;
  };

  // 处理从(sx, sy)到(ex, ey)的一个路段（带朝向角）。
  // 采样点：起点、中间步进点、终点（受budget截断）。
  // 返回true表示继续，返回false表示已停止。
  auto processSegment = [&](
      double sx, double sy, double sz,
      double ex, double ey, double ez,
      double budget) -> bool
  {
    const double dx = ex - sx;
    const double dy = ey - sy;
    const double dz = ez - sz;
    const double seg_len = std::hypot(dx, dy);

    if (seg_len < kEpsilon)
      return true;  // 退化路段（起终点重合），跳过

    const double seg_yaw = std::atan2(dy, dx);
    const double usable_len = std::min(seg_len, budget);

    // --- 采样起点 ---
    {
      if (!queryPoint(sx, sy, sz, seg_yaw, cumulative_distance))
        return false;
    }

    // --- 采样中间步进点 ---
    if (sample_step < usable_len)
    {
      const int num_steps =
          static_cast<int>(std::floor(usable_len / sample_step));

      for (int i = 1; i <= num_steps; ++i)
      {
        const double t = (i * sample_step) / seg_len;
        // 不要采样超过usable_len
        if (t * seg_len > usable_len + kEpsilon)
          break;

        const double px = sx + t * dx;
        const double py = sy + t * dy;
        const double pz = sz + t * dz;
        const double dist = cumulative_distance + t * seg_len;

        if (!queryPoint(px, py, pz, seg_yaw, dist))
          return false;
      }
    }

    // --- 采样终点（受budget截断） ---
    {
      const double t = usable_len / seg_len;
      const double px = sx + t * dx;
      const double py = sy + t * dy;
      const double pz = sz + t * dz;
      const double dist = cumulative_distance + usable_len;

      if (!queryPoint(px, py, pz, seg_yaw, dist))
        return false;
    }

    cumulative_distance += usable_len;
    return true;
  };

  // --- 单点路线特殊处理 ---
  if (task.points.size() == 1)
  {
    const navdog::RoutePoint& target = task.points.back();
    const double dx = target.x - robot.x;
    const double dy = target.y - robot.y;
    const double robot_to_target = std::hypot(dx, dy);

    if (robot_to_target < kEpsilon)
    {
      // 机器人已在目标点
      assessment.checked_distance_m = 0.0;
      assessment.samples_checked = 0;
      assessment.blocked = false;
      assessment.valid = true;
      return assessment;
    }

    const double seg_yaw = std::atan2(dy, dx);
    const double usable_len =
        std::min(robot_to_target, check_distance);

    // 采样起点（机器人当前位置）
    if (!queryPoint(robot.x, robot.y, robot.z, seg_yaw, 0.0))
    {
      return assessment;
    }

    // 采样中间步进点
    if (sample_step < usable_len)
    {
      const int num_steps =
          static_cast<int>(std::floor(usable_len / sample_step));

      for (int i = 1; i <= num_steps; ++i)
      {
        const double t = (i * sample_step) / robot_to_target;
        if (t * robot_to_target > usable_len + kEpsilon)
          break;

        const double px = robot.x + t * dx;
        const double py = robot.y + t * dy;
        const double pz = robot.z + t * (target.z - robot.z);
        const double dist = t * robot_to_target;

        if (!queryPoint(px, py, pz, seg_yaw, dist))
        {
          return assessment;
        }
      }
    }

    // 采样终点
    {
      const double t = usable_len / robot_to_target;
      const double px = robot.x + t * dx;
      const double py = robot.y + t * dy;
      const double pz = robot.z + t * (target.z - robot.z);
      const double dist = usable_len;

      if (!queryPoint(px, py, pz, seg_yaw, dist))
      {
        return assessment;
      }
    }

    cumulative_distance = usable_len;
    assessment.checked_distance_m = cumulative_distance;
    assessment.blocked = false;
    assessment.valid = true;
    return assessment;
  }

  // --- 多点路线处理 ---
  // 第一段：投影点 → points[segment_index + 1]
  if (progress.segment_index + 1 < task.points.size())
  {
    const navdog::RoutePoint& next_pt =
        task.points[progress.segment_index + 1];

    if (!processSegment(
            cur_x, cur_y, cur_z,
            next_pt.x, next_pt.y, next_pt.z,
            remaining_budget))
    {
      return assessment;
    }

    remaining_budget = check_distance - cumulative_distance;

    cur_x = next_pt.x;
    cur_y = next_pt.y;
    cur_z = next_pt.z;
  }

  // 处理剩余路段
  for (std::size_t i = progress.segment_index + 2;
       i < task.points.size() && remaining_budget > kEpsilon;
       ++i)
  {
    const navdog::RoutePoint& seg_start = task.points[i - 1];
    const navdog::RoutePoint& seg_end = task.points[i];

    if (!processSegment(
            seg_start.x, seg_start.y, seg_start.z,
            seg_end.x, seg_end.y, seg_end.z,
            remaining_budget))
    {
      return assessment;
    }

    remaining_budget = check_distance - cumulative_distance;
    cur_x = seg_end.x;
    cur_y = seg_end.y;
    cur_z = seg_end.z;
  }

  // 所有路段均处理完毕且未发现障碍物
  assessment.checked_distance_m = cumulative_distance;
  assessment.blocked = false;
  assessment.valid = true;
  return assessment;
}

}  // namespace navdog_scan_adapter
