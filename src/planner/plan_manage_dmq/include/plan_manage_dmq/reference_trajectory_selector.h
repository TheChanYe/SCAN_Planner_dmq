#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Eigen>

namespace scan_planner_dmq
{

struct ReferenceTrajectorySample
{
  double time_sec{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double arc_length_m{0.0};
};

struct ReferenceTargetWindow
{
  std::size_t progress_index{0};
  std::size_t target_index{0};
  bool valid{false};
};

// 在有序参考轨迹上选择前向窗口：先按路线弧长限制机器人投影范围，再按路线
// 弧长推进目标，防止空间上靠近机器人的后续折返段越序抢占局部目标。
ReferenceTargetWindow selectForwardReferenceWindow(
    const std::vector<ReferenceTrajectorySample>& samples,
    const Eigen::Vector3d& robot_position,
    double projection_search_arc_m,
    double target_distance_m) noexcept;

}  // namespace scan_planner_dmq
