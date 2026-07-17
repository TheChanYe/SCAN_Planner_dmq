#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// TrajectoryFollower
//
// 跟踪 LocalTrajectory（采样后的 B-spline）。
// 全向底盘支持 vy。
// =============================================================================

class TrajectoryFollower
{
public:
  explicit TrajectoryFollower(
      const TrajectoryFollowerConfig& config);

  void reset() noexcept;

  VelocityCommand update(
      const LocalTrajectory& trajectory,
      const RobotState& robot,
      double max_vx,
      double max_vy,
      double max_yaw_rate,
      NavigationMode expected_mode,
      std::uint64_t expected_task_sequence,
      double now_sec);

  // 返回当前轨迹执行时间（从 0 开始）。
  double trajectoryTimeSec() const noexcept;

private:
  bool sampleTrajectory(
      const LocalTrajectory& trajectory,
      double t_eval,
      double& out_x,
      double& out_y,
      double& out_vx,
      double& out_vy,
      double& out_yaw,
      bool& out_has_yaw) const noexcept;

  TrajectoryFollowerConfig config_{};

  double exec_time_sec_{0.0};
  double last_update_stamp_sec_{0.0};
  bool has_last_update_stamp_{false};

  std::uint64_t active_task_sequence_{0};
  std::uint64_t active_plan_sequence_{0};
  NavigationMode active_purpose_{NavigationMode::NONE};
};

}  // namespace navdog
