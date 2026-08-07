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
  // 构造函数：保存局部轨迹跟随的控制参数配置（比例增益、前瞻时间等）。
  explicit TrajectoryFollower(
      const TrajectoryFollowerConfig& config);

  // reset：清零轨迹执行时间与上次更新时刻，并清空当前跟随的任务/规划序号与用途，
  // 用于任务切换或轨迹失效时重新开始跟踪。
  void reset() noexcept;

  // update：每周期驱动局部轨迹跟随，输出一帧速度指令。
  // 输入：trajectory - 待跟踪的局部B样条采样轨迹；robot - 机器人当前位姿；
  //       max_vx/max_vy/max_yaw_rate - 各方向速度上限；expected_mode/expected_task_sequence -
  //       用于校验轨迹是否与当前期望的导航模式和任务一致；now_sec - 当前时间戳。
  // 输出：VelocityCommand速度指令（含是否有效标志、来源标记）。
  // 内部会维护exec_time_sec_作为轨迹的本地执行时钟，按真实dt推进。
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
  // sampleTrajectory：按给定时间t_eval在轨迹点序列中插值采样，得到该时刻的期望位置/
  // 速度/朝向。t_eval小于等于0取首点，超出总时长取末点，否则在相邻两点间线性插值
  // （朝向按角度差归一化后插值）。轨迹为空时返回false。
  bool sampleTrajectory(
      const LocalTrajectory& trajectory,
      double t_eval,
      double& out_x,
      double& out_y,
      double& out_vx,
      double& out_vy,
      double& out_yaw,
      bool& out_has_yaw) const noexcept;

  TrajectoryFollowerConfig config_{};  // 跟随控制参数配置

  double exec_time_sec_{0.0};  // 当前轨迹的本地执行时钟（从0开始随dt推进）
  double last_update_stamp_sec_{0.0};  // 上一次update()调用的时间戳，用于计算dt
  bool has_last_update_stamp_{false};  // 是否已有上一次时间戳记录

  std::uint64_t active_task_sequence_{0};  // 当前正在跟踪的任务序号
  std::uint64_t active_plan_sequence_{0};  // 当前正在跟踪的规划序号
  NavigationMode active_purpose_{NavigationMode::NONE};  // 当前跟踪轨迹对应的导航模式
};

}  // namespace navdog
