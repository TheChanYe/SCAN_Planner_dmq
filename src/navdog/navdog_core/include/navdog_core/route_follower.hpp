#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>
#include <vector>

namespace navdog
{

// =============================================================================
// RouteFollower
//
// 沿着原始 NavigationTask 路线生成速度指令（TRACKING 阶段的默认跟踪器）。
// 采用前瞻点附近的局部路线切线作为主方向，并只用横向误差做几何回归。
// RouteFollower 面向真机 forward-motion 跟踪，仅输出 vx+yaw；Native SCAN
// 仍保留 holonomic 轨迹并由 driver 适配。
// =============================================================================

class RouteFollower
{
public:
  // 构造函数：传入跟踪相关配置（比例增益 kp_x/kp_yaw、前瞻距离等）。
  explicit RouteFollower(
      const RouteFollowerConfig& config);

  // update：路线跟踪主入口，每个控制周期调用一次。
  // 输入：
  //   task     - 当前导航任务（完整路线点列）
  //   robot    - 机器人当前位姿与速度状态
  //   progress - 当前路线跟踪进度（已走弧长、总长度等）
  //   max_vx   - 当前允许的最大线速度上限
  //   now_sec  - 当前时间戳（秒）
  // 输出：VelocityCommand，包含 vx/yaw_rate 及有效性标志；vy固定为0。
  // 若路线只有单个点或总长度接近零，会退化为 updatePointGoal 直达模式。
  VelocityCommand update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double now_sec);

private:
  struct TrackingPoint
  {
    double arc_m{0.0};
    double x{0.0};
    double y{0.0};
  };

  // updatePointGoal：单点/极短路线的直达模式，直接朝向任务最后一个点行驶，
  // 不使用前瞻点插值。目标在前半平面时边走边转，后半平面时原地转向。
  VelocityCommand updatePointGoal(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double now_sec) const;

  void rebuildTrackingPath(
      const NavigationTask& task);

  // interpolateTrackingPoint：根据原始路线弧长在派生tracking path上插值位置。
  // 输入：target_arc_m - 原始路线累计弧长（米）
  // 输出：out_x/out_y - 插值得到的坐标；返回值表示是否插值成功
  // （点数少于2个时失败）。若目标弧长超出路线总长，则钳到终点。
  bool interpolateTrackingPoint(
      double target_arc_m,
      double& out_x,
      double& out_y) const noexcept;

  RouteFollowerConfig config_{};    // 跟踪控制配置参数
  bool tracking_path_ready_{false};
  std::uint64_t tracking_task_sequence_{0};
  std::vector<TrackingPoint> tracking_path_;
};

}  // namespace navdog
