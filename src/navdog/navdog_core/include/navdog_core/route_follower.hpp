#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// RouteFollower
//
// 沿着原始 NavigationTask 路线生成速度指令（TRACKING 阶段的默认跟踪器）。
// 采用类似纯追踪(pure pursuit)的前瞻点策略：在路线上根据当前弧长
// 加上一个随路线可执行速度自适应的前瞻距离找到目标点，朝向该点行驶。
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
  // updatePointGoal：单点/极短路线的直达模式，直接朝向任务最后一个点行驶，
  // 不使用前瞻点插值。目标在前半平面时边走边转，后半平面时原地转向。
  VelocityCommand updatePointGoal(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double now_sec) const;

  // interpolateRoutePoint：根据目标弧长在折线路线上插值出对应位置。
  // 输入：task - 路线点列；target_arc_length_m - 目标累积弧长（米）
  // 输出：out_x/out_y - 插值得到的坐标；返回值表示是否插值成功
  // （点数少于2个时失败）。若目标弧长超出路线总长，则钳到终点。
  bool interpolateRoutePoint(
      const NavigationTask& task,
      double target_arc_length_m,
      double& out_x,
      double& out_y) const noexcept;

  RouteFollowerConfig config_{};    // 跟踪控制配置参数
};

}  // namespace navdog
