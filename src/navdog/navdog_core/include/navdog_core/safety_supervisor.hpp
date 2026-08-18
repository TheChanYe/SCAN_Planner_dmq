#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// SafetySupervisor
//
// 安全监督层：所有要下发给底盘的速度指令必须最后经过本类的 apply() 处理，
// 它是整个导航链路的最后一道安全阀门。
// 主要职责：
//   1. 有效性/NaN防护：无效或非法数值的指令直接转为安全停止；
//   2. 时效性/新鲜度检测：里程计/障碍物/地图数据过时则停止（防定位退化/感知卡死）；
//   3. Route前方障碍物减速/紧急制动；Native SCAN保留已经碰撞检查过的
//      vx/vy/yaw方向，不被Runtime扇区摘要重复改写；
//   4. 动态限速：转向越大允许的前进速度越小（避免转弯时打滑）；
//   5. 加速度限制：防止速度突变（但安全停止/急停情境下不限制减速）；
//   6. 轨迹身份校验：确保上报的局部轨迹属于当前任务，避免旧轨迹误用。
// =============================================================================

class SafetySupervisor
{
public:
  // 构造函数：传入安全参数（紧急停止距离、减速距离、超时阈值等）
  // 和运动学限幅参数（最大速度、最大加速度等）。
  explicit SafetySupervisor(
      const SafetyConfig& config,
      const LimitConfig& limits);

  // Context：调用 apply() 时需要的安全评估上下文：
  //   robot          - 机器人当前位姿状态（含时间戳，用于时效性检查）
  //   obstacles      - 障碍物概况（最近障碍物距离等）
  //   corridor       - 三维路线走廊评估结果（楼梯期间用于前向安全距离）
  //   trajectory     - 当前局部轨迹（用于轨迹身份校验）
  //   map_stamp_sec  - 占据地图最后更新时间戳
  //   map_valid      - 地图是否有效
  //   prefer_route_corridor_front - 是否以三维路线走廊替代水平前向射线
  struct Context
  {
    RobotState robot{};
    ObstacleSummary obstacles{};
    RouteCorridorAssessment corridor{};
    LocalTrajectory trajectory{};
    double map_stamp_sec{0.0};
    bool map_valid{false};
    bool prefer_route_corridor_front{false};
  };

  // apply：安全监督主入口，每个控制周期对上游控制器输出的原始指令进行安全审查。
  // 输入：
  //   raw_cmd  - 上游（RouteFollower/GoalController/SCAN等）给出的原始速度指令
  //   context  - 当前安全评估上下文（见上方 Context）
  //   max_vx   - 当前任务允许的最大线速度上限
  //   now_sec  - 当前时间戳（秒）
  // 输出：经过安全检查与限幅后的最终 VelocityCommand（保证可以直接下发给底盘）。
  VelocityCommand apply(
      const VelocityCommand& raw_cmd,
      const Context& context,
      double max_vx,
      double now_sec);

  // reset：清空上一周期输出记录（用于加速度限制的参考基准），
  // 在任务重新开始/取消时调用，避免旧速度影响新任务的加速度限制。
  void reset() noexcept;

private:
  // safetyStop：生成一个零速安全指令并同时更新内部上一次输出记录（作为新的基准点）。
  // 输入：now_sec - 当前时间戳；source - 标记这次停止的原因（如 SAFETY_STOP、CANCEL_STOP等）。
  // 输出：零速、有效的 VelocityCommand。
  VelocityCommand safetyStop(
      double now_sec,
      CommandSource source) noexcept;

  // checkTimeouts：检查里程计、障碍物、地图三个数据源的时戳是否都在有效时间窗口内，
  // 任一数据源缺失/过时/未来时间戳异常均返回 false。
  bool checkTimeouts(
      const Context& context,
      double now_sec) const noexcept;

  // checkTrajectoryIdentity：校验上报的局部轨迹是否属于当前有效任务
  // （purpose非NONE且task_sequence非0，且持续时长合法），防止因旧轨迹未清除而误用。
  bool checkTrajectoryIdentity(
      const Context& context) const noexcept;

  // computeFrontSpeedLimit：根据正前方最近障碍物距离计算一个 [0,1] 的前进速度缩放系数
  // （距离<=紧急停止阈值时为0，距离>=减速起始阈值时为无穷大不限制，中间线性插值）。
  double computeFrontSpeedLimit(
      const ObstacleSummary& obstacles) const noexcept;

  // computeYawRateSpeedPenalty：根据当前转向角速度占最大角速度的比例，
  // 计算一个前进速度的惩罚系数（转得越快，允许的前进速度越低，避免转弯打滑）。
  double computeYawRateSpeedPenalty(
      double yaw_rate_cmd,
      double max_vx) const noexcept;

  // shouldApplyAccelerationLimit：判断本周期是否应对加速度进行限制。
  // 安全停止/失败停止/暂停/取消等主动减速场景下不限制（允许立即停下），地图无效时也不限制。
  bool shouldApplyAccelerationLimit(
      const VelocityCommand& raw_cmd,
      const Context& context) const noexcept;

  SafetyConfig safety_config_{};    // 安全相关配置（距离阈值、超时阈值等）
  LimitConfig limit_config_{};      // 运动学限幅配置（最大速度/加速度等）

  VelocityCommand previous_output_{};   // 上一次输出的指令，作为加速度限制的基准
  double previous_stamp_sec_{0.0};      // 上一次输出的时间戳
  bool has_previous_output_{false};     // 是否已有上一次输出记录
};

}  // namespace navdog
