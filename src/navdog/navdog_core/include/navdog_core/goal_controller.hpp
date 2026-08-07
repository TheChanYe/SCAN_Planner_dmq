#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

// =============================================================================
// GoalController
//
// 负责机器人接近终点后的收尾行为：判断是否已进入"近终点"区间、
// 在原地做最后的朝向对齐（GOAL_ALIGN，只转向不平移）、以及判定任务是否
// 最终完成（位置+朝向都达标，或者对齐超时兜底）。
// 这是 NAV_STATE 状态机中 TRACKING -> GOAL_ALIGN -> SUCCEEDED 阶段的核心逻辑。
// =============================================================================

class GoalController
{
public:
  // 构造函数：传入终点控制相关配置（近终点距离阈值、对齐超时时间、
  // 原地转向的最大角速度/比例增益等）。
  explicit GoalController(
      const GoalControllerConfig& config);

  // reset：清空对齐计时器状态（对齐是否已开始、开始时刻）。
  // 在任务取消、重新规划或从近终点区间跳出（如目标丢失/重新拉远）时调用，
  // 避免下次进入对齐阶段时误用上一次遗留的计时起点。
  void reset() noexcept;

  // update() 的返回结果：
  //   command        - 本周期要下发的速度指令（原地对齐阶段只有 yaw_rate，无平移）
  //   finished       - 是否已经完成终点对齐（位置和朝向都达标，或对齐超时兜底判成功）
  //   position_lost  - 位置是否丢失/偏离过远（例如中途被推离终点太多），触发后需要重新规划
  //   timed_out      - finished 为 true 时，标记这次完成是否是因为对齐超时而不是真正达标
  struct Result
  {
    VelocityCommand command{};
    bool finished{false};
    bool position_lost{false};
    bool timed_out{false};
  };

  // update：终点对齐主逻辑，每个控制周期调用一次。
  // 输入：
  //   task          - 当前导航任务（用于取最后一个目标点的位置/朝向）
  //   robot         - 机器人当前位姿状态
  //   progress      - 路线跟踪进度（剩余距离、路线朝向等）
  //   max_vx        - 当前允许的最大线速度上限（本函数对齐阶段不平移，暂未使用）
  //   max_yaw_rate  - 当前允许的最大角速度上限
  //   now_sec       - 当前时间戳（秒），用于对齐超时计时
  // 输出：见上方 Result 说明。
  Result update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      double max_vx,
      double max_yaw_rate,
      double now_sec);

  // isNearGoal：根据路线剩余距离判断是否已进入"近终点"区间
  // （remaining_distance_m <= near_goal_switch_dist），供上层状态机决定
  // 是否应该从 TRACKING 切换到 GOAL_ALIGN。
  bool isNearGoal(
      const RouteProgress& progress) const noexcept;

private:
  GoalControllerConfig config_{};       // 终点控制配置参数
  double align_started_sec_{0.0};       // 本轮原地对齐开始的时间戳（秒）
  bool align_timer_active_{false};      // 对齐计时器是否已启动
};

}  // namespace navdog
