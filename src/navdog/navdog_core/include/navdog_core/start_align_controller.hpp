#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// =============================================================================
// StartAlignResult
// 起点对齐结果枚举。ALIGNED为对齐完成；ALIGNING为正在旋转对齐中；
// WAITING_FOR_ROBOT等待机器人位姿可用；TIMED_OUT为对齐超时；其余为任务/时间/配置无效。
// =============================================================================

enum class StartAlignResult : std::uint8_t
{
  IDLE = 0,
  WAITING_FOR_ROBOT,
  ALIGNING,
  ALIGNED,
  TIMED_OUT,
  INVALID_TASK,
  INVALID_TIME,
  INVALID_CONFIG
};

// =============================================================================
// StartAlignOutput
// update() 的返回结果：结果枚举、实际下发的速度指令、目标朝向角与当前朝向误差、
// 以及是否成功解析出了目标朝向。
// =============================================================================

struct StartAlignOutput
{
  StartAlignResult result{StartAlignResult::IDLE};

  VelocityCommand command{};

  double target_yaw{0.0};
  double yaw_error{0.0};
  bool has_target{false};
};

// =============================================================================
// StartAlignController
// 起点对齐控制器。在进入 TRACKING 之前，让机器人原地旋转朝向路线初始方向（或第一个
// 路线点方向），避免直接走斜线。支持超时判失败。
// =============================================================================

class StartAlignController
{
public:
  // 构造函数：保存对齐配置。
  explicit StartAlignController(
      const StartAlignConfig& config = StartAlignConfig{});

  // reset：清除对齐阶段已开始/目标已解析/需要旋转等内部标志与开始时刻/目标角。
  void reset() noexcept;

  // update：每周期驱动起点对齐逻辑。
  // 输入：task - 当前任务（用于解析目标朝向）；robot - 机器人位姿；now_sec - 当前时间。
  // 输出：包含结果枚举与控制指令的输出结构体。
  StartAlignOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec);

  // active：返回当前对齐阶段是否已开始。
  bool active() const noexcept;

private:
  // isConfigValid：校验对齐配置本身是否合法。
  bool isConfigValid() const noexcept;

  // isRobotUsable：校验机器人位姿是否为合理有限数。
  bool isRobotUsable(
      const RobotState& robot) const noexcept;

  // resolveTargetYaw：根据任务解析对齐目标朝向角（优先路线点自带yaw，否则用机器人
  // 指向第一个路线点的方向角），成功则返回true。
  bool resolveTargetYaw(
      const NavigationTask& task,
      const RobotState& robot,
      double& target_yaw) const noexcept;

  // normalizeAngle：将角度归一化到 (-π, π] 区间。
  static double normalizeAngle(
      double angle) noexcept;

  // degreesToRadians：度数转弧度工具函数。
  static double degreesToRadians(
      double degrees) noexcept;

  StartAlignConfig config_{};  // 对齐配置

  bool phase_started_{false};     // 对齐阶段是否已开始
  bool target_resolved_{false};   // 目标朝向角是否已成功解析
  bool rotation_required_{false}; // 当前朝向与目标朝向差异是否需要实际旋转

  double phase_started_sec_{0.0}; // 对齐阶段开始时刻（用于超时判定）
  double target_yaw_{0.0};        // 目标朝向角
};

}  // namespace navdog
