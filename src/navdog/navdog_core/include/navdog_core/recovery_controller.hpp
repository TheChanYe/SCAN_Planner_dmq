#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

namespace navdog
{

enum class RecoveryMode { NONE, SIDE_SHIFT, REVERSE };

struct RecoveryOutput
{
  RecoveryMode mode{RecoveryMode::NONE};
  VelocityCommand command{};
  bool active{false};
};

// RecoveryController：脱困/恢复行为控制器。
// 当机器人陷入无法通过常规 Route/SCAN 模式脱离的困境（如被卡住、反复回避失败）时，
// 由上层（NavigationModeManager/NavigationCoordinator）决定进入某种 RecoveryMode，
// 本类负责根据当前恢复模式生成对应的速度指令输出。
// 注意：当前实现是一个占位/透传版本，update() 内部并未包含侧移(SIDE_SHIFT)、
// 倒退(REVERSE)等具体的运动规划逻辑，只是把已设置好的 mode_ 原样包装成输出，
// 真正切换 mode_ 的逻辑目前也还未实现（没有 setter），后续扩展脱困策略时需要在这里补充。
class RecoveryController
{
public:
  // 构造函数：传入全局导航配置（速度限制、安全参数等），供后续脱困策略使用。
  explicit RecoveryController(const NavdogConfig& config) : config_(config) {}

  // reset：将恢复模式清空为 NONE，通常在任务取消/重新开始导航时调用，
  // 避免上一次任务遗留的恢复状态影响新任务。
  void reset() noexcept { mode_ = RecoveryMode::NONE; }

  // update：根据机器人当前状态、障碍物概况和占据栅格查询接口，计算本周期的脱困输出。
  // 输入：
  //   RobotState        - 机器人当前位姿/速度状态（当前实现未使用，预留给未来的脱困算法）
  //   ObstacleSummary    - 周围障碍物概况（当前实现未使用，预留）
  //   OccupancyQuery3D*  - 占据栅格查询接口指针，可为空（当前实现未使用，预留）
  //   now_sec            - 当前时间戳（秒），用于给输出的速度指令打时间戳
  // 输出：
  //   RecoveryOutput，包含当前恢复模式 mode、是否处于激活状态 active、
  //   以及对应的速度指令 command（若未激活则 command.valid=false）
  RecoveryOutput update(const RobotState&, const ObstacleSummary&,
                        const OccupancyQuery3D*, double now_sec) const noexcept;

private:
  NavdogConfig config_{};              // 全局导航配置
  RecoveryMode mode_{RecoveryMode::NONE}; // 当前恢复模式，默认不处于恢复状态
};

}  // namespace navdog
