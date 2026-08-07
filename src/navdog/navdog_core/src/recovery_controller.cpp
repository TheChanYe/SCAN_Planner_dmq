#include "navdog_core/recovery_controller.hpp"

namespace navdog
{

// update：把当前保存的恢复模式(mode_)包装成一次可被上层直接使用的输出。
// 步骤：
//   1. 把当前 mode_ 原样复制到输出的 mode 字段；
//   2. 只要 mode_ 不是 NONE，就认为本周期恢复行为处于激活状态（active=true）；
//   3. 给输出速度指令打上当前时间戳，方便上层判断指令新鲜度；
//   4. 只有激活状态下 command 才是有效指令（valid=true），来源标记为 RECOVERY；
//      未激活时 command.valid=false、来源为 NONE，上层应忽略该指令。
// 注：RobotState/ObstacleSummary/OccupancyQuery3D 三个输入参数当前均未使用，
// 是为后续实现真正的侧移/倒退等脱困运动规划预留的接口。
RecoveryOutput RecoveryController::update(const RobotState&,
    const ObstacleSummary&, const OccupancyQuery3D*, double now_sec) const noexcept
{
  RecoveryOutput output{};
  output.mode = mode_;
  output.active = mode_ != RecoveryMode::NONE;
  output.command.stamp_sec = now_sec;
  output.command.valid = output.active;
  output.command.source = output.active ? CommandSource::RECOVERY
                                        : CommandSource::NONE;
  return output;
}

}  // namespace navdog
