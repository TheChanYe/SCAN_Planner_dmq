#include "navdog_core/recovery_controller.hpp"

namespace navdog
{

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
