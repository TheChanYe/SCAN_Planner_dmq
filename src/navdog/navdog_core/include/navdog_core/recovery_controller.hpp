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

class RecoveryController
{
public:
  explicit RecoveryController(const NavdogConfig& config) : config_(config) {}
  void reset() noexcept { mode_ = RecoveryMode::NONE; }
  RecoveryOutput update(const RobotState&, const ObstacleSummary&,
                        const OccupancyQuery3D*, double now_sec) const noexcept;

private:
  NavdogConfig config_{};
  RecoveryMode mode_{RecoveryMode::NONE};
};

}  // namespace navdog
