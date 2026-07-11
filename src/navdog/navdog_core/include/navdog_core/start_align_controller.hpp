#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// =============================================================================
// StartAlignResult
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
// =============================================================================

class StartAlignController
{
public:
  explicit StartAlignController(
      const StartAlignConfig& config = StartAlignConfig{});

  void reset() noexcept;

  StartAlignOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec);

  bool active() const noexcept;

private:
  bool isConfigValid() const noexcept;

  bool isRobotUsable(
      const RobotState& robot) const noexcept;

  bool resolveTargetYaw(
      const NavigationTask& task,
      const RobotState& robot,
      double& target_yaw) const noexcept;

  static double normalizeAngle(
      double angle) noexcept;

  static double degreesToRadians(
      double degrees) noexcept;

  StartAlignConfig config_{};

  bool phase_started_{false};
  bool target_resolved_{false};
  bool rotation_required_{false};

  double phase_started_sec_{0.0};
  double target_yaw_{0.0};
};

}  // namespace navdog
